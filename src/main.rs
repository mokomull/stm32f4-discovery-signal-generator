#![no_main]
#![no_std]

use core::cell::RefCell;
use core::cmp::min;

use cortex_m_rt::entry;
use panic_itm as _;

use stm32f4xx_hal::prelude::*;
use usb_device::prelude::*;

use cortex_m::interrupt::free as interrupt_free;
use cortex_m::interrupt::Mutex;

use stm32f4xx_hal::stm32;

// the "interrupt" name is required to be in this namespacefor the cortex_m_rt::interrupt macro
use stm32::interrupt;

type CommandDeque = arraydeque::ArrayDeque<[u8; 8], arraydeque::behavior::Wrapping>;

const DAC_VOLTAGE: f32 = 3.0;

// 84MHz, since I suppose the APBx prescaler causes the timer clock to be doubled...
const TIMER_CLOCK_RATE: usize = 84_000_000;
const SAMPLE_RATE: usize = 10_500_000;
// the timer won't behave correctly if the sample rate is not an exact integer number of ticks
static_assertions::const_assert_eq!(TIMER_CLOCK_RATE % SAMPLE_RATE, 0);
// nor if it takes more than 16 bits to represent the delay
static_assertions::const_assert!(TIMER_CLOCK_RATE / SAMPLE_RATE <= 65536);

static USB_EVENT: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[entry]
fn main() -> ! {
    let peripherals = stm32::Peripherals::take().unwrap();

    let rcc = peripherals.RCC.constrain();
    let _clocks = rcc.cfgr.use_hse(8.mhz()).sysclk(168.mhz()).freeze();

    let porta = peripherals.GPIOA.split();

    // the DAC overrides what was selected in the GPIO module, but the datasheet recommended the pin
    // be switched to analog input.
    let _signal_out = porta.pa4.into_analog();

    let usb = stm32f4xx_hal::otg_fs::USB {
        usb_global: peripherals.OTG_FS_GLOBAL,
        usb_device: peripherals.OTG_FS_DEVICE,
        usb_pwrclk: peripherals.OTG_FS_PWRCLK,
        pin_dp: porta.pa12.into_alternate_af10(),
        pin_dm: porta.pa11.into_alternate_af10(),
    };

    static mut USB_BUF: [u32; 32] = [0; 32];

    let bus = stm32f4xx_hal::otg_fs::UsbBus::new(usb, unsafe { &mut USB_BUF });
    let serial = usbd_serial::SerialPort::new(&bus);
    let device = UsbDeviceBuilder::new(&bus, UsbVidPid(0x1337, 0xd00d))
        .manufacturer("Matt Mullins")
        .product("STM32F4 experiment")
        .build();

    // enable the DAC peripheral
    unsafe {
        let rcc = &*stm32f4xx_hal::stm32::RCC::ptr();
        rcc.apb1enr.modify(|_r, w| {
            w.dacen().set_bit();
            w.tim4en().set_bit()
        });
        rcc.ahb1enr.modify(|_r, w| w.dma1en().set_bit());
    }

    let signal_generator =
        SignalGenerator::new(peripherals.DAC, peripherals.DMA1, peripherals.TIM4);
    // do not use signal_generator before passing it to usb_command; it prevents the SignalGenerator
    // from being optimized to constructed in-place, which causes both this one AND the one inside
    // usb_command to be allocated on the stack.  SignalGenerator is too large to fit in memory
    // twice.

    let mut usb_command = UsbCommand::new(device, serial, signal_generator);

    // make sure the signal generator sample memory has been initialized; we have to do this here,
    // because the SignalGenerator can only be used after it's been moved to its final resting
    // place.  We're still in the same module, so the lack of `pub` is a mere suggestion ;)
    usb_command.signal_generator.update();

    loop {
        interrupt_free(|cs| {
            if USB_EVENT.borrow(cs).replace(false) {
                usb_command.poll();
            }

            unsafe {
                stm32::NVIC::unmask(interrupt::OTG_FS);
            }

            cortex_m::asm::wfi();
        });
    }
}

#[cortex_m_rt::interrupt]
fn OTG_FS() {
    interrupt_free(|cs| USB_EVENT.borrow(cs).replace(true));
    // the peripheral will continue asserting the interrupt until it is poll()ed, so mask it here to
    // avoid an infinite loop.  It needs to be unmasked before calling WFI.
    stm32::NVIC::mask(interrupt::OTG_FS);
}

struct SignalGenerator {
    samples: [u16; 42000],
    dac: stm32::DAC,
    dma: stm32::DMA1,
    hz: usize,
    mvpp: usize,
    mvoff: usize,
}

impl SignalGenerator {
    pub fn new(dac: stm32::DAC, dma: stm32::DMA1, timer: stm32::TIM4) -> Self {
        // subtract one because the timer iterates from zero through (and including) this value.
        timer
            .arr
            .write(|w| w.arr().bits((TIMER_CLOCK_RATE / SAMPLE_RATE - 1) as u16));
        timer.cr2.write(|w| w.mms().update()); // send a TRGO event when the timer updates
        timer.cr1.write(|w| w.cen().set_bit());

        Self {
            samples: [0; 42000],
            dac,
            dma,
            hz: 1000,
            mvpp: 2_700,
            mvoff: 1_500,
        }
    }

    pub fn set_frequency(&mut self, hz: usize) {
        self.hz = hz;
        self.update();
    }

    pub fn set_mvpp(&mut self, mvpp: usize) {
        self.mvpp = mvpp;
        self.update();
    }

    pub fn set_mvoff(&mut self, mvoff: usize) {
        self.mvoff = mvoff;
        self.update();
    }

    fn update(&mut self) {
        // DAC is stream 5, channel 7
        let stream = &self.dma.st[5];
        // first, disable the stream so the addresses can be updated
        stream.cr.write(|w| w.en().clear_bit());
        // and wait for anything in progress to finish
        while stream.cr.read().en().bit() {}

        // calculate the new samples to be sent
        let loop_samples = update_frequency(&mut self.samples, self.hz, self.mvpp, self.mvoff);

        // from and to address
        stream
            .par
            .write(|w| unsafe { w.bits(&self.dac.dhr12r1 as *const _ as u32) });
        stream
            .m0ar
            .write(|w| unsafe { w.bits(&self.samples[0] as *const _ as u32) });
        // since we'll have the memory transferred a u16 at a time, the number of samples is the
        // number of memory transactions
        stream.ndtr.write(|w| w.ndt().bits(loop_samples as u16));
        // clear all the stream 5 bits from the status register -- the datasheet says they need to
        // be clear before enabling the stream
        self.dma.hifcr.write(|w| {
            w.ctcif5().set_bit();
            w.chtif5().set_bit();
            w.cteif5().set_bit();
            w.cdmeif5().set_bit();
            w.cfeif5().set_bit()
        });
        // enable the DMA channel
        stream.cr.write(|w| {
            w.chsel().bits(7); // channel 7 on stream 5 is DAC1
            w.mburst().single(); // single-word transfers
            w.pburst().single();
            w.dbm().disabled(); // no need for double-buffering
            w.msize().bits16(); // each transfer is 16-bits
            w.psize().bits16();
            w.minc().incremented(); // increment the memory address we're reading from each cycle
            w.pinc().fixed(); // but keep the peripheral register the same
            w.circ().enabled(); // circular mode means I don't have to keep refilling the buffer
            w.dir().memory_to_peripheral();
            w.pfctrl().dma(); // DMA controller sets the buffer size
            w.en().enabled() // enable the DMA stream!
        });
        // and tell the DAC to trigger DMA transfers
        self.dac.cr.write(|w| {
            w.dmaen1().enabled();
            // the datasheet values for TSEL do not match the stm32f407 crate, so I'm trusting the datasheet.
            unsafe { w.tsel1().bits(5) }; // timer 4 trigger
            w.ten1().enabled();
            w.en1().set_bit()
        });
    }
}

#[inline(never)]
fn update_frequency(samples: &mut [u16], hz: usize, mvpp: usize, mvoff: usize) -> usize {
    use micromath::F32Ext;

    let loop_samples = min(SAMPLE_RATE / hz, samples.len());
    let vpp = mvpp as f32 / 1000.0;
    let amplitude = 0x1000 /* 12 bits */ as f32 * (vpp / 2.0) / DAC_VOLTAGE;
    let off = mvoff as f32 / 1000.0;
    let offset = 0x1000 /* 12 bits */ as f32 * off / DAC_VOLTAGE;
    for i in 0..loop_samples {
        samples[i] = ((amplitude * (2.0 * 3.141592653589 * i as f32 / loop_samples as f32).sin())
            + offset) as u16;
    }

    loop_samples
}

struct UsbCommand<'a, T: usb_device::bus::UsbBus> {
    buffer: CommandDeque,
    usb_device: UsbDevice<'a, T>,
    serial_class: usbd_serial::SerialPort<'a, T>,
    signal_generator: SignalGenerator,
}

impl<'a, T: usb_device::bus::UsbBus> UsbCommand<'a, T> {
    pub fn new(
        usb_device: UsbDevice<'a, T>,
        serial_class: usbd_serial::SerialPort<'a, T>,
        signal_generator: SignalGenerator,
    ) -> Self {
        Self {
            buffer: CommandDeque::new(),
            usb_device,
            serial_class,
            signal_generator,
        }
    }

    pub fn poll(&mut self) {
        let mut read_buffer = [0; 8];

        if let Some(count) = self.check_for_serial_bytes(&mut read_buffer) {
            if count == 0 {
                // why is read() returning me Ok(0), it should be WouldBlock in this case...
                return;
            }

            self.buffer
                .extend_back(read_buffer.iter().take(count).cloned());
            if let Some(newline) = self.buffer.iter().position(|&c| c == b'\n') {
                let value_bytes = self.buffer.iter().skip(1).take(newline - 1);
                if value_bytes.clone().all(|&c| c >= b'0' && c <= b'9') {
                    let mut value: usize = 0;
                    for &c in value_bytes {
                        value *= 10;
                        value += (c - b'0') as usize;
                    }

                    // execute the command that we just parsed; the first character tells us what we
                    // should change
                    match self.buffer.front() {
                        Some(b'f') => self.signal_generator.set_frequency(value),
                        Some(b'v') => self.signal_generator.set_mvpp(value),
                        Some(b'o') => self.signal_generator.set_mvoff(value),
                        _ => (),
                    };
                }
                self.buffer.clear();
            }
        }
    }

    fn check_for_serial_bytes(&mut self, read_buffer: &mut [u8]) -> Option<usize> {
        if self.usb_device.poll(&mut [&mut self.serial_class]) {
            return self.serial_class.read(read_buffer).ok();
        }

        None
    }
}
