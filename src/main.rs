#![no_main]
#![no_std]

use core::cell::RefCell;
use core::pin::Pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
use futures::prelude::*;
use futures::select_biased;

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
const SAMPLE_RATE: usize = 500_000;
// the timer won't behave correctly if the sample rate is not an exact integer number of ticks
static_assertions::const_assert_eq!(TIMER_CLOCK_RATE % SAMPLE_RATE, 0);
// nor if it takes more than 16 bits to represent the delay
static_assertions::const_assert!(TIMER_CLOCK_RATE / SAMPLE_RATE <= 65536);

static INTERRUPTED: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

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

    let mut usb_command = UsbCommand::new(device, serial).fuse();

    let mut signal_generator =
        SignalGenerator::new(peripherals.DAC, peripherals.DMA1, peripherals.TIM4);
    // since this &mut shadows the name of the underlying object, (a) the reference will be used
    // instead, forcing (b) the borrow checker to vehemently oppose anything ever moving the
    // underlying object.
    let signal_generator = &mut signal_generator;

    poll_interrupt(async move {
        loop {
            select_biased! {
                _ = signal_generator.flip_buffer().fuse() => {}
                cmd = usb_command.next() => do_line(&(cmd.expect("nothing ends the stream")), signal_generator),
            }
        }
    })
}

fn do_line(line: &[u8], signal_generator: &mut SignalGenerator) {
    let value_bytes = &line[1..];
    if value_bytes.iter().all(|c| (b'0'..=b'9').contains(c)) {
        // SAFETY: b'0'..=b'9' are all valid UTF-8 bytes
        let value = unsafe { core::str::from_utf8_unchecked(value_bytes) };
        // SAFETY: 7 decimal digits will always fit in a usize
        let value = unsafe { value.parse().unwrap_unchecked() };

        // execute the command that we just parsed; the first character tells us what we
        // should change
        match line[0] {
            b'f' => signal_generator.set_frequency(value),
            b'v' => signal_generator.set_mvpp(value),
            b'o' => signal_generator.set_mvoff(value),
            _ => {}
        };
    }
}

#[cortex_m_rt::interrupt]
fn OTG_FS() {
    interrupt_free(|cs| INTERRUPTED.borrow(cs).replace(true));
    // the peripheral will continue asserting the interrupt until it is poll()ed, so mask it here to
    // avoid an infinite loop.  It needs to be unmasked before calling WFI.
    stm32::NVIC::mask(interrupt::OTG_FS);
}

#[cortex_m_rt::interrupt]
fn DMA1_STREAM5() {
    interrupt_free(|cs| INTERRUPTED.borrow(cs).replace(true));
    // the peripheral will continue asserting the interrupt until it is poll()ed, so mask it here to
    // avoid an infinite loop.  It needs to be unmasked before calling WFI.
    stm32::NVIC::mask(interrupt::DMA1_STREAM5);
}

fn poll_interrupt<F>(mut future: F) -> F::Output
where
    F: core::future::Future,
{
    fn nope<T>(_: *const ()) -> T {
        unreachable!()
    }

    static VTABLE: RawWakerVTable = RawWakerVTable::new(nope, nope, nope, nope);
    let raw_waker = RawWaker::new(core::ptr::null(), &VTABLE);
    let waker = unsafe { Waker::from_raw(raw_waker) };

    let mut cx = Context::from_waker(&waker);

    loop {
        let polled = interrupt_free(|cs| -> Poll<F::Output> {
            // the interrupt needs to be masked no matter what happens before we re-enable
            // interrupts
            unsafe {
                stm32::NVIC::unmask(interrupt::OTG_FS);
                stm32::NVIC::unmask(interrupt::DMA1_STREAM5);
            }

            if INTERRUPTED.borrow(cs).replace(false) {
                // TODO: what does the Waker even need to do here; for know I know everything needs
                // to be polled if and only if the USB interrupt fires
                // unsafe: I don't even move `future` so this is fine, right?
                return unsafe { Pin::new_unchecked(&mut future) }.poll(&mut cx);
            }

            cortex_m::asm::wfi();
            Poll::Pending
        });
        match polled {
            Poll::Ready(x) => return x,
            Poll::Pending => {}
        }
    }
}

struct SignalGenerator {
    samples: [[u16; 20000]; 2],
    dac: stm32::DAC,
    dma: stm32::DMA1,
    hz: usize,
    mvpp: usize,
    mvoff: usize,
    periods: u32,
    last_buffer_was_zero: bool,
}

impl SignalGenerator {
    pub fn new(dac: stm32::DAC, dma: stm32::DMA1, timer: stm32::TIM4) -> Self {
        // subtract one because the timer iterates from zero through (and including) this value.
        timer
            .arr
            .write(|w| w.arr().bits((TIMER_CLOCK_RATE / SAMPLE_RATE - 1) as u16));
        timer.cr2.write(|w| w.mms().update()); // send a TRGO event when the timer updates
        timer.cr1.write(|w| w.cen().set_bit());

        let mut me = Self {
            samples: [[0; 20000]; 2],
            dac,
            dma,
            hz: 1000,
            mvpp: 2_700,
            mvoff: 1_500,
            periods: 0,
            last_buffer_was_zero: true,
        };
        me.update();
        me
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
        self.periods = 0;
        self.fill_buffer();

        // DAC is stream 5, channel 7
        let stream = &self.dma.st[5];
        // first, disable the stream so the addresses can be updated
        stream.cr.write(|w| w.en().clear_bit());
        // and wait for anything in progress to finish
        while stream.cr.read().en().bit() {}

        // from and to address
        stream
            .par
            .write(|w| unsafe { w.bits(&self.dac.dhr12r1 as *const _ as u32) });
        stream
            .m0ar
            .write(|w| unsafe { w.bits(&self.samples[0][0] as *const _ as u32) });

        // since we'll have the memory transferred a u16 at a time, the number of samples is the
        // number of memory transactions
        stream
            .ndtr
            .write(|w| w.ndt().bits(self.samples[0].len() as u16));
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
            w.dir().memory_to_peripheral();
            w.pfctrl().dma(); // DMA controller sets the buffer size
            w.tcie().enabled(); // enable interrupts when the transfer is complete
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

    fn fill_buffer(&mut self) {
        use micromath::F32Ext;

        let samples = match self.last_buffer_was_zero {
            true => &mut self.samples[0], // we are about to start sending buffer 1, so overwrite buffer 0
            false => &mut self.samples[1],
        };

        let vpp = self.mvpp as f32 / 1000.0;
        let amplitude = 0x1000 /* 12 bits */ as f32 * (vpp / 2.0) / DAC_VOLTAGE;
        let off = self.mvoff as f32 / 1000.0;
        let offset = 0x1000 /* 12 bits */ as f32 * off / DAC_VOLTAGE;

        for i in 0..samples.len() {
            samples[i] = ((amplitude
                * (2.0
                    * core::f32::consts::PI
                    * self.hz as f32
                    * (i + self.periods as usize * samples.len()) as f32
                    / SAMPLE_RATE as f32)
                    .sin())
                + offset) as u16;
        }

        self.periods += 1;
    }

    async fn flip_buffer(&mut self) {
        WaitForDMA { dma: &mut self.dma }.await;

        let stream = &self.dma.st[5];

        let next_buffer = match self.last_buffer_was_zero {
            true => &self.samples[1],
            false => &self.samples[0],
        };

        stream
            .m0ar
            .write(|w| unsafe { w.bits(next_buffer as *const _ as u32) });
        stream
            .ndtr
            .write(|w| w.ndt().bits(next_buffer.len() as u16));
        stream.cr.modify(|_r, w| w.en().enabled());

        self.fill_buffer();

        let status = self.dma.hisr.read();
        let dac_status = self.dac.sr.read();

        // if fill_buffer() takes longer than 20,000 samples' worth of time, then the DAC may
        // *still* underrun its buffer.  Make that loud for now.
        if status.teif5().bit() || status.dmeif5().bit() || dac_status.dmaudr1().bit() {
            panic!("dma: {:x?}, dac {:x?}", status.bits(), dac_status.bits());
        }

        self.last_buffer_was_zero = !self.last_buffer_was_zero;
    }
}

struct UsbCommand<'a, T: usb_device::bus::UsbBus> {
    buffer: CommandDeque,
    usb_device: UsbDevice<'a, T>,
    serial_class: usbd_serial::SerialPort<'a, T>,
}

impl<'a, T: usb_device::bus::UsbBus> UsbCommand<'a, T> {
    pub fn new(usb_device: UsbDevice<'a, T>, serial_class: usbd_serial::SerialPort<'a, T>) -> Self {
        Self {
            buffer: CommandDeque::new(),
            usb_device,
            serial_class,
        }
    }

    fn split_borrow(&mut self) -> (&mut UsbDevice<'a, T>, &mut usbd_serial::SerialPort<'a, T>) {
        (&mut self.usb_device, &mut self.serial_class)
    }
}

impl<'a, T: usb_device::bus::UsbBus> Stream for UsbCommand<'a, T> {
    type Item = heapless::Vec<u8, 8>;

    fn poll_next(mut self: Pin<&mut Self>, _cx: &mut Context) -> Poll<Option<Self::Item>> {
        let mut read_buffer = [0; 8];

        // workaround: the Pin<> wrapper apparently confuses disjoint borrows:
        // error[E0499]: cannot borrow `self` as mutable more than once at a time
        //     --> src/main.rs:314:45
        //     |
        // 314 |         if !self.usb_device.poll(&mut [&mut self.serial_class]) {
        //     |             ----            ----            ^^^^ second mutable borrow occurs here
        //     |             |               |
        //     |             |               first borrow later used by call
        //     |             first mutable borrow occurs here
        let (usb_device, serial_class) = self.split_borrow();
        if !usb_device.poll(&mut [serial_class]) {
            return Poll::Pending;
        }

        let count = match self.serial_class.read(&mut read_buffer).ok() {
            Some(count) => count,
            None => return Poll::Pending,
        };

        if count == 0 {
            // why is read() returning me Ok(0), it should be WouldBlock in this case...
            return Poll::Pending;
        }

        self.buffer
            .extend_back(read_buffer.iter().take(count).cloned());
        if let Some(newline) = self.buffer.iter().position(|&c| c == b'\n') {
            return Poll::Ready(Some(
                self.buffer.drain(..newline + 1).take(newline).collect(),
            ));
        }

        Poll::Pending
    }
}

struct WaitForDMA<'a> {
    dma: &'a mut stm32::DMA1,
}

impl<'a> Future for WaitForDMA<'a> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<()> {
        let sr = self.dma.hisr.read();
        if sr.tcif5().bit_is_set() {
            self.dma.hifcr.write(|w| w.ctcif5().set_bit());
            return Poll::Ready(());
        }

        Poll::Pending
    }
}
