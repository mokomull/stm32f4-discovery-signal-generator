#![no_main]
#![no_std]

use core::cmp::{max, min};

use cortex_m::iprintln;
use cortex_m_rt::entry;
use panic_itm as _;

use stm32f4xx_hal::prelude::*;
use usb_device::prelude::*;

const SAMPLE_RATE: usize = 400_000;

#[entry]
fn main() -> ! {
    let peripherals = stm32f407g_disc::Peripherals::take().unwrap();
    let mut core_peripherals = cortex_m::Peripherals::take().unwrap();

    let rcc = peripherals.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.mhz()).sysclk(168.mhz()).freeze();

    let itm = &mut core_peripherals.ITM.stim[0];

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
    let mut serial = usbd_serial::SerialPort::new(&bus);
    let mut device = UsbDeviceBuilder::new(&bus, UsbVidPid(0x1337, 0xd00d))
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
    let dac = peripherals.DAC;
    let timer = peripherals.TIM4;
    let dma = peripherals.DMA1;
    // 84MHz (since I suppose the APBx prescaler causes the timer clock to be doubled) / 400ksps;
    // subtract one because the timer iterates from zero through (and including) this value.
    timer.arr.write(|w| w.arr().bits(210 - 1));
    timer.cr2.write(|w| w.mms().update()); // send a TRGO event when the timer updates
    timer.cr1.write(|w| w.cen().set_bit());

    let mut command = [0; 8];
    let mut chars = 0;

    let mut samples = [0u16; 4000];
    let mut repeat = update_frequency(&mut samples, 1000);

    loop {
        // set up DMA to move `repeat` samples from `samples` to the DAC
        let stream = &dma.st[5];
        // first, disable the stream so the addresses can be updated
        stream.cr.write(|w| w.en().clear_bit());
        // from and to address
        stream
            .par
            .write(|w| unsafe { w.bits(&dac.dhr12r1 as *const _ as u32) });
        stream
            .m0ar
            .write(|w| unsafe { w.bits(&samples[0] as *const _ as u32) });
        // since we'll have the memory transferred a u16 at a time, the number of samples is the
        // number of memory transactions
        stream.ndtr.write(|w| w.ndt().bits(repeat as u16));
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
        dac.cr.write(|w| {
            w.dmaen1().enabled();
            // the datasheet values for TSEL do not match the stm32f407 crate, so I'm trusting the datasheet.
            unsafe { w.tsel1().bits(5) }; // timer 4 trigger
            w.ten1().enabled();
            w.en1().set_bit()
        });

        let (action, value) = 'command: loop {
            // make sure there's at least one byte available, by potentially sacrificing the last character in the buffer
            chars = min(chars, command.len() - 1);

            let buf = &mut command[chars..];
            if let Some(count) = check_usb(&mut device, &mut serial, buf) {
                if count == 0 {
                    // why is read() returning me Ok(0), it should be WouldBlock in this case...
                    continue;
                }
                chars = min(command.len(), chars + count);
                if let Some(newline) = command[..chars].iter().position(|&c| c == b'\n') {
                    let value_bytes = &command[1..newline];
                    chars = 0;
                    if value_bytes.iter().all(|&c| c >= b'0' && c <= b'9') {
                        let mut value: usize = 0;
                        for &c in value_bytes.iter() {
                            value *= 10;
                            value += (c - b'0') as usize;
                        }
                        break 'command (command[0], value);
                    }
                }
            }
        };
        if action == b'f' {
            repeat = update_frequency(&mut samples, value);
        }
    }
}

fn check_usb<T: usb_device::bus::UsbBus>(
    bus: &mut UsbDevice<T>,
    class: &mut usbd_serial::SerialPort<T>,
    buf: &mut [u8],
) -> Option<usize> {
    if bus.poll(&mut [class]) {
        return class.read(buf).ok();
    }

    None
}

#[inline(never)]
fn update_frequency(samples: &mut [u16], hz: usize) -> usize {
    use micromath::F32Ext;

    let loop_samples = min(SAMPLE_RATE / hz, samples.len());
    for i in 0..loop_samples {
        samples[i] = ((0x700 as f32
            * (2.0 * 3.141592653589 * i as f32 / loop_samples as f32).sin())
            + 0x800 as f32) as u16;
    }

    loop_samples
}
