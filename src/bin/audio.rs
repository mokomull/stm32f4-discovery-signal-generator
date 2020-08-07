#![no_main]
#![no_std]

use core::cmp::{max, min};

use cortex_m::iprintln;
use cortex_m_rt::entry;
use panic_itm as _;

use stm32f4xx_hal::prelude::*;
use usb_device::prelude::*;

// generated by: [int(0x700 * math.sin(2*math.pi/400* t)) + 0x800 for t in range(0, 400)]
static SINES: &[u16] = &[
    2048, 2076, 2104, 2132, 2160, 2188, 2216, 2244, 2272, 2300, 2328, 2356, 2383, 2411, 2438, 2466,
    2493, 2520, 2547, 2574, 2601, 2628, 2655, 2681, 2707, 2733, 2759, 2785, 2810, 2836, 2861, 2886,
    2911, 2935, 2960, 2984, 3008, 3031, 3055, 3078, 3101, 3123, 3146, 3168, 3190, 3211, 3233, 3254,
    3274, 3295, 3315, 3334, 3354, 3373, 3392, 3410, 3428, 3446, 3463, 3481, 3497, 3514, 3530, 3545,
    3561, 3575, 3590, 3604, 3618, 3631, 3644, 3657, 3669, 3681, 3692, 3703, 3714, 3724, 3734, 3743,
    3752, 3760, 3768, 3776, 3783, 3790, 3796, 3802, 3808, 3813, 3817, 3822, 3825, 3829, 3832, 3834,
    3836, 3838, 3839, 3839, 3840, 3839, 3839, 3838, 3836, 3834, 3832, 3829, 3825, 3822, 3817, 3813,
    3808, 3802, 3796, 3790, 3783, 3776, 3768, 3760, 3752, 3743, 3734, 3724, 3714, 3703, 3692, 3681,
    3669, 3657, 3644, 3631, 3618, 3604, 3590, 3575, 3561, 3545, 3530, 3514, 3497, 3481, 3463, 3446,
    3428, 3410, 3392, 3373, 3354, 3334, 3315, 3295, 3274, 3254, 3233, 3211, 3190, 3168, 3146, 3123,
    3101, 3078, 3055, 3031, 3008, 2984, 2960, 2935, 2911, 2886, 2861, 2836, 2810, 2785, 2759, 2733,
    2707, 2681, 2655, 2628, 2601, 2574, 2547, 2520, 2493, 2466, 2438, 2411, 2383, 2356, 2328, 2300,
    2272, 2244, 2216, 2188, 2160, 2132, 2104, 2076, 2048, 2020, 1992, 1964, 1936, 1908, 1880, 1852,
    1824, 1796, 1768, 1740, 1713, 1685, 1658, 1630, 1603, 1576, 1549, 1522, 1495, 1468, 1441, 1415,
    1389, 1363, 1337, 1311, 1286, 1260, 1235, 1210, 1185, 1161, 1136, 1112, 1088, 1065, 1041, 1018,
    995, 973, 950, 928, 906, 885, 863, 842, 822, 801, 781, 762, 742, 723, 704, 686, 668, 650, 633,
    615, 599, 582, 566, 551, 535, 521, 506, 492, 478, 465, 452, 439, 427, 415, 404, 393, 382, 372,
    362, 353, 344, 336, 328, 320, 313, 306, 300, 294, 288, 283, 279, 274, 271, 267, 264, 262, 260,
    258, 257, 257, 256, 257, 257, 258, 260, 262, 264, 267, 271, 274, 279, 283, 288, 294, 300, 306,
    313, 320, 328, 336, 344, 353, 362, 372, 382, 393, 404, 415, 427, 439, 452, 465, 478, 492, 506,
    521, 535, 551, 566, 582, 599, 615, 633, 650, 668, 686, 704, 723, 742, 762, 781, 801, 822, 842,
    863, 885, 906, 928, 950, 973, 995, 1018, 1041, 1065, 1088, 1112, 1136, 1161, 1185, 1210, 1235,
    1260, 1286, 1311, 1337, 1363, 1389, 1415, 1441, 1468, 1495, 1522, 1549, 1576, 1603, 1630, 1658,
    1685, 1713, 1740, 1768, 1796, 1824, 1852, 1880, 1908, 1936, 1964, 1992, 2020,
];

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
            w.tim3en().set_bit()
        });
    }
    let dac = peripherals.DAC;
    let timer = peripherals.TIM3;
    // 84MHz (since I suppose the APBx prescaler causes the timer clock to be doubled) / 400ksps
    timer.arr.write(|w| w.arr().bits(210));
    timer.cr1.write(|w| w.cen().set_bit());

    dac.cr.write(|w| w.en1().set_bit());

    let mut command = [0; 8];
    let mut chars = 0;

    loop {
        let (action, value) = 'command: loop {
            for &value in SINES {
                while !timer.sr.read().uif().bit() {
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
                }
                timer.sr.modify(|_r, w| w.uif().clear_bit());

                dac.dhr12r1.write(|w| unsafe { w.dacc1dhr().bits(value) });
            }
        };
        let _ = serial.write(&[action, b'\n']);
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
