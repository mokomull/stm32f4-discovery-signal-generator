#![no_main]
#![no_std]

use cortex_m::iprintln;
use cortex_m_rt::entry;
use panic_itm as _;

use stm32f4xx_hal::prelude::*;

const ADDRESS: u8 = 0x94 >> 1;

#[entry]
fn main() -> ! {
    let peripherals = stm32f407g_disc::Peripherals::take().unwrap();
    let mut core_peripherals = cortex_m::Peripherals::take().unwrap();

    let rcc = peripherals.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.mhz()).sysclk(168.mhz()).freeze();

    let itm = &mut core_peripherals.ITM.stim[0];

    let portb = peripherals.GPIOB.split();
    let portd = peripherals.GPIOD.split();

    let mut audio_reset = portd.pd4.into_push_pull_output();
    audio_reset.set_high().unwrap();
    let mut i2c = stm32f4xx_hal::i2c::I2c::i2c1(
        peripherals.I2C1,
        (
            portb.pb6.into_alternate_af4_open_drain(),
            portb.pb9.into_alternate_af4_open_drain(),
        ),
        50.khz(),
        clocks,
    );

    let mut buffer = [0x01u8; 1];
    match i2c.write(ADDRESS, &buffer) {
        Ok(()) => {
            iprintln!(itm, "wrote the address");
            match i2c.read(ADDRESS, &mut buffer) {
                Ok(()) => iprintln!(itm, "read byte 0x{:02x}", buffer[0]),
                x => iprintln!(itm, "error reading {:?}", x),
            }
        }
        x => iprintln!(itm, "error writing {:?}", x),
    }

    set_dac_register(&mut i2c, 0x04, 0xaf); // headphone channels ON, speaker channels OFF
    set_dac_register(&mut i2c, 0x05, 0x30); // auto = 0, speed = 01, 32kgroup = 1, videoclk = 0, ratio = 0, mclkdiv2 = 0
    set_dac_register(&mut i2c, 0x06, 0x00); // I2S slave, not inverted, not DSP mode, left justified format
    set_dac_register(&mut i2c, 0x07, 0x00); // leave Interface Control 2 alone

    // section 4.11 from the CS43L22 datasheet
    set_dac_register(&mut i2c, 0x00, 0x99);
    set_dac_register(&mut i2c, 0x47, 0x80);
    set_dac_register(&mut i2c, 0x32, 0x80);
    set_dac_register(&mut i2c, 0x32, 0x00);

    // step 6 of 4.9 of CS43L22 datasheet
    set_dac_register(&mut i2c, 0x00, 0x9e);

    loop {}
}

use embedded_hal::blocking::i2c;

fn set_dac_register<I>(i2c: &mut I, register: u8, value: u8)
where
    I: i2c::Write,
    I::Error: core::fmt::Debug,
{
    i2c.write(ADDRESS, &[register, value]).unwrap();
}
