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

    loop {}
}
