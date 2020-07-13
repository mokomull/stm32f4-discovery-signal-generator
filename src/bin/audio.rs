#![no_main]
#![no_std]

use cortex_m::iprintln;
use cortex_m_rt::entry;
use panic_itm as _;

use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::stm32::spi1::i2scfgr;

const ADDRESS: u8 = 0x94 >> 1;

#[entry]
fn main() -> ! {
    let peripherals = stm32f407g_disc::Peripherals::take().unwrap();
    let mut core_peripherals = cortex_m::Peripherals::take().unwrap();

    let rcc = peripherals.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.mhz()).sysclk(168.mhz()).freeze();

    let itm = &mut core_peripherals.ITM.stim[0];

    let porta = peripherals.GPIOA.split();
    let portb = peripherals.GPIOB.split();
    let portc = peripherals.GPIOC.split();
    let portd = peripherals.GPIOD.split();

    let _mck = portc.pc7.into_alternate_af6();
    let _sck = portc.pc10.into_alternate_af6();
    let _sd = portc.pc12.into_alternate_af6();
    let _ws = porta.pa4.into_alternate_af6();

    // enable the I2S PLL: the VCO input should be 2MHz since we have an 8MHz crystal -- but that's
    // going to depend on what freeze() chose above.
    // 2MHz * 128 / 5 = 51.2 MHz
    unsafe {
        let rcc = &*stm32f4xx_hal::stm32::RCC::ptr();
        rcc.plli2scfgr.write(|w| {
            w.plli2sn().bits(128);
            w.plli2sr().bits(5)
        });
        rcc.cr.modify(|_r, w| w.plli2son().set_bit());
        while !rcc.cr.read().plli2srdy().bit() {}
        // turn on power to the SPI3/I2S3 peripheral
        rcc.apb1enr.modify(|_r, w| w.spi3en().set_bit());
    }
    let spi = peripherals.SPI3;
    spi.i2scfgr.write(|w| {
        w.i2smod().set_bit();
        w.i2scfg().variant(i2scfgr::I2SCFG_A::MASTERTX);
        w.i2sstd().variant(i2scfgr::I2SSTD_A::MSB);
        w.ckpol().set_bit();
        w.datlen().variant(i2scfgr::DATLEN_A::SIXTEENBIT);
        w.chlen().variant(i2scfgr::CHLEN_A::SIXTEENBIT)
    });
    spi.i2spr.write(|w| {
        w.mckoe().set_bit();
        unsafe { w.i2sdiv().bits(12) };
        w.odd().set_bit()
    });
    spi.i2scfgr.modify(|_r, w| w.i2se().set_bit());

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
    set_dac_register(&mut i2c, 0x05, 0x80); // auto = 1, everything else 0
    set_dac_register(&mut i2c, 0x06, 0x00); // I2S slave, not inverted, not DSP mode, left justified format
    set_dac_register(&mut i2c, 0x07, 0x00); // leave Interface Control 2 alone

    // section 4.11 from the CS43L22 datasheet
    set_dac_register(&mut i2c, 0x00, 0x99);
    set_dac_register(&mut i2c, 0x47, 0x80);
    set_dac_register(&mut i2c, 0x32, 0x80);
    set_dac_register(&mut i2c, 0x32, 0x00);

    // step 6 of 4.9 of CS43L22 datasheet
    set_dac_register(&mut i2c, 0x02, 0x9e);

    const LEVEL: i16 = 0x7ff;

    loop {
        // 4 samples, L and R channels, of each low and high - should give me 8k / (4 * 2) = 1000Hz
        for _ in 0..(4 * 2) {
            spi.dr.write(|w| w.dr().bits(LEVEL as u16));
            while !spi.sr.read().txe().bit() {}
        }

        for _ in 0..(4 * 2) {
            spi.dr.write(|w| w.dr().bits(-LEVEL as u16));
            while !spi.sr.read().txe().bit() {}
        }
    }
}

use embedded_hal::blocking::i2c;

fn set_dac_register<I>(i2c: &mut I, register: u8, value: u8)
where
    I: i2c::Write,
    I::Error: core::fmt::Debug,
{
    i2c.write(ADDRESS, &[register, value]).unwrap();
}
