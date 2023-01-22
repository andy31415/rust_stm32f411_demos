#![no_std]
#![no_main]

use hal::i2c::Mode;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal as hal;

use crate::hal::{pac, prelude::*};

use cortex_m_rt::entry;

use pcf857x::Pcf8575;

use crate::hal::i2c::I2c;

#[entry]
fn main() -> ! {
    rtt_init_print!();

    rprintln!("Program started!");

    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze();

    let gpioc = dp.GPIOC.split();
    let mut led = gpioc.pc13.into_push_pull_output();
    let mut delay = cp.SYST.delay(&clocks);
    
    let gpiob = dp.GPIOB.split();
    let sda = gpiob.pb7;
    let scl = gpiob.pb6;
    
    let i2c = I2c::new(
        dp.I2C1, 
        (scl, sda), 
        Mode::Standard { frequency: 100.kHz() },
        &clocks
    );
    
    let mut expander = Pcf8575::new(i2c, pcf857x::SlaveAddr::Default);
    
    loop {
        rprintln!("Blink loop...");
        for _ in 1..3 {
            delay.delay_ms(200u32);

            led.toggle();
            expander.set(0xFFFF).unwrap();

            delay.delay_ms(200u32);

            led.toggle();
            expander.set(0).unwrap();
        }
        for _ in 1..3 {
            delay.delay_ms(1000u32);

            led.toggle();
            expander.set(0xFFFF).unwrap();

            delay.delay_ms(1000u32);

            led.toggle();
            expander.set(0).unwrap();
        }
    }
}
