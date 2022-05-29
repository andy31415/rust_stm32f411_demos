#![no_std]
#![no_main]

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal as hal;

use crate::hal::{pac, prelude::*};

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    rtt_init_print!();

    rprintln!("Testing USB functionality!");

    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze();

    let gpioc = dp.GPIOC.split();
    let mut led = gpioc.pc13.into_push_pull_output();
    let mut delay = cp.SYST.delay(&clocks);

    loop {
        rprintln!("Looping ...");
        delay.delay_ms(1000u32);
        led.toggle();
    }
}
