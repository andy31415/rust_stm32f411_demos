#![no_std]
#![no_main]

use hal::i2c::Mode;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal as hal;

use crate::hal::{pac, prelude::*};

use cortex_m_rt::entry;

use pcf857x::OutputPin;
use pcf857x::Pcf8575;

use crate::hal::i2c::I2c;

struct ToggleableOutputPin<O: OutputPin> {
    pin: O,
    is_high: bool,
}

impl<O: OutputPin> ToggleableOutputPin<O> {
    fn new(mut pin: O, is_high: bool) -> Result<Self, O::Error> {
        if is_high {
            pin.set_high()?
        } else {
            pin.set_low()?
        }
        Ok(Self { pin, is_high })
    }

    fn toggle(&mut self) -> Result<(), O::Error> {
        self.is_high = !self.is_high;

        if self.is_high {
            self.pin.set_high()?;
        } else {
            self.pin.set_low()?;
        }
        Ok(())
    }
}

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
        Mode::Standard {
            frequency: 100.kHz(),
        },
        &clocks,
    );

    let expander = Pcf8575::new(i2c, pcf857x::SlaveAddr::Default);
    let mut led2 = ToggleableOutputPin::new(expander.split().p3, false).unwrap();

    loop {
        rprintln!("Blink loop...");
        for _ in 1..6 {
            delay.delay_ms(200u32);
            led.toggle();
            led2.toggle().unwrap();
        }
        for _ in 1..6 {
            delay.delay_ms(1000u32);

            led.toggle();
            led2.toggle().unwrap();
        }
    }
}
