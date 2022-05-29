#![no_std]
#![no_main]

use core::cell::RefCell;
use core::ops::DerefMut;

use crate::hal::pac::interrupt;
use crate::hal::pac::Interrupt;
use crate::hal::{pac, prelude::*};
use cortex_m::interrupt::Mutex;
use embedded_time::rate::Fraction;
use embedded_time::{Clock, Instant};
use hal::pac::TIM2;
use hal::timer::{CounterUs, Event};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal as hal;

use usbd_human_interface_device::device::keyboard::NKROBootKeyboardInterface;
use usbd_human_interface_device::page::Keyboard;
use usbd_human_interface_device::prelude::*;

use cortex_m_rt::entry;

use stm32f4xx_hal::otg_fs::{UsbBus, USB};
use usb_device::prelude::*;

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

static GLOBAL_TIMER: Mutex<RefCell<Option<CounterUs<TIM2>>>> = Mutex::new(RefCell::new(None));
static GLOBAL_MILLISECONDS: Mutex<RefCell<u64>> = Mutex::new(RefCell::new(0));

fn now_ms() -> u64 {
    cortex_m::interrupt::free(|cs| *GLOBAL_MILLISECONDS.borrow(cs).borrow())
}

struct Throttler {
    last_tick: u64,
    last_report: u64,
}

impl Throttler {
    fn new() -> Self {
        Self {
            last_tick: now_ms(),
            last_report: now_ms(),
        }
    }

    fn should_tick(&mut self) -> bool {
        let c = now_ms();
        if c < self.last_tick + 1 {
            return false;
        }
        self.last_tick = c;
        true
    }

    fn should_report(&mut self) -> bool {
        let c = now_ms();
        if c < self.last_report + 10 {
            return false;
        }
        self.last_report = c;
        true
    }
}

struct FakeClock {}

impl FakeClock {
    fn new() -> Self {
        Self {}
    }
}

#[interrupt]
fn TIM2() {
    cortex_m::interrupt::free(|cs| {
        *GLOBAL_MILLISECONDS.borrow(cs).borrow_mut() += 1;

        if let Some(ref mut t2) = GLOBAL_TIMER.borrow(cs).borrow_mut().deref_mut() {
            t2.clear_interrupt(Event::Update);
        }
    });
}

impl Clock for FakeClock {
    type T = u64;

    const SCALING_FACTOR: Fraction = Fraction::new(1, 1000);

    fn try_now(&self) -> Result<embedded_time::Instant<Self>, embedded_time::clock::Error> {
        Ok(Instant::<FakeClock>::new(now_ms()))
    }
}

#[entry]
fn main() -> ! {
    rtt_init_print!();

    rprintln!("Testing USB functionality!");

    let dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(25.MHz())
        .sysclk(48.MHz())
        .require_pll48clk()
        .freeze();

    let gpioa = dp.GPIOA.split();

    let usb = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: gpioa.pa11.into_alternate(),
        pin_dp: gpioa.pa12.into_alternate(),
        hclk: clocks.hclk(),
    };

    let pin = gpioa.pa0.into_input();

    let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });

    let mut timer = dp.TIM2.counter_us(&clocks);
    timer.start(1.millis()).unwrap();
    timer.listen(Event::Update);

    cortex_m::interrupt::free(|cs| {
        GLOBAL_TIMER.borrow(cs).borrow_mut().replace(timer);
    });

    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
    }

    let clock = FakeClock::new();

    let mut keyboard = UsbHidClassBuilder::new()
        .add_interface(NKROBootKeyboardInterface::default_config(&clock))
        .build(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Andy314")
        .product("Test HID")
        .serial_number("TEST123321")
        .supports_remote_wakeup(false)
        .build();

    let mut throttler = Throttler::new();

    loop {
        let keys = if pin.is_low() {
            &[Keyboard::A]
        } else {
            &[Keyboard::NoEventIndicated]
        };

        if throttler.should_report() {
            match keyboard.interface().write_report(keys) {
                Ok(_) => {}
                Err(UsbHidError::WouldBlock) => {}
                Err(UsbHidError::Duplicate) => {}
                Err(e) => {
                    rprintln!("WRITE REPORT ERROR: {:?}", e);
                }
            }
        }

        if throttler.should_tick() {
            match keyboard.interface().tick() {
                Ok(_) => {}
                Err(UsbHidError::WouldBlock) => {}
                Err(e) => {
                    rprintln!("TICK ERROR: {:?}", e);
                }
            }
        }

        if usb_dev.poll(&mut [&mut keyboard]) {
            match keyboard.interface().read_report() {
                Ok(l) => {
                    rprintln!("REPORT: {:?}", l);
                }
                Err(UsbError::WouldBlock) => {}
                Err(e) => {
                    rprintln!("Read report error: {:?}", e);
                }
            }
        }
    }
}
