#![no_std]
#![no_main]

use core::cell::RefCell;
use core::ops::{Add, Sub, Mul, Div, DerefMut};
use core::sync::atomic::AtomicU32;

use crate::hal::pac::interrupt;
use crate::hal::pac::Interrupt;
use crate::hal::{pac, prelude::*};
use cortex_m::interrupt::Mutex;
use hal::hal::digital::v2::InputPin;
use hal::pac::TIM2;
use hal::timer::{CounterUs, Event};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal as hal;

use usbd_human_interface_device::device::mouse::BootMouseInterface;
use usbd_human_interface_device::device::mouse::BootMouseReport;
use usbd_human_interface_device::prelude::*;

use cortex_m_rt::entry;

use stm32f4xx_hal::otg_fs::{UsbBus, USB};
use usb_device::prelude::*;

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

static GLOBAL_TIMER: Mutex<RefCell<Option<CounterUs<TIM2>>>> = Mutex::new(RefCell::new(None));
static GLOBAL_MILLISECONDS: AtomicU32 = AtomicU32::new(0);

fn now_ms() -> u32 {
    GLOBAL_MILLISECONDS.load(core::sync::atomic::Ordering::Acquire)
}

struct Throttler {
    last_report: u32,
}

/// Figures out when USB reports should be sent to a
/// USB.
impl Throttler {
    fn new() -> Self {
        Self {
            last_report: now_ms(),
        }
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

/// A position in space
#[derive(Debug, Default, PartialEq, PartialOrd, Copy, Clone)]
struct Point {
    x: f32,
    y: f32,
}

impl Add for Point {
    type Output = Point;

    fn add(self, rhs: Self) -> Self::Output {
        Self{x: self.x + rhs.x, y: self.y+rhs.y}
    }
}

impl Sub for Point {
    type Output = Point;

    fn sub(self, rhs: Self) -> Self::Output {
        Self{x: self.x - rhs.x, y: self.y-rhs.y}
    }
}

impl Mul<f32> for Point {
    type Output = Point;

    fn mul(self, rhs: f32) -> Self::Output {
        Self{x: self.x*rhs, y: self.y*rhs}
    }
}

impl Div<f32> for Point {
    type Output = Point;

    fn div(self, rhs: f32) -> Self::Output {
        Self{x: self.x/rhs, y: self.y/rhs}
    }
}


/// Returns a value [start, end] depending on
/// the given amount [0, 1]
fn interpolate(start: Point, end: Point, amount: f32) -> Point {
    assert!(amount >= 0.);
    assert!(amount <= 1.);
    start + (end - start)*amount
}



/// Defines a specific X/Y location of a mouse relative to the
/// starting point
#[derive(Debug, PartialEq)]
struct SprayOffset {
    /// when this position should be reached
    time_ms: u32,
    
    /// Where we should be at the given point in time
    position: Point,
}

const SPEED: u32 = 40;
const SCALE: f32 = 1.6;

impl SprayOffset {
    const fn new(time_ms: u32, x: i32, y: i32) -> Self {
        Self {
            time_ms: time_ms * SPEED,
            position: Point{x: x as f32, y: y as f32}
        }
    }
    
    // Awkward methods because floating point operations cannot
    // be used in CONST functions (https://github.com/rust-lang/rust/issues/57241)
    fn scaled_position(&self) -> Point {
        self.position * SCALE
    }
}

static AK47_RECOIL: &'static [SprayOffset] = &[
    SprayOffset::new(0, 0, 0),       // START
    SprayOffset::new(10, 0, 31),     // 29
    SprayOffset::new(18, -6, 80),    // 28
    SprayOffset::new(24, -3, 138),   // 27
    SprayOffset::new(30, 11, 210),   // 26
    SprayOffset::new(35, 32, 273),   // 25
    SprayOffset::new(39, 50, 320),   // 24
    SprayOffset::new(43, 47, 346),   // 23
    SprayOffset::new(47, 0, 357),    // 22
    SprayOffset::new(47, 0, 357),    // 21
    SprayOffset::new(51, -62, 359),  // 20
    SprayOffset::new(54, -100, 369), // 19
    SprayOffset::new(57, -119, 365), // 18
    SprayOffset::new(61, -137, 365), // 17
    SprayOffset::new(64, -155, 366), // 16
    SprayOffset::new(68, -163, 371), // 15
    SprayOffset::new(71, -141, 381), // 14
    SprayOffset::new(75, -85, 395),  // 13
    SprayOffset::new(78, -47, 397),  // 12
    SprayOffset::new(81, -12, 404),  // 11
    SprayOffset::new(85, 35, 404),   // 10
    SprayOffset::new(88, 69, 397),   // 9
    SprayOffset::new(91, 85, 399),   // 8
    SprayOffset::new(96, 82, 403),   // 7
    SprayOffset::new(99, 74, 408),   // 6
    SprayOffset::new(103, 72, 412),  // 5
    SprayOffset::new(108, 75, 419),  // 4
    SprayOffset::new(112, 83, 424),  // 3
    SprayOffset::new(118, 70, 423),  // 2
    SprayOffset::new(126, 8, 409),   // 1
    SprayOffset::new(134, -63, 399), // 0
    SprayOffset::new(144, -116, 379),
];

/// Manages spray control for a specific variable
struct SprayControl<'a, ButtonPin: InputPin> {
    button: ButtonPin,
    recoil_pattern: &'a [SprayOffset],
    spray_start_ms: Option<u32>,
    current_position: Point,
}

fn clamped_i8(value: f32) -> i8 {
    if value < i8::MIN as f32 {
        return i8::MIN;
    }
    if value > i8::MAX as f32 {
        return i8::MAX;
    }

    return value as i8;
}

impl<'a, ButtonPin: InputPin> SprayControl<'a, ButtonPin> {
    fn new(button: ButtonPin, recoil_pattern: &'a [SprayOffset]) -> Self {
        Self {
            button,
            recoil_pattern,
            spray_start_ms: None,
            current_position: Point::default(),
        }
    }

    fn report(&mut self) -> BootMouseReport {
        match self.button.is_low() {
            Ok(true) => self.spray_report(),
            Ok(false) => {
                self.spray_start_ms = None;
                BootMouseReport::default()
            }
            Err(_) => {
                rprintln!("Failure to check pin state!");
                BootMouseReport::default()
            }
        }
    }

    fn expected_position(&mut self) -> Point {
        let delta_ms = match self.spray_start_ms {
            None => {
                // start a new spray, at position 0
                self.spray_start_ms = Some(now_ms());
                self.current_position = Point::default();
                0
            }
            Some(time_ms) => now_ms() - time_ms,
        };

        for (prev, next) in self
            .recoil_pattern
            .iter()
            .zip(self.recoil_pattern.iter().skip(1))
        {
            if (delta_ms < prev.time_ms) || (delta_ms > next.time_ms) {
                continue;
            }

            let amount = (delta_ms - prev.time_ms) as f32 / (next.time_ms - prev.time_ms) as f32;
            return interpolate(prev.scaled_position(), next.scaled_position(), amount);
        }

        match self.recoil_pattern {
           [] => Point::default(),
           [.., last] => last.scaled_position()
        }
    }

    fn spray_report(&mut self) -> BootMouseReport {
        let expected = self.expected_position();
        let move_amount = expected - self.current_position;

        let delta_x = clamped_i8(move_amount.x);
        let delta_y = clamped_i8(move_amount.y);

        // ready to propose a movement
        self.current_position.x += delta_x as f32;
        self.current_position.y += delta_y as f32;

        return BootMouseReport {
            x: delta_x,
            y: delta_y,
            ..Default::default()
        };
    }
}

#[interrupt]
fn TIM2() {
    GLOBAL_MILLISECONDS.fetch_add(1, core::sync::atomic::Ordering::Release);

    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut t2) = GLOBAL_TIMER.borrow(cs).borrow_mut().deref_mut() {
            t2.clear_interrupt(Event::Update);
        }
    });
}

#[entry]
fn main() -> ! {
    rtt_init_print!();

    rprintln!("Preparing to start...");

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

    let mut mouse = UsbHidClassBuilder::new()
        .add_interface(BootMouseInterface::default_config())
        .build(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Andy314")
        .product("Test HID")
        .serial_number("TEST123321")
        .supports_remote_wakeup(false)
        .build();

    let mut throttler = Throttler::new();

    let mut spray_control = SprayControl::new(gpioa.pa0.into_pull_up_input(), AK47_RECOIL);

    rprintln!("Recoil control ready!");

    loop {
        if throttler.should_report() {
            match mouse.interface().write_report(&spray_control.report()) {
                Ok(_) => {}
                Err(UsbHidError::WouldBlock) => {}
                Err(UsbHidError::Duplicate) => {}
                Err(e) => {
                    rprintln!("WRITE REPORT ERROR: {:?}", e);
                }
            }
        }

        // no data to read, just dispatch as needed
        if usb_dev.poll(&mut [&mut mouse]) {}
    }
}
