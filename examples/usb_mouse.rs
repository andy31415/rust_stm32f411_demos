#![no_std]
#![no_main]

use core::cell::RefCell;
use core::ops::DerefMut;
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

/// Defines a specific X/Y location of a mouse relative to the
/// starting point
#[derive(Debug, PartialEq)]
struct SprayOffset {
    time_ms: u32,
    /// when this position should be reached
    x: i32,
    y: i32,
}

const SPEED: u32 = 40;
const SCALE: f32 = 1.6;

impl SprayOffset {
    const fn new(time_ms: u32, x: i32, y: i32) -> Self {
        Self {
            time_ms: time_ms * SPEED,
            x,
            y,
        }
    }

    fn scaled_x(&self) -> i32 {
        ((self.x as f32) * SCALE) as i32
    }

    fn scaled_y(&self) -> i32 {
        ((self.y as f32) * SCALE) as i32
    }
}

static AK47_RECOIL: &'static [SprayOffset] = &[
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

#[derive(Debug, PartialEq, Default)]
struct MousePosition {
    x: i32,
    y: i32,
}

impl MousePosition {
    fn offset(&mut self, delta_x: i8, delta_y: i8) {
        self.x += delta_x as i32;
        self.y += delta_y as i32;
    }
}

/// Manages spray control for a specific variable
struct SprayControl<'a, ButtonPin: InputPin> {
    button: ButtonPin,
    recoil_pattern: &'a [SprayOffset],
    spray_start_ms: Option<u32>,
    current_position: MousePosition,
}

fn clamped_i8(value: i32) -> i8 {
    if value < i8::MIN as i32 {
        return i8::MIN;
    }
    if value > i8::MAX as i32 {
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
            current_position: MousePosition::default(),
        }
    }

    fn report(&mut self) -> BootMouseReport {
        match self.button.is_high() {
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

    fn spray_report(&mut self) -> BootMouseReport {
        let delta_ms = match self.spray_start_ms {
            None => {
                // start a new spray, at position 0
                self.spray_start_ms = Some(now_ms());
                self.current_position = MousePosition::default();
                0
            }
            Some(time_ms) => now_ms() - time_ms,
        };

        // Figure out where we should be compared to where we are
        for (prev, next) in self
            .recoil_pattern
            .iter()
            .zip(self.recoil_pattern.iter().skip(1))
        {
            if (delta_ms < prev.time_ms) || (delta_ms > next.time_ms) {
                continue;
            }
            // Figure out where we should be and adjust x and y locations accordingly.
            // Note that we may not be able to reach it, so we may need to clamp
            let offset_ms = (delta_ms - prev.time_ms) as i32;
            let total_ms = (next.time_ms - prev.time_ms) as i32;

            let target_x =
                (prev.scaled_x() * (total_ms - offset_ms) + next.scaled_x() * offset_ms) / total_ms;
            let target_y =
                (prev.scaled_y() * (total_ms - offset_ms) + next.scaled_y() * offset_ms) / total_ms;

            // Figure out how much to move. Note that we may need to clamp down on the movement
            let delta_x = target_x - self.current_position.x;
            let delta_y = target_y - self.current_position.y;

            let delta_x = clamped_i8(delta_x);
            let delta_y = clamped_i8(delta_y);

            // ready to propose a movement
            self.current_position.offset(delta_x, delta_y);
            return BootMouseReport {
                x: delta_x,
                y: delta_y,
                ..Default::default()
            };
        }

        // nothing to do: no proper position
        BootMouseReport::default()
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

    let mut spray_control = SprayControl::new(gpioa.pa0.into_input(), AK47_RECOIL);

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
