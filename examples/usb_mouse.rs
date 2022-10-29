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

impl SprayOffset {
    const fn new(time_ms: u32, x: i32, y: i32) -> Self {
        Self { time_ms, x, y }
    }
}

static AK47_RECOIL: &'static [SprayOffset] = &[
    SprayOffset::new(0, 0, 0),
    SprayOffset::new(1000, 200, 200),
    SprayOffset::new(2000, 400, 0),
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
    if value > i8::MAX as i32{
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

            let target_x = (prev.x * (total_ms - offset_ms) + next.x * offset_ms) / total_ms;
            let target_y = (prev.y * (total_ms - offset_ms) + next.y * offset_ms) / total_ms;

            // Figure out how much to move. Note that we may need to clamp down on the movement
            let delta_x = target_x - self.current_position.x;
            let delta_y = target_y - self.current_position.y;
            
            let delta_x = clamped_i8(delta_x);
            let delta_y = clamped_i8(delta_y);
            
            
            // ready to propose a movement
            self.current_position.offset(delta_x, delta_y);
            return BootMouseReport{
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
