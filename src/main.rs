//! Displays an animated Nyan cat
#![no_std]
#![no_main]

mod hub75;

use rp_pico as bsp;
use bsp::entry;
use core::cell::RefCell;
use panic_rtt_target as _;
use core::cell::RefMut;
use critical_section::Mutex;
use bsp::hal;
use hal::timer::Alarm;
use hal::fugit::MicrosDurationU32;
use pac::interrupt;
use bsp::hal::pio::PIOExt;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use rtt_target::{rprintln, rtt_init_default};
use hub75::DMAExt;

type DisplayAndAlarm = (
    hub75::Display<hub75::CH0, hub75::CH1>,
    hal::timer::Alarm0,
);

// Place our LED and Alarm type in a static variable, so we can access it from interrupts
static mut DISPALY_AND_ALARM: Mutex<RefCell<Option<DisplayAndAlarm>>> = Mutex::new(RefCell::new(None));
const FRESH_INTERVAL_US: MicrosDurationU32 = MicrosDurationU32::millis(10);
const PIXEL_BUF: &'static [u8] = include_bytes!("../assets/pixel_buf.bin");
const ADDR_BUF: &'static [u8] = include_bytes!("../assets/addr_buf.bin");
const ANGLE_BUF: &'static [u8] = include_bytes!("../assets/angle_buf.bin");

#[entry]
fn main() -> ! {
    let mut rtt = rtt_init_default!();
    rtt_target::set_print_channel(rtt.up.0);
    let mut read_buf = [0; 16];
    let _read = rtt.down.0.read(&mut read_buf);
    // rtt_init_print!();
    rprintln!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
        .ok()
        .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Split PIO0 SM
    let (mut pio, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // Reset DMA
    let resets = pac.RESETS;
    resets.reset.modify(|_, w| w.dma().set_bit());
    resets.reset.modify(|_, w| w.dma().clear_bit());
    while resets.reset_done.read().dma().bit_is_clear() {}

    // Split DMA
    let dma = pac.DMA.split();
    let mut display = unsafe {
        hub75::Display::new(
            hub75::DisplayPins {
                r1: pins.gpio0.into_function().into_pull_type().into_dyn_pin(),
                g1: pins.gpio1.into_function().into_pull_type().into_dyn_pin(),
                b1: pins.gpio2.into_function().into_pull_type().into_dyn_pin(),
                r2: pins.gpio3.into_function().into_pull_type().into_dyn_pin(),
                g2: pins.gpio4.into_function().into_pull_type().into_dyn_pin(),
                b2: pins.gpio5.into_function().into_pull_type().into_dyn_pin(),
                addra: pins.gpio6.into_function().into_pull_type().into_dyn_pin(),
                addrb: pins.gpio7.into_function().into_pull_type().into_dyn_pin(),
                addrc: pins.gpio8.into_function().into_pull_type().into_dyn_pin(),
                addrd: pins.gpio9.into_function().into_pull_type().into_dyn_pin(),
                addre: pins.gpio10.into_function().into_pull_type().into_dyn_pin(),
                oe0: pins.gpio11.into_function().into_pull_type().into_dyn_pin(),
                oe1: pins.gpio12.into_function().into_pull_type().into_dyn_pin(),
                oe2: pins.gpio13.into_function().into_pull_type().into_dyn_pin(),
                clk: pins.gpio14.into_function().into_pull_type().into_dyn_pin(),
                lat: pins.gpio15.into_function().into_pull_type().into_dyn_pin(),
            },
            &mut pio,
            (sm0, sm1),
            (dma.ch0, dma.ch1),
            PIXEL_BUF,
            ADDR_BUF,
            ANGLE_BUF,
        )
    };

    critical_section::with(|cs| {
        let mut alarm = timer.alarm_0().unwrap();
        // Schedule an alarm in 1 second
        let _ = alarm.schedule(FRESH_INTERVAL_US);
        // Enable generating an interrupt on alarm
        alarm.enable_interrupt();
        // Move alarm into ALARM, so that it can be accessed from interrupts
        unsafe {
            DISPALY_AND_ALARM.borrow(cs).replace(Some((display, alarm)));
        }
    });
    // Unmask the timer0 IRQ so that it will generate an interrupt
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }
    rprintln!("before loop");
    loop {
        critical_section::with(|cs| {
            let mut borrow = unsafe {
                DISPALY_AND_ALARM.borrow(cs).borrow_mut()
            };
            let display = borrow.as_mut().map(|v| &mut v.0).unwrap();
            // display.commit();
        });
        delay.delay_ms(100);
    }
}

#[interrupt]
fn TIMER_IRQ_0() {
    // info!("timer0 irq");
    static mut ANGLE: usize = 0;
    critical_section::with(|cs| {
        let mut borrow: RefMut<Option<DisplayAndAlarm>> = unsafe { DISPALY_AND_ALARM.borrow(cs).borrow_mut() };
        if let Some((display, alarm)) = borrow.as_mut() {
            alarm.clear_interrupt();
            let _ = alarm.schedule(FRESH_INTERVAL_US);
            display.refresh(*ANGLE);
            *ANGLE += 1;
            *ANGLE %= 96;
            // info!("display refresh");
        }
    });
}