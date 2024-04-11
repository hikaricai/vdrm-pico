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
use rp2040_hal::gpio;
use rp2040_hal::gpio::Interrupt::EdgeLow;
use rp2040_hal::timer::Instant;
use rtt_target::{rprintln, rprint, rtt_init_default};
use hub75::DMAExt;

type ButtonPin = gpio::Pin<gpio::bank0::Gpio17, gpio::FunctionSioInput, gpio::PullUp>;

struct Ctx {
    display: hub75::Display<hub75::CH0, hub75::CH1>,
    alarm: hal::timer::Alarm0,
    timer: hal::Timer,
    button: ButtonPin,
    angle: usize,
    angle_offset: i32,
    interval: u32,
}

impl Ctx {
    fn new(display: hub75::Display<hub75::CH0, hub75::CH1>,
           mut timer: hal::Timer,
           button: ButtonPin, ) -> Self {
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(FRESH_INTERVAL_US);
        alarm.enable_interrupt();
        Self {
            display,
            alarm,
            timer,
            button,
            angle: 0,
            angle_offset: 0,
            interval: 0,
        }
    }
}

// Place our LED and Alarm type in a static variable, so we can access it from interrupts
static mut CTX: Mutex<RefCell<Option<Ctx>>> = Mutex::new(RefCell::new(None));
const FRESH_INTERVAL_US: MicrosDurationU32 = MicrosDurationU32::millis(10);
const PIXEL_BUF: &'static [u8] = include_bytes!("../assets/pixel_buf.bin");
const ADDR_BUF: &'static [u8] = include_bytes!("../assets/addr_buf.bin");
const ANGLE_BUF: &'static [u8] = include_bytes!("../assets/angle_buf.bin");

#[entry]
fn main() -> ! {
    let mut rtt = rtt_init_default!();
    rtt_target::set_print_channel(rtt.up.0);
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
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
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
    let display =
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
                lat: pins.gpio14.into_function().into_pull_type().into_dyn_pin(),
                clk0: pins.gpio15.into_function().into_pull_type().into_dyn_pin(),
                clk1: pins.gpio16.into_function().into_pull_type().into_dyn_pin(),
            },
            &mut pio,
            (sm0, sm1),
            (dma.ch0, dma.ch1),
            PIXEL_BUF,
            ADDR_BUF,
            ANGLE_BUF,
        );

    let button = pins.gpio17.reconfigure();
    button.set_interrupt_enabled(EdgeLow, true);

    critical_section::with(|cs| {
        // Move alarm into ALARM, so that it can be accessed from interrupts
        unsafe {
            let ctx = Ctx::new(
                display,
                timer,
                button,
            );
            CTX.borrow(cs).replace(Some(ctx));
        }
    });
    // Unmask the timer0 IRQ so that it will generate an interrupt
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }
    rprintln!("before loop");
    let mut read_buf = [0; 16];
    loop {
        // critical_section::with(|cs| {
        //     let mut borrow = unsafe {
        //         CTX.borrow(cs).borrow_mut()
        //     };
        //     let display = borrow.as_mut().map(|v| &mut v.0).unwrap();
        //     // display.commit();
        // });
        delay.delay_ms(100);
        let read_len = rtt.down.0.read(&mut read_buf);
        if read_len == 0 {
            continue;
        }
        critical_section::with(|cs| {
            let mut borrow: RefMut<Option<Ctx>> = unsafe { CTX.borrow(cs).borrow_mut() };
            let Some(ctx) = borrow.as_mut() else { return; };
            for &c in read_buf.iter().take(read_len) {
                let deta = match c {
                    b'1' => { -1 }
                    b'2' => { 1 }
                    _ => { 0 }
                };
                ctx.angle_offset += deta;
            }
            rprintln!("\nangle_offset {}", ctx.angle_offset);
        });
    }
}

#[interrupt]
fn TIMER_IRQ_0() {
    // info!("timer0 irq");
    critical_section::with(|cs| {
        let mut borrow: RefMut<Option<Ctx>> = unsafe { CTX.borrow(cs).borrow_mut() };
        let Some(ctx) = borrow.as_mut() else { return; };
        ctx.alarm.clear_interrupt();
        if ctx.interval == 0 {
            return;
        }
        ctx.angle += 1;
        ctx.angle %= 100;
        let interval = MicrosDurationU32::micros(ctx.interval);
        let _ = ctx.alarm.schedule(interval);
        let busy = ctx.display.commit();
        if busy {
            rprintln!("busy in timer interrupt");
        }
        ctx.display.refresh(ctx.angle);
    });
}

#[interrupt]
fn IO_IRQ_BANK0() {
    static mut TICKS0: u64 = 0;
    static mut TICKS1: u64 = 0;
    static mut LOG_CNT: u32 = 0;
    *LOG_CNT += 1;
    critical_section::with(|cs| {
        let mut borrow: RefMut<Option<Ctx>> = unsafe { CTX.borrow(cs).borrow_mut() };
        let Some(ctx) = borrow.as_mut() else { return; };
        let _ = ctx.alarm.cancel();
        ctx.alarm.clear_interrupt();

        ctx.button.clear_interrupt(EdgeLow);
        let ticks: u64 = {
            *TICKS0 = *TICKS1;
            *TICKS1 = ctx.timer.get_counter().ticks();
            if *TICKS0 == 0 {
                return;
            }
            *TICKS1 - *TICKS0
        };
        // begins at 1/4 of circle
        let angle = ((ctx.angle_offset + 100) as usize + 28) % 100;
        ctx.angle = angle;
        let interval = ticks as u32 / 100;
        ctx.interval = interval;
        if *LOG_CNT % 50 == 0 {
            rprintln!("interval {}", interval);
        }
        let interval = MicrosDurationU32::micros(interval);
        let _ = ctx.alarm.schedule(interval);
        let busy = ctx.display.commit();
        if busy {
            rprintln!("busy in io interrupt");
        }
        ctx.display.refresh(ctx.angle);
    });
}