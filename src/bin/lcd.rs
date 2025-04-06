//! # GPIO 'Blinky' Example
//!
//! This application demonstrates how to control a GPIO pin on the rp235x.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the `Cargo.toml` file for Copyright and license details.


//todo note that this still needs to be tested on the actual hardware
//but have to wait for the voltage regulator to arrive
//todo also need to test the i2c pins and the lcd driver

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;


// Alias for our HAL crate
use rp235x_hal as hal;

//lcd driver
use hd44780_driver::{self as lcd_driver, HD44780};

// Some things we need
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin};


/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the rp235x peripherals, then toggles a GPIO pin in
/// an infinite loop. If there is an LED connected to that pin, it will blink.
#[hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio14.into_push_pull_output();


    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio18.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio19.reconfigure(); 

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let mut i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let mut lcd = 
    lcd_driver::HD44780::new_i2c(i2c, 0x3F, &mut timer).unwrap();

    lcd.reset(&mut timer).unwrap();
    lcd.clear(&mut timer).unwrap();

    match lcd.write_str("HI", &mut timer) {
        Ok(_) => {
            led_pin.set_high().unwrap();
            timer.delay_ms(500);
            led_pin.set_low().unwrap();
            timer.delay_ms(500);
            led_pin.set_high().unwrap();
            timer.delay_ms(500);
            led_pin.set_low().unwrap();
            timer.delay_ms(500);
        },
        Err(e) => {
            led_pin.set_high().unwrap();
            timer.delay_ms(1000);
            led_pin.set_low().unwrap();
            timer.delay_ms(1000);
        }
    };


    loop {
        // led_pin.set_high().unwrap();
        // timer.delay_ms(500);
        // led_pin.set_low().unwrap();
        // timer.delay_ms(500);
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"LCD example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file

//motion detector: https://www.hobbyelectronica.nl/product/am312-mini-pir/

// https://www.handsontec.com/dataspecs/module/I2C_1602_LCD.pdf

//https://github.com/JohnDoneth/hd44780-driver/tree/master
//https://github.com/JohnDoneth/hd44780-driver

//https://crates.io/crates/hd44780-driver

//https://datasheets.raspberrypi.com/picow/pico-2-w-datasheet.pdf

//https://forums.raspberrypi.com/viewtopic.php?t=313458