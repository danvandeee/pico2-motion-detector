//! # GPIO 'usb' Example
//!
//! This application demonstrates how to control a GPIO pin on the rp235x.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

use rp235x_hal::clocks::ClockSource;
use rp235x_hal::gpio::bank0::Gpio14;
use rp235x_hal::gpio::{FunctionSio, Pin, PullDown, SioOutput};
use rp235x_hal::timer::CopyableTimer0;
// Alias for our HAL crate
use rp235x_hal as hal;

// Some things we need
use core::fmt::Write;
use heapless::String;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

// Some things we need
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

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

    let mut timer: rp235x_hal::Timer<CopyableTimer0> = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USB,
        pac.USB_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Danvandeee bv")
            .product("Serial port dan")
            .serial_number("TEST DAN")])
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let mut said_hello = false;



   

    // Configure GPIO14 as an output
    let mut led: LedStatus = LedStatus::new(pins.gpio14.into_push_pull_output(), timer);


    led.set_high();
    timer.delay_ms(200); // 200 ms delay
    led.set_low();
    timer.delay_ms(200); // 200 ms delay

    let last_toggle_time = timer.get_counter().ticks();
    
    loop {
        
        // A welcome message at the beginning
        if !said_hello && timer.get_counter().ticks() - last_toggle_time >= 8_000_000 {
            said_hello = true;
            let _ = serial.write(b"Hello, World!\r\n");

            let time = timer.get_counter().ticks();
            let mut text: String<64> = String::new();
            writeln!(&mut text, "Current timer ticks: {}", time).unwrap();

            // This only works reliably because the number of bytes written to
            // the serial port is smaller than the buffers available to the USB
            // peripheral. In general, the return value should be handled, so that
            // bytes not transferred yet don't get lost.
            let _ = serial.write(text.as_bytes());

            led.set_high();
        }

        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                    led.set_high();
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    // We have data to process
                    // Blink the LED to show we are processing data
                    led.set_high();

                    // Convert to upper case
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });
                    // Send back to the host
                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }
                }
            }
        }

        led.set_low();
    }
}


/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"Blinky Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

struct LedStatus {
    led_active: bool,
    last_toggle_time: u64,
    led_pin: Pin<Gpio14, FunctionSio<SioOutput>, PullDown>,
    cooldown_time: u64,
    timer: rp235x_hal::Timer<CopyableTimer0>
}

impl LedStatus {
    fn new(led_pin: Pin<Gpio14, FunctionSio<SioOutput>, PullDown>, timer: rp235x_hal::Timer<CopyableTimer0>) -> Self {
        Self {
            led_active: false,
            last_toggle_time: timer.get_counter().ticks(),
            led_pin,
            cooldown_time: 500_000, // 0.5 second cooldown time
            timer
        }
    }

    fn set_high(&mut self, ) {
        let current_time = self.timer.get_counter().ticks();

        if !self.led_active && current_time - self.last_toggle_time >= self.cooldown_time { 
                self.led_pin.set_high().unwrap();
                self.led_active = true;
                self.last_toggle_time = current_time;
        }
    }

    fn set_low(&mut self, ) {
        let current_time = self.timer.get_counter().ticks();

        if self.led_active && current_time - self.last_toggle_time >= self.cooldown_time { 
                self.led_pin.set_low().unwrap();
                self.led_active = false;
                self.last_toggle_time = current_time;
                self.reset_cooldown();
        }
    }

    fn reset_cooldown(&mut self) {
        self.cooldown_time = 500_000; // Reset cooldown time to 0.5 second
    }
}

// End of file
