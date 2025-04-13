//! # GPIO 'usb' Example

#![no_std]
#![no_main]

mod utils;

use utils::led::CoolDownTime as CoolDownTime;
use utils::led::LedStatus as LedStatus;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;


// Alias for our HAL crate
use rp235x_hal as hal;
use rp235x_hal::gpio::bank0::Gpio14;
use rp235x_hal::gpio::{FunctionSio, Pin, PullDown, SioOutput};
use rp235x_hal::timer::CopyableTimer0;

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
    let mut serial: SerialPort<'_, rp235x_hal::usb::UsbBus> = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev: UsbDevice<'_, rp235x_hal::usb::UsbBus> = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Danvandeee bv")
            .product("Serial port dan")
            .serial_number("TEST DAN")])
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let mut said_hello: bool = false;

    // Configure GPIO14 as an output
    let mut led: LedStatus = LedStatus::new(pins.gpio14.into_push_pull_output(), timer);


    led.set_high();
    timer.delay_ms(500); // 500 ms delay
    led.set_low();
    timer.delay_ms(500); // 500 ms delay
    led.set_high_repeatable_custom_speed(5, CoolDownTime::OneTenthSecond); // Set the LED to blink 3 times with a 0.1 second cooldown time

    let last_toggle_time = timer.get_counter().ticks();
    loop {
        said_hello = usb_communication_say_hello(said_hello, last_toggle_time, &mut timer, &mut serial, &mut led);
        usb_communication(&mut serial, &mut usb_dev, &mut led);
        led.repeat_if_needed();
    }
}

fn usb_communication_say_hello(
    said_hello: bool, 
    last_toggle_time: u64, 
    timer: &mut rp235x_hal::Timer<CopyableTimer0>, 
    serial: &mut SerialPort<'_, rp235x_hal::usb::UsbBus>, 
    led: &mut LedStatus) -> bool {

    if said_hello {
        return true;
    }

    // A welcome message at the beginning
    if !said_hello && timer.get_counter().ticks() - last_toggle_time >= 8_000_000 {
        write_serial_string(&"Hello, World!\r\n", serial);

        let time = timer.get_counter().ticks();
        let mut text: String<64> = String::new();
        writeln!(&mut text, "Current timer ticks: {}", time).unwrap();
        write_serial_string(&text, serial);
        led.set_high();
        return true;
    } else {
        return false;
    }
}

fn usb_communication(
    serial: &mut SerialPort<'_, rp235x_hal::usb::UsbBus>, 
    usb_dev: &mut UsbDevice<'_, rp235x_hal::usb::UsbBus>, 
    led: &mut LedStatus) {

    // Check for new data
    if usb_dev.poll(&mut [ serial] ) {
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

                write_serial_string(&"\r\nReceived: ", serial);
                write_serial_data(&buf, serial, count);
            }
        }
    }
}

fn write_serial_string(text: &str, serial: &mut SerialPort<'_, rp235x_hal::usb::UsbBus>) {
    write_serial_data(text.as_bytes(), serial, text.len());
}

fn write_serial_data(buf: &[u8], serial: &mut SerialPort<'_, rp235x_hal::usb::UsbBus>, count: usize) {
    let mut wr_ptr: &[u8] = &buf[..count];
    while !wr_ptr.is_empty() {
        match serial.write(wr_ptr) {
            Ok(len) => wr_ptr = &wr_ptr[len..],
            Err(_) => {
                break;
            },
        };
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

// End of file
