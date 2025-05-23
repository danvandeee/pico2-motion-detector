#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

use super::led::LedStatus;

pub fn usb_communication(
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

pub fn write_serial_string(text: &str, serial: &mut SerialPort<'_, rp235x_hal::usb::UsbBus>) {
    write_serial_data(text.as_bytes(), serial, text.len());
}

pub fn write_serial_data(buf: &[u8], serial: &mut SerialPort<'_, rp235x_hal::usb::UsbBus>, count: usize) {
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