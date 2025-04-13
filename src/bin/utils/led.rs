#![no_std]
#![no_main]

    use panic_halt as _;

    // Alias for our HAL crate
    use rp235x_hal::gpio::bank0::Gpio14;
    use rp235x_hal::gpio::{FunctionSio, Pin, PullDown, SioOutput};
    use rp235x_hal::timer::CopyableTimer0;
    use embedded_hal::digital::OutputPin;

    pub enum CoolDownTime {
        OneTenthSecond = 100_000, // 0.1 second cooldown time
        HalfSecond = 500_000, // 0.5 second cooldown time
        OneSecond = 1_000_000, // 1 second cooldown time
    }
    
    impl CoolDownTime {
        fn get(&self) -> u64 {
            match self {
                CoolDownTime::OneTenthSecond => 100_000,
                CoolDownTime::HalfSecond => 500_000,
                CoolDownTime::OneSecond => 1_000_000
            }
        }
        
    }

    pub struct LedStatus {
        led_active: bool,
        last_toggle_time: u64,
        led_pin: Pin<Gpio14, FunctionSio<SioOutput>, PullDown>,
        cooldown_time: u64,
        timer: rp235x_hal::Timer<CopyableTimer0>,
        repeat: u64,
    }
    
    impl LedStatus {
        pub fn new(led_pin: Pin<Gpio14, FunctionSio<SioOutput>, PullDown>, timer: rp235x_hal::Timer<CopyableTimer0>) -> Self {
            Self {
                led_active: false,
                last_toggle_time: timer.get_counter().ticks(),
                led_pin,
                cooldown_time: 500_000, // 0.5 second cooldown time
                timer,
                repeat: 0,
            }
        }
    
        pub fn repeat_if_needed(&mut self) {
            if self.repeat > 0 {
                if self.led_active {
                    self.set_low();
                } else if !self.led_active && self.set_high() {
                    self.repeat -= 1;
                }
    
            } else {
                self.repeat = 0;
                self.set_low();
                self.reset_cooldown();
            }
        }
    
        pub fn set_high(&mut self) -> bool {
        // fn set_high(&mut self) {
            let current_time = self.timer.get_counter().ticks();
    
            if !self.led_active && current_time - self.last_toggle_time >= self.cooldown_time { 
                    self.led_pin.set_high().unwrap();
                    self.led_active = true;
                    self.last_toggle_time = current_time;
                    return true;
            }
            return false;
        }
    
        pub fn set_high_repeatable(&mut self, repeat: u64) {
            self.repeat = repeat;
        }
    
        pub fn set_high_repeatable_custom_speed(&mut self, repeat: u64, cooldown: CoolDownTime) {
            self.repeat = repeat;
            self.cooldown_time = cooldown.get(); // Set the cooldown time based on the enum value  
        }
    
        pub fn set_low(&mut self, ) {
            let current_time = self.timer.get_counter().ticks();
    
            if self.led_active && current_time - self.last_toggle_time >= self.cooldown_time { 
                    self.led_pin.set_low().unwrap();
                    self.led_active = false;
                    self.last_toggle_time = current_time;
                    // self.reset_cooldown();
            }
        }
    
        pub fn reset_cooldown(&mut self) {
            if !self.led_active {
                self.cooldown_time = CoolDownTime::HalfSecond.get(); // Reset cooldown time to 0.5 second
            }
           
        }
    }