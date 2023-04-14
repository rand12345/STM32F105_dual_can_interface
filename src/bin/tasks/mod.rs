use defmt::{info, warn};
use embassy_stm32::peripherals::{PA15, PC12, TIM2};

use crate::statics::CONTACTOR_STATE;

pub mod can_interfaces;
pub mod process;

pub mod mqtt;

// Misc tasks

#[embassy_executor::task]
pub async fn contactor_task(pin: PA15, timer: TIM2) {
    use embassy_stm32::pwm::simple_pwm::{PwmPin, SimplePwm};
    use embassy_stm32::pwm::Channel as TimerChannel;
    use embassy_stm32::time::khz;
    use embassy_time::{Duration, Timer};
    let ch1 = PwmPin::new_ch1(pin);
    let mut pwm = SimplePwm::new(timer, Some(ch1), None, None, None, khz(1));
    let max = pwm.get_max_duty() - 1;
    let mut active = false;
    loop {
        let state = CONTACTOR_STATE.wait().await;
        match (state, active) {
            (false, true) => {
                warn!("Contactor shutdown");
                pwm.disable(TimerChannel::Ch1);
                info!("Contactor disabled");
                active = false;
            }
            (true, false) => {
                warn!("Activate 100% duty, wait 100ms, set duty to 50%, set active to true");
                pwm.enable(TimerChannel::Ch1);
                info!("Contactor enabled");
                pwm.set_duty(TimerChannel::Ch1, max);
                info!("Contactor at 100%");
                Timer::after(Duration::from_millis(100)).await;
                pwm.set_duty(TimerChannel::Ch1, (max / 4) * 3);
                info!("Contactor at hold 50%");
                active = true
            }
            (true, true) => info!("Contactor holding"),
            _ => (),
        }
    }
}

#[embassy_executor::task]
pub async fn led_task(led: PC12) {
    use embassy_stm32::gpio::{Level, Output, Speed};
    use embassy_time::{Duration, Timer};
    let mut led = Output::new(led, Level::Low, Speed::Medium);
    info!("Spawn activity LED");
    loop {
        // change on static state
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}

// #[embassy_executor::task]
// pub async fn init(instance: IWDG, timeout_us: u32) {
//     use crate::statics::WDT;
//     use embassy_stm32::wdg::IndependentWatchdog;
//     let mut wdt = IndependentWatchdog::new(instance, timeout_us); // 1sec
//     unsafe {
//         wdt.unleash();
//     }
//     info!("Watchdog started");
//     loop {
//         // await a signal and pet the dog, timeout triggers device reset
//         let signal = WDT.wait().await;
//         if signal {
//             unsafe {
//                 wdt.pet();
//             }
//         }
//     }
// }
