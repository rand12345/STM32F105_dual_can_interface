use defmt::info;
use embassy_stm32::peripherals::PC12;

pub mod can_interfaces;
pub mod process;

pub mod mqtt;

// Misc tasks

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
