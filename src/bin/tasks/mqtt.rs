use crate::statics::*;
use defmt::error;
use defmt::info;
use defmt::Debug2Format;
use embassy_stm32::peripherals::*;
use embassy_stm32::usart::Uart;
use embassy_time::Instant;
use miniserde::__private::String;
use miniserde::{json, Serialize};
#[embassy_executor::task]
pub async fn uart_task(uart: Uart<'static, USART3, DMA1_CH2, DMA1_CH3>) {
    use embassy_futures::select::{select, Either};
    let mut uart = uart;
    if uart.blocking_flush().is_err() {
        panic!();
    };
    let (mut tx, mut rx) = uart.split();
    let mut buf = [0_u8; 512];
    let mut mqtt_frequency = Instant::now();
    loop {
        match select(rx.read_until_idle(&mut buf), SEND_MQTT.wait()).await {
            Either::First(read) => match read {
                Ok(len) => {
                    let mut config = CONFIG.lock().await;
                    if let Err(e) = config.update_from_json(&buf[..len]) {
                        error!("UART deserialise bytes error {}", Debug2Format(&e))
                    } else {
                        info!("Config updated from UART")
                    };
                    buf = [0_u8; 512];
                }
                Err(_) => continue,
            },
            Either::Second(_) => {
                if mqtt_frequency.elapsed().as_secs() < LAST_READING_TIMEOUT_SECS {
                    continue;
                }
                mqtt_frequency = Instant::now();
                buf = [0_u8; 512];
                let mqtt_data = MQTTFMT.lock().await;
                if let Err(e) = tx.write(mqtt_data.device_update_msg().as_bytes()).await {
                    error!("UART send bytes error {}", Debug2Format(&e));
                } else {
                    info!("MQTT sent to UART")
                };
            }
        }
    }
}

#[derive(Clone, Copy, Serialize)]
pub struct MqttFormat {
    soc: f32,
    volts: f32,
    cell_mv_high: f32,
    cell_mv_low: f32,
    cell_temp_high: f32,
    cell_temp_low: f32,
    // #[serde(with = "BigArray")]
    // #[serde(skip)]
    // cells_millivolts: [u16; 96],
    // #[serde(skip)]
    // #[serde(with = "BigArray")]
    // cell_balance: [bool; 96],
    amps: f32,
    kwh: f32,
    charge: f32,
    discharge: f32,
    bal: u8,
    valid: bool,
}

impl MqttFormat {
    pub fn default() -> Self {
        Self {
            soc: 0.0,
            volts: 0.0,
            cell_mv_high: 0.0,
            cell_mv_low: 0.0,
            cell_temp_high: 0.0,
            cell_temp_low: 0.0,
            // cells_millivolts: [0; 96],
            // cell_balance: [false; 96],
            amps: 0.0,
            kwh: 0.0,
            charge: 0.0,
            discharge: 0.0,
            bal: 0,
            valid: false,
        }
    }
    pub fn update(&mut self, bmsdata: kangoo_battery::Bms) {
        self.soc = bmsdata.soc as f32;
        self.volts = (bmsdata.pack_volts as f32) * 0.1;
        self.cell_mv_high = bmsdata.max_volts as f32;
        self.cell_mv_low = bmsdata.min_volts as f32;
        self.cell_temp_high = (bmsdata.temp_max as f32) * 0.1;
        self.cell_temp_low = (bmsdata.temp_min as f32) * 0.1;
        // self.cells_millivolts = bmsdata.cells;
        // self.cell_balance = bmsdata.bal_cells;
        self.amps = (bmsdata.current as f32) * 0.1;
        self.kwh = (bmsdata.kwh_remaining as f32) * 0.1;
        self.charge = (bmsdata.charge_max as f32) * 0.1;
        self.discharge = (bmsdata.discharge_max as f32) * 0.1;
        self.bal = bmsdata.balancing_cells;
        self.valid = bmsdata.valid;
    }
    fn device_update_msg(&self) -> String {
        json::to_string(&self)
    }
}
