use crate::statics::*;
use defmt::Debug2Format;
use defmt::{error, info};
use embassy_stm32::peripherals::*;
use embassy_stm32::usart::Uart;
use embassy_time::{Duration, Timer};
use serde::Serialize;

#[embassy_executor::task]
pub async fn uart_task(uart: Uart<'static, USART3, DMA1_CH2, DMA1_CH3>) {
    let mut uart = uart;
    // turn this into a logger or interface?
    loop {
        Timer::after(Duration::from_secs(MQTT_FREQUENCY_SECS)).await;
        if SEND_MQTT.wait().await {
            if let Some(mut message) = MQTTFMT.lock().await.device_update_msg() {
                message.push('\n').unwrap();
                uart.write(message.as_ref()).await.unwrap();
                info!("UART sent: {}", Debug2Format(&message));
            }
        }
    }
}

#[derive(Clone, Copy, Serialize)]
pub struct MqttFormat {
    soc: f32,
    battery_voltage: f32,
    highest_cell_voltage: f32,
    lowest_cell_voltage: f32,
    highest_cell_temperature: f32,
    lowest_cell_temperature: f32,
    #[serde(with = "BigArray")]
    #[serde(skip)]
    cells_millivolts: [u16; 96],
    #[serde(skip)]
    #[serde(with = "BigArray")]
    cell_balance: [bool; 96],
    current_amps: f32,
    kwh_remaining: f32,
    charge_rate: f32,
    discharge_rate: f32,
    balancing_cells: u8,
    valid: bool,
}

impl MqttFormat {
    pub fn default() -> Self {
        Self {
            soc: 0.0,
            battery_voltage: 0.0,
            highest_cell_voltage: 0.0,
            lowest_cell_voltage: 0.0,
            highest_cell_temperature: 0.0,
            lowest_cell_temperature: 0.0,
            cells_millivolts: [0; 96],
            cell_balance: [false; 96],
            current_amps: 0.0,
            kwh_remaining: 0.0,
            charge_rate: 0.0,
            discharge_rate: 0.0,
            balancing_cells: 0,
            valid: false,
        }
    }
    pub fn update(&mut self, bmsdata: kangoo_battery::Bms) {
        self.soc = bmsdata.soc.into();
        self.battery_voltage = (bmsdata.pack_volts as f32) * 0.1;
        self.highest_cell_voltage = bmsdata.max_volts.into();
        self.lowest_cell_voltage = bmsdata.min_volts.into();
        self.highest_cell_temperature = (bmsdata.temp_max as f32) * 0.1;
        self.lowest_cell_temperature = (bmsdata.temp_min as f32) * 0.1;
        self.cells_millivolts = bmsdata.cells;
        self.cell_balance = bmsdata.bal_cells;
        self.current_amps = (bmsdata.current as f32) * 0.1;
        self.kwh_remaining = (bmsdata.kwh_remaining as f32) * 0.1;
        self.charge_rate = (bmsdata.charge_max as f32) * 0.1;
        self.discharge_rate = (bmsdata.discharge_max as f32) * 0.1;
        self.balancing_cells = bmsdata.balancing_cells;
        self.valid = bmsdata.valid;
    }
    fn device_update_msg(&self) -> Option<serde_json_core::heapless::String<1024>> {
        match serde_json_core::to_string(&self) {
            Ok(string) => {
                info!("Serialiased {} bytes", &string.len());
                Some(string)
            }
            Err(e) => {
                error!("MQTT serialiasation failed: {}", Debug2Format(&e));
                None
            }
        }
    }
}
