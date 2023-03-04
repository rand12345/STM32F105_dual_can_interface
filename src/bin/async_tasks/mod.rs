use crate::statics::*;
use defmt::error;
use defmt::info;
use defmt::warn;
use defmt::Debug2Format;
use embassy_stm32::can::bxcan::Frame;
use embassy_stm32::peripherals::*;
use embassy_stm32::usart::Uart;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Instant;
use lazy_static::lazy_static;
use serde::Serialize;
use serde_big_array::BigArray;
use solax_can_bus::SolaxBms;

pub type InverterDataMutex = embassy_sync::mutex::Mutex<CriticalSectionRawMutex, SolaxBms>;

pub type MqttFmtMutex = embassy_sync::mutex::Mutex<CriticalSectionRawMutex, MqttFormat>;
lazy_static! {
    pub static ref MQTTFMT: MqttFmtMutex = embassy_sync::mutex::Mutex::new(MqttFormat::new());
    pub static ref INVERTER_DATA: InverterDataMutex =
        embassy_sync::mutex::Mutex::new(SolaxBms::default());
}

#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum CommsState {
    CAN1RX,
    CAN1TX,
    CAN2RX,
    CAN2TX,
    UpdateMqtt,
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
    cells_millivolts: [u16; 96],
    #[serde(with = "BigArray")]
    cell_balance: [bool; 96],
    current_amps: f32,
    kwh_remaining: f32,
    charge_rate: f32,
    discharge_rate: f32,
    valid: bool,
}

impl MqttFormat {
    fn new() -> Self {
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
            valid: false,
        }
    }
    fn device_update_msg(&self) -> Result<heapless::String<512>, serde_json::Error> {
        let json_msg = serde_json::json!(&self);
        let json_string = serde_json::to_string(&json_msg)?;
        let string: heapless::String<512> = json_string.chars().take(512).collect();
        Ok(string)
    }
}

enum Safe {
    Yes,
    No,
}

impl From<bool> for Safe {
    fn from(v: bool) -> Safe {
        if v {
            Safe::Yes
        } else {
            Safe::No
        }
    }
}

#[allow(unused_assignments)]
#[cfg(feature = "solax")]
#[embassy_executor::task]
pub async fn inverter_rx_processor() {
    use embassy_time::{Duration, Instant};

    let timeout = Duration::from_secs(30);
    warn!("Starting Inverter Processor");
    let mut communications_valid = Safe::No;

    let recv = INVERTER_CHANNEL_RX.receiver();
    let trans = INVERTER_CHANNEL_TX.sender();
    let mut timer = Instant::now();
    loop {
        if timer.elapsed().as_secs() > 5 {
            // unsafe {
            //     let free = esp_idf_sys::uxTaskGetStackHighWaterMark(
            //         esp_idf_sys::xTaskGetCurrentTaskHandle(),
            //     );
            //     warn!("Free stack = {free}")
            // };
            timer = Instant::now()
        }
        // This is a server
        let frame = recv.recv().await;

        let response = {
            let mut solax = INVERTER_DATA.lock().await;
            let r = solax.parser(frame, timeout);
            communications_valid = solax.is_valid().into(); // spweing out unwanted log! messages
            r
        };

        // arkward but allows RWLock drop on solax_data
        match response {
            Ok(frames) => {
                for frame in frames.iter() {
                    trans.send(frame.clone()).await;
                }
                // Send signal to push json data to UART
                STATE.signal(CommsState::UpdateMqtt);
            }

            Err(e) => match e {
                solax_can_bus::SolaxError::InvalidData => warn!("Invalid Data"),
                solax_can_bus::SolaxError::BadId(id) => warn!("Bad Can Id {}", id),
                solax_can_bus::SolaxError::InvalidFrameEncode(id) => {
                    error!("Critical: frame encoding failed for {:02x}", id);
                    // maybe halt?
                }
                solax_can_bus::SolaxError::TimeStamp(time) => info!("Inverter time: {}", time),
                solax_can_bus::SolaxError::InvalidTimeData => warn!("InvalidTimeData"),
            },
        }

        // Apply state change to contactor

        match communications_valid {
            Safe::Yes => CONTACTOR_STATE.signal(true),
            Safe::No => CONTACTOR_STATE.signal(false),
        };
    }
}

#[allow(unused_assignments)]
#[cfg(feature = "kangoo")]
#[embassy_executor::task]
pub async fn bms_rx_processor() {
    use embassy_stm32::can::bxcan::Id;
    use embassy_stm32::can::bxcan::Id::Standard;

    let rx = BMS_CHANNEL_RX.receiver();
    let tx = BMS_CHANNEL_TX.sender();
    let mut bms_validated = kangoo_battery::Bms::new();
    let mut data = kangoo_battery::Data::default();
    let mut update_solax = false;
    warn!("Starting Kangoo Rx Processor");
    let canid = |frame: &Frame| -> Option<u16> {
        match frame.id() {
            Standard(id) => Some(id.as_raw()),
            Id::Extended(_) => None,
        }
    };
    loop {
        let frame = rx.recv().await;
        // Process 10ms data
        let id = match canid(&frame) {
            Some(id) => id,
            None => continue,
        };
        if !id == 0x7bb {
            update_solax = match data.rapid_data_processor(frame) {
                Ok(state) => state, // data can be parsed without diag data is true - use a different validity checker
                Err(e) => {
                    warn!("Rapid data parsing error: {:?}", Debug2Format(&e));
                    continue;
                }
            }
        } else {
            update_solax = match data.diag_data_processor(frame) {
                Ok(None) => match bms_validated.update_bms_data(&data) {
                    Ok(_) => true,
                    Err(e) => {
                        warn!("BMS data update error: {:?}", Debug2Format(&e));
                        false
                    }
                },
                Ok(Some(next_tx_frame)) => {
                    tx.send(next_tx_frame).await;
                    continue; // break
                }
                Err(e) => {
                    warn!("BMS diag error: {:?}", Debug2Format(&e));
                    false
                }
            };
        }
        if update_solax {
            push_bms_to_solax(bms_validated).await;
            info!("Pushed values to Inverter data store");
            push_all_to_mqtt(bms_validated).await;
            info!("Push values to MQTT data store");
            update_solax = false;
        }
    }
}

async fn push_bms_to_solax(bmsdata: kangoo_battery::Bms) {
    /*
        slave_voltage_max: 4000,
    slave_voltage_min: 2900,
    charge_max: 660,
    discharge_max: 350,
    voltage: 3528,
    current: 0,
    capacity: 50,
    kwh: 1700,
    cell_temp_min: 0,
    cell_temp_max: 0,
    cell_voltage_min: 36,
    cell_voltage_max: 37,
    pack_voltage_max: 4100,
    wh_total: 10000,
    contactor: true,
    int_temp: 0,
    v_max: 3681,
    v_min: 3671,

    */
    let mut solax_data = INVERTER_DATA.lock().await;
    // convert everything to standard units, volts, millivolts, etc
    solax_data.pack_voltage_max = bmsdata.pack_voltage_max as f32 * 0.1;
    solax_data.slave_voltage_max = bmsdata.slave_voltage_max as f32 * 0.1;
    solax_data.slave_voltage_min = bmsdata.slave_voltage_min as f32 * 0.1;
    solax_data.charge_max = bmsdata.charge_max as f32 * 0.1;
    solax_data.discharge_max = bmsdata.discharge_max as f32 * 0.1;
    solax_data.voltage = bmsdata.pack_volts as f32 * 0.1;
    solax_data.current = bmsdata.current as f32 * 0.1; // 100mA = 1
    solax_data.capacity = bmsdata.soc;
    solax_data.kwh = bmsdata.kwh_remaining as f32 * 0.1;
    solax_data.cell_temp_min = bmsdata.temp_min as f32 * 0.1;
    solax_data.cell_temp_max = bmsdata.temp_max as f32 * 0.1;
    solax_data.int_temp = bmsdata.temp_avg as f32 * 0.1;
    solax_data.cell_voltage_min = bmsdata.min_volts; //cell as millivolts
    solax_data.cell_voltage_max = bmsdata.max_volts;
    solax_data.wh_total = 10000;
    solax_data.contactor = bmsdata.contactor;
    solax_data.v_max = bmsdata.max_volts as f32; // cell as millivolts
    solax_data.v_min = bmsdata.min_volts as f32;
    solax_data.valid = bmsdata.valid;
    solax_data.timestamp = Some(Instant::now());
}

async fn push_all_to_mqtt(bmsdata: kangoo_battery::Bms) {
    let mut mqtt = MQTTFMT.lock().await;
    mqtt.soc = bmsdata.soc.into();
    mqtt.battery_voltage = (bmsdata.pack_volts as f32) * 0.1;
    mqtt.highest_cell_voltage = bmsdata.max_volts.into();
    mqtt.lowest_cell_voltage = bmsdata.min_volts.into();
    mqtt.highest_cell_temperature = (bmsdata.temp_max as f32) * 0.1;
    mqtt.lowest_cell_temperature = (bmsdata.temp_min as f32) * 0.1;
    mqtt.cells_millivolts = bmsdata.cells;
    mqtt.cell_balance = bmsdata.bal_cells;
    mqtt.current_amps = (bmsdata.current as f32) * 0.1;
    mqtt.kwh_remaining = (bmsdata.kwh_remaining as f32) * 0.1;
    mqtt.charge_rate = (bmsdata.charge_max as f32) * 0.1;
    mqtt.discharge_rate = (bmsdata.discharge_max as f32) * 0.1;
    mqtt.valid = bmsdata.valid;
}

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
            (true, true) => {}
            (true, false) => {
                warn!("Contactor shutdown");
                pwm.disable(TimerChannel::Ch1);
                info!("Contactor disabled");
                active = false;
            }
            (false, true) => {
                warn!("Activate 100% duty, wait 100ms, set duty to 50%, set active to true");
                pwm.enable(TimerChannel::Ch1);
                info!("Contactor enabled");
                pwm.set_duty(TimerChannel::Ch1, max);
                info!("Contactor at 100%");
                Timer::after(Duration::from_millis(100)).await;
                pwm.set_duty(TimerChannel::Ch1, max / 2);
                info!("Contactor at hold 50%");
                active = true
            }
            (false, false) => {}
        }
    }
}

#[embassy_executor::task]
pub async fn activity_led(led: PC12) {
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

#[embassy_executor::task]
pub async fn uart_task(uart: Uart<'static, USART3, DMA1_CH2, DMA1_CH3>) {
    use embassy_time::{Duration, Timer};

    let mut uart = uart;
    loop {
        let state = STATE.wait().await;
        let message = match state {
            CommsState::UpdateMqtt => {
                let mqtt_data = { MQTTFMT.lock().await };
                if let Ok(string) = mqtt_data.device_update_msg() {
                    string
                } else {
                    warn!("MQTT serialise failed");
                    continue;
                }
            }
            _ => continue,
        };
        uart.write(message.as_bytes()).await.unwrap();
        Timer::after(Duration::from_millis(1000)).await
    }
}
