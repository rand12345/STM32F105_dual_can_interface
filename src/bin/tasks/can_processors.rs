use crate::statics::*;
use defmt::error;
use defmt::info;
use defmt::warn;
use defmt::Debug2Format;
use embassy_stm32::can::bxcan::Frame;

#[allow(unused_assignments)]
#[cfg(feature = "solax")]
#[embassy_executor::task]
pub async fn inverter_rx() -> ! {
    warn!("Starting Inverter Processor");
    let mut inverter_comms_valid = false;

    let recv = INVERTER_CHANNEL_RX.receiver();
    let trans = INVERTER_CHANNEL_TX.sender();
    loop {
        let frame = recv.recv().await;
        inverter_comms_valid = false;

        if LAST_BMS_MESSAGE.lock().await.elapsed().as_secs() > LAST_READING_TIMEOUT_SECS {
            error!("BMS last update timeout, inverter communications stopped");
            CONTACTOR_STATE.signal(inverter_comms_valid);
            continue;
        };

        let response = {
            let mut solax = INVERTER_DATA.lock().await;
            solax.parser(frame)
        };

        inverter_comms_valid = match response {
            Ok(frames) => {
                info!("Sending {} frames to inverter", frames.len());
                for frame in frames {
                    trans.send(frame).await;
                }
                // Send signal to push json data to UART
                SEND_MQTT.signal(true);
                true
            }
            Err(e) => {
                match e {
                    solax_can_bus::SolaxError::InvalidData => {
                        error!("Invalid data");
                        warn!(
                            "Inverter data: {:?}",
                            Debug2Format(&*INVERTER_DATA.lock().await)
                        );
                        true
                    }
                    solax_can_bus::SolaxError::BadId(id) => {
                        error!("Critical: unexpected frame in inverter can data {:02x}", id);
                        false // disable contactor
                    }
                    solax_can_bus::SolaxError::InvalidFrameEncode(id) => {
                        error!("Critical: frame encoding failed for {:02x}", id);
                        false // disable contactor
                    }
                    solax_can_bus::SolaxError::TimeStamp(time) => {
                        info!("Inverter time: {}", time);
                        true
                    }
                    solax_can_bus::SolaxError::InvalidTimeData => {
                        warn!("InvalidTimeData");
                        true // not critical
                    }
                    solax_can_bus::SolaxError::UnwantedFrame => true,
                    // _ => true,
                }
            }
        };

        CONTACTOR_STATE.signal(inverter_comms_valid);
    }
}

#[cfg(feature = "kangoo")]
#[embassy_executor::task]
pub async fn bms_tx_periodic() {
    use embassy_futures::select::{select3, Either3};
    use embassy_time::{Duration, Ticker};
    use kangoo_battery::*;
    let tx = BMS_CHANNEL_TX.sender();
    let ticker_ms = |ms| Ticker::every(Duration::from_millis(ms));
    let sender = |frame| {
        if let Err(_e) = tx.try_send(frame) {
            error!("Periodic queue buf error")
        };
    };
    warn!("Starting BMS TX periodic");
    let mut t1 = ticker_ms(100);
    let mut t2 = ticker_ms(5050);
    let mut t3 = ticker_ms(11025);
    loop {
        let frame: Frame = match select3(t1.next(), t2.next(), t3.next()).await {
            Either3::First(_) => request_init().unwrap(),
            Either3::Second(_) => request_tx_frame(RequestMode::CellBank1).unwrap(),
            Either3::Third(_) => request_tx_frame(RequestMode::Balance).unwrap(),
        };
        sender(frame);
    }
}

#[allow(unused_assignments)]
#[cfg(feature = "kangoo")]
#[embassy_executor::task]
pub async fn bms_rx() {
    use embassy_stm32::can::bxcan::Id;
    use embassy_stm32::can::bxcan::Id::Standard;
    use embassy_time::Instant;
    use kangoo_battery::bms::Bms;

    let rx = BMS_CHANNEL_RX.receiver();
    let tx = BMS_CHANNEL_TX.sender();
    let mut bms_validated = Bms::new();
    let mut data = kangoo_battery::Data::new();
    let mut update_inverter = false;
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

        // warn!("{}: {:?}", id, Debug2Format(&frame.data()));

        // if !id == 0x7bb { // weird rust bug on nightly?
        if id != 1979 {
            update_inverter = match data.rapid_data_processor(frame) {
                Ok(state) => state, // data can be parsed without diag data is true - use a different validity checker
                Err(_e) => {
                    warn!("Rapid data parsing error"); //: {:?}", Debug2Format(&e));
                    continue;
                }
            };
            if update_inverter {
                *LAST_BMS_MESSAGE.lock().await = Instant::now();
                info!("BMS watchdog reset")
            }
        } else {
            match data.diag_data_processor(frame) {
                Ok(None) => {
                    if let Err(_e) = bms_validated.update_bms_data(&data) {
                        warn!("BMS data update error"); //: {:?}", Debug2Format(&e));
                    }
                }
                Ok(Some(next_tx_frame)) => {
                    tx.send(next_tx_frame).await;
                    continue;
                }
                Err(e) => {
                    warn!("BMS diag error: {:?}", Debug2Format(&e));
                }
            };
        }

        if update_inverter {
            push_bms_to_solax(bms_validated).await;
            info!("Pushed values to Inverter data store");
            push_all_to_mqtt(bms_validated).await;
            info!("Push values to MQTT data store");
            update_dod(&mut bms_validated).await;
        }
        update_inverter = false;
    }
}

#[inline]
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
    solax_data.capacity = bmsdata.soc as u16;
    solax_data.kwh = bmsdata.kwh_remaining as f32 * 0.1;
    solax_data.cell_temp_min = bmsdata.temp_min as f32 * 0.1;
    solax_data.cell_temp_max = bmsdata.temp_max as f32 * 0.1;
    solax_data.int_temp = bmsdata.temp_avg as f32 * 0.1;
    solax_data.cell_voltage_min = bmsdata.min_volts; //cell as millivolts
    solax_data.cell_voltage_max = bmsdata.max_volts;
    solax_data.wh_total = 10000;
    solax_data.contactor = true;
    solax_data.v_max = bmsdata.max_volts as f32; // cell as millivolts
    solax_data.v_min = bmsdata.min_volts as f32;
    solax_data.valid = bmsdata.valid;
    // solax_data.timestamp = Some(Instant::now());
}

#[inline]
async fn push_all_to_mqtt(bmsdata: kangoo_battery::Bms) {
    MQTTFMT.lock().await.update(bmsdata);
}

#[inline]
async fn update_dod(bmsdata: &mut kangoo_battery::Bms) {
    let config = CONFIG.lock().await;
    bmsdata.set_dod(config.dod.min(), config.dod.max());
}
