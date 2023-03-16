use crate::statics::*;
use defmt::{error, info, warn, Debug2Format};

#[cfg(feature = "ze50")]
#[embassy_executor::task]
pub async fn bms_tx_periodic() {
    use embassy_futures::select::{select, Either};
    use embassy_stm32::can::bxcan::Frame;
    use embedded_hal::can::Frame as _;
    // use embedded_hal::can::Id;
    use embedded_hal::can::{ExtendedId, Id, StandardId};

    use embassy_time::{Duration, Ticker};
    use renault_zoe_ph2_battery::{init_payloads, preamble_payloads};
    let tx = BMS_CHANNEL_TX.sender();
    let ticker_ms = |ms| Ticker::every(Duration::from_millis(ms));
    let sender = |frame| {
        if let Err(_e) = tx.try_send(frame) {
            error!("Periodic queue buf error")
        };
    };

    ticker_ms(2000).next().await;
    let mut preamble_frame_number_1 = true;
    let preamble_payloads = preamble_payloads();
    warn!("Starting BMS TX periodic");

    // send init
    for payload in init_payloads() {
        ticker_ms(200).next().await;
        let frame = Frame::new(Id::Standard(StandardId::new(0x373).unwrap()), &payload);
        sender(frame.unwrap());
    }

    let mut t1 = ticker_ms(200);
    let mut t2 = ticker_ms(225);
    loop {
        let frame: Frame = match select(t1.next(), t2.next()).await {
            Either::First(_) => {
                let payload = {
                    if preamble_frame_number_1 {
                        preamble_frame_number_1 = false;
                        preamble_payloads[0]
                    } else {
                        preamble_frame_number_1 = true;
                        preamble_payloads[1]
                    }
                };
                Frame::new(Id::Standard(StandardId::new(0x373).unwrap()), &payload).unwrap()
            }
            Either::Second(_) => {
                // ticker_ms(50).next().await;
                // read the current ZE50_DATA mode
                let pid = { ZE50_DATA.lock().await.req_code };
                if pid == 0x1 {
                    // push vals to ZE50_BMS struct when reading loop has finished
                    let data = ZE50_DATA.lock().await;
                    let mut bms = ZE50_BMS.lock().await;
                    if let Err(e) = bms.update_bms_data(&data) {
                        error!("BMS value parsing failed: {}", Debug2Format(&e))
                    } else {
                        push_bms_to_inverter(*bms).await;
                        info!("{}", Debug2Format(&*bms));
                        let config = CONFIG.lock().await;
                        bms.set_dod(config.dod.min(), config.dod.max());
                    };
                }
                let pid_id = if pid == 0x5d {
                    // current 100ms sampling
                    0x92
                } else if pid == 0xc8 {
                    // kWh remaining
                    0x91
                } else {
                    0x90
                };
                Frame::new(
                    embedded_hal::can::Id::Extended(ExtendedId::new(0x18DADBF1).unwrap()),
                    &[0x03, 0x22, pid_id, pid, 0xff, 0xff, 0xff, 0xff],
                )
                .unwrap()
            }
        };
        sender(frame);
    }
}

#[cfg(feature = "ze50")]
#[allow(unused_assignments)]
#[embassy_executor::task]
pub async fn bms_rx() {
    use defmt::info;
    use embassy_stm32::can::bxcan::{Frame, Id::Extended};
    let (mut _min, mut _max, _pid) = (u32::MAX, u32::MIN, 0u8);
    let rx = BMS_CHANNEL_RX.receiver();
    loop {
        let frame: Frame = rx.recv().await;
        if let Extended(id) = frame.id() {
            // info!("RX {}", frame);
            if id.as_raw() == !0x18DAF1DB {
                info!("Unknown Extended ID - RX: {:02x}", id.as_raw());
                continue;
            }
            if frame.data().is_none() {
                continue;
            }
            let mut data = ZE50_DATA.lock().await;
            // process_payload into Data struct
            if let Err(e) = data.process_payload(frame.data().unwrap()) {
                match e {
                    renault_zoe_ph2_battery::BmsError::InvalidData => (),
                    renault_zoe_ph2_battery::BmsError::RangeError(_) => (),
                    renault_zoe_ph2_battery::BmsError::IsoTpError => (),
                    renault_zoe_ph2_battery::BmsError::InvalidCanData => (),
                    renault_zoe_ph2_battery::BmsError::InvalidTemperature(_) => (),
                    renault_zoe_ph2_battery::BmsError::PackVolts(_) => (),
                    renault_zoe_ph2_battery::BmsError::Current(_) => (),
                    renault_zoe_ph2_battery::BmsError::UnknownCanData => (),
                }
            };
        } else {
            // error!("Found standard Id on ZE50 can line");
        }
    }
}

#[cfg(feature = "solax")]
#[inline]
async fn push_bms_to_inverter(bmsdata: kangoo_battery::Bms) {
    let mut inverter_data = INVERTER_DATA.lock().await;
    // convert everything to standard units, volts, millivolts, etc
    inverter_data.pack_voltage_max = bmsdata.pack_voltage_max as f32 * 0.1;
    inverter_data.slave_voltage_max = bmsdata.slave_voltage_max as f32 * 0.1;
    inverter_data.slave_voltage_min = bmsdata.slave_voltage_min as f32 * 0.1;
    inverter_data.charge_max = bmsdata.charge_max as f32 * 0.1;
    inverter_data.discharge_max = bmsdata.discharge_max as f32 * 0.1;
    inverter_data.voltage = bmsdata.pack_volts as f32 * 0.1;
    inverter_data.current = bmsdata.current as f32 * 0.1; // 100mA = 1
    inverter_data.capacity = bmsdata.soc as u16;
    inverter_data.kwh = bmsdata.kwh_remaining as f32 * 0.1;
    inverter_data.cell_temp_min = bmsdata.temp_min as f32 * 0.1;
    inverter_data.cell_temp_max = bmsdata.temp_max as f32 * 0.1;
    inverter_data.int_temp = bmsdata.temp_avg as f32 * 0.1;
    inverter_data.cell_voltage_min = bmsdata.min_volts; //cell as millivolts
    inverter_data.cell_voltage_max = bmsdata.max_volts;
    inverter_data.wh_total = 10000;
    inverter_data.contactor = true;
    inverter_data.v_max = bmsdata.max_volts as f32; // cell as millivolts
    inverter_data.v_min = bmsdata.min_volts as f32;
    inverter_data.valid = bmsdata.valid;
    // solax_data.timestamp = Some(Instant::now());
}
#[cfg(feature = "pylontech")]
#[inline]
async fn push_bms_to_inverter(bmsdata: renault_zoe_ph2_battery::bms::Bms) {
    let mut inverter_data = INVERTER_DATA.lock().await;
    // convert everything to standard units, volts, millivolts, etc
    inverter_data.pack_voltage_max = bmsdata.pack_voltage_max as f32 * 0.1;
    inverter_data.charge_voltage_max = bmsdata.slave_voltage_max as f32 * 0.1;
    inverter_data.charge_voltage_min = bmsdata.slave_voltage_min as f32 * 0.1;
    inverter_data.charge_max = bmsdata.charge_max as f32 * 0.1;
    inverter_data.discharge_max = bmsdata.discharge_max as f32 * 0.1;
    inverter_data.voltage = bmsdata.pack_volts as f32 * 0.1;
    inverter_data.current = bmsdata.current as f32 * 0.1; // 100mA = 1
    inverter_data.capacity = bmsdata.soc as u16;
    inverter_data.kwh = bmsdata.kwh_remaining as f32 * 0.1;
    inverter_data.int_temp = bmsdata.temp_avg as f32 * 0.1;
    inverter_data.wh_total = 10000;
    inverter_data.contactor = true;
    inverter_data.v_max = bmsdata.max_volts as f32; // cell as millivolts
    inverter_data.v_min = bmsdata.min_volts as f32;
    inverter_data.valid = bmsdata.valid;
    // solax_data.timestamp = Some(Instant::now());
    // MQTTFMT.lock().await.update(bmsdata);  <- bat type NEED TO STANDARDISE THE BMS STRUCT!
}

#[cfg(feature = "byd")]
#[inline]
async fn push_bms_to_inverter(bmsdata: renault_zoe_ph2_battery::bms::Bms) {
    // BYD struct not yet implemented

    // let mut inverter_data = INVERTER_DATA.lock().await;
    // // convert everything to standard units, volts, millivolts, etc
    // inverter_data.pack_voltage_max = bmsdata.pack_voltage_max as f32 * 0.1;
    // inverter_data.charge_voltage_max = bmsdata.slave_voltage_max as f32 * 0.1;
    // inverter_data.charge_voltage_min = bmsdata.slave_voltage_min as f32 * 0.1;
    // inverter_data.charge_max = bmsdata.charge_max as f32 * 0.1;
    // inverter_data.discharge_max = bmsdata.discharge_max as f32 * 0.1;
    // inverter_data.voltage = bmsdata.pack_volts as f32 * 0.1;
    // inverter_data.current = bmsdata.current as f32 * 0.1; // 100mA = 1
    // inverter_data.capacity = bmsdata.soc as u16;
    // inverter_data.kwh = bmsdata.kwh_remaining as f32 * 0.1;
    // inverter_data.int_temp = bmsdata.temp_avg as f32 * 0.1;
    // inverter_data.wh_total = 10000;
    // inverter_data.contactor = true;
    // inverter_data.v_max = bmsdata.max_volts as f32; // cell as millivolts
    // inverter_data.v_min = bmsdata.min_volts as f32;
    // inverter_data.valid = bmsdata.valid;
    // solax_data.timestamp = Some(Instant::now());
    // MQTTFMT.lock().await.update(bmsdata);
    // let config = CONFIG.lock().await;
    // bmsdata.set_dod(config.dod.min(), config.dod.max());
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
