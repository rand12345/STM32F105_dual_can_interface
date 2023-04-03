use crate::statics::*;
use bms_standard::Bms;
use defmt::{error, warn};
use embassy_stm32::can::bxcan::Frame;

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
    use bms_standard::MinMax;
    use embassy_stm32::can::bxcan::Id;
    use embassy_stm32::can::bxcan::Id::Standard;
    use embassy_time::Instant;

    let rx = BMS_CHANNEL_RX.receiver();
    let tx = BMS_CHANNEL_TX.sender();
    let mut data = kangoo_battery::Data::new();
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
        if ![0x155, 0x424, 0x425, 0x4ae, 0x7bb].contains(&id) {
            continue; // filter unwanted frames
        }
        if id != 1979 {
            let update_inverter = match data.rapid_data_processor(frame) {
                Ok(state) => state, // data can be parsed without diag data is true - use a different validity checker
                Err(e) => {
                    warn!("Rapid data parsing error: {}", e); //: {:?}", Debug2Format(&e));
                    continue;
                }
            };
            if update_inverter {
                {
                    // change to BMS wdt and signal update
                    *LAST_BMS_MESSAGE.lock().await = Some(Instant::now());
                }
                let c = async || {
                    // remove async, not bubbling errors
                    let mut bmsdata = BMS.lock().await;
                    bmsdata.cell_mv = data.cells_mv;
                    bmsdata.cell_range_mv = data.cell_mv;
                    bmsdata.pack_volts = data.pack_volts;
                    bmsdata.kwh_remaining = data.kwh_remaining;
                    bmsdata.soc = data.soc_value as f32;
                    bmsdata.temp = data.pack_temp;
                    bmsdata.temps =
                        MinMax::new(*data.temp.minimum() as f32, *data.temp.maximum() as f32);
                    bmsdata
                        .set_valid(true)?
                        .update_current(data.current_value)?
                        .update_soc(data.soc_value)?
                        .update_kwh(data.kwh_remaining)?
                        .update_max_charge_amps(data.max_charge_amps)?
                        .update_pack_volts(data.pack_volts)?
                        .set_pack_temp(data.pack_temp)?
                        .throttle_pack()
                };
                match c().await {
                    Ok(bms) => {
                        push_all_to_mqtt(bms).await;
                        update_dod(bms).await;
                    }
                    Err(e) => error!("Rapid data update error: {}", e),
                };
            }
        } else {
            match data.diag_data_processor(frame) {
                Ok(None) => {
                    WDT.signal(true); // temp whilst testing
                    {
                        *LAST_BMS_MESSAGE.lock().await = Some(Instant::now());
                    }
                    let c = async || {
                        let mut bmsdata = BMS.lock().await;
                        bmsdata.bal_cells = data.bal_cells;
                        bmsdata
                            .set_valid(true)?
                            .set_cell_mv_low_high(*data.cell_mv.minimum(), *data.cell_mv.maximum())?
                            .update_pack_volts(data.pack_volts)?
                            .set_temps(*data.temp.minimum(), *data.temp.maximum())?
                            .throttle_pack()
                    };
                    match c().await {
                        Ok(bms) => {
                            push_all_to_mqtt(bms).await;
                            update_dod(bms).await;
                        }
                        Err(e) => error!("Diag update error: {}", e),
                    };
                }

                Ok(Some(next_tx_frame)) => {
                    tx.send(next_tx_frame).await;
                    continue;
                }
                Err(e) => {
                    error!("BMS diag error: {:?}", e);
                }
            };
        }
    }
}

#[inline]
async fn push_all_to_mqtt(bms: Bms) {
    MQTTFMT.lock().await.update(bms);
}

#[inline]
async fn update_dod(bms: Bms) {
    let config = CONFIG.lock().await;
    let mut bmsmut = bms;
    if let Err(e) = bmsmut.set_dod(config.dod.min(), config.dod.max()) {
        error!("{}", e)
    }
}
