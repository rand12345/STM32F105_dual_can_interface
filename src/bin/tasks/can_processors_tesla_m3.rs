use crate::statics::*;
use bms_standard::Bms;
use defmt::{error, info, warn};
use embassy_time::Instant;

/*

   Todo: Standardise Bms struct, include setters for cell volts, temps etc.

*/
#[cfg(feature = "tesla_m3")]
#[embassy_executor::task]
pub async fn bms_tx_periodic() {
    use embassy_time::{Duration, Ticker};
    let tx = BMS_CHANNEL_TX.sender();
    let ticker_ms = |ms| Ticker::every(Duration::from_millis(ms));
    let sender = |frame| {
        if let Err(_e) = tx.try_send(frame) {
            error!("Periodic queue buf error")
        };
    };
    // Wait until can is up
    ticker_ms(2000).next().await;
    #[allow(dead_code)]
    enum ContactorState {
        Close,
        Precharge,
        Open,
        Unimplemented,
    }
    let contactor_command = ContactorState::Unimplemented;
    let mut t1 = ticker_ms(30);
    // let mut t2 = ticker_ms(225);
    loop {
        t1.next().await;

        match contactor_command {
            ContactorState::Close => {
                sender(frame_builder(
                    0x221,
                    &[0x40, 0x41, 0x05, 0x15, 0x00, 0x50, 0x71, 0x7f],
                ));
                sender(frame_builder(
                    0x221,
                    &[0x60, 0x55, 0x55, 0x15, 0x54, 0x51, 0xd1, 0xb8],
                ));
            }
            ContactorState::Precharge => {
                sender(frame_builder(
                    0x221,
                    &[0x41, 0x11, 0x01, 0x00, 0x00, 0x00, 0x20, 0x96],
                ));
                sender(frame_builder(
                    0x221,
                    &[0x61, 0x15, 0x01, 0x00, 0x00, 0x00, 0x20, 0xba],
                ));
            }
            ContactorState::Open => sender(frame_builder(
                0x332,
                &[0x61, 0x15, 0x01, 0x55, 0x00, 0x00, 0xe0, 0x13],
            )),
            ContactorState::Unimplemented => continue,
        };
    }
}

#[cfg(feature = "tesla_m3")]
#[allow(unused_assignments)]
#[embassy_executor::task]
pub async fn bms_rx() {
    use bms_standard::BmsError;
    use defmt::info;
    use embassy_stm32::can::bxcan::{Frame, Id::Standard};
    let (mut _min, mut _max, _pid) = (u32::MAX, u32::MIN, 0u8);
    let rx = BMS_CHANNEL_RX.receiver();
    let mut data = BMS_DATA.lock().await;
    let mut rx_check = 0u8;
    loop {
        let frame: Frame = rx.recv().await;
        let id = if let Standard(id) = frame.id() {
            id.as_raw()
        } else {
            continue;
        };
        let payload = if let Some(payload) = frame.data() {
            payload
        } else {
            continue;
        };

        match id {
            0x132 => {
                rx_check |= 1; // set internal read status high
                data.pack_volts = u16::from_le_bytes([payload[0], payload[1]]);
                data.current_value = i16::from_le_bytes([payload[2], payload[3]]) as i32;
                info!("data.pack_volts: {}", data.pack_volts);
                info!("data.current_value: {}", data.current_value);
            }
            // min max volt,temp, seems there are no messages for individual cell temp
            0x332 => {
                rx_check |= 1 << 1; // set internal read status high
                let mux = payload[0] & 0x03;
                match mux {
                    0 => {
                        let _cell_volts_min = (u16::from_le_bytes([payload[0], payload[1]]) >> 2
                            & 0xFFF)
                            .saturating_div(500);
                        let _cell_volts_max = (u16::from_le_bytes([payload[2], payload[3]]) >> 2
                            & 0xFFF)
                            .saturating_div(500);
                        info!("_cell_volts_min: {}", _cell_volts_min);
                        info!("_cell_volts_max: {}", _cell_volts_max);
                    }
                    1 => {
                        data.temp_max = (payload[2] - 40) as i16 * 10;
                        info!("data.temp_max: {}", data.temp_max);
                    }
                    _ => {}
                }
            }
            0x401 => {
                rx_check |= 1 << 2; // set internal read status high
                let mux = payload[0] as usize;
                let mut cell = [0u16; 256];
                if payload[1] == 0x02a {
                    cell[1 + mux * 3] =
                        u16::from_le_bytes([payload[2], payload[3]]).saturating_div(10000);
                    info!("Cell{} : {}", (1 + mux) * 3, cell[1 + mux * 3]);
                    cell[2 + mux * 3] =
                        u16::from_le_bytes([payload[4], payload[5]]).saturating_div(10000);
                    info!("Cell{} : {}", (2 + mux) * 3, cell[1 + mux * 3]);
                    cell[3 + mux * 3] =
                        u16::from_le_bytes([payload[6], payload[7]]).saturating_div(10000);
                    info!("Cell{} : {}", (3 + mux) * 3, cell[1 + mux * 3]);
                }
            }
            0x352 => {
                if frame.dlc() != 8 {
                    continue;
                }
                rx_check |= 1 << 3; // set internal read status high
                                    // this could be a problem, if this le or be?
                let bits = u64::from_le_bytes(payload[..8].try_into().unwrap());
                let expected_energy_remaining = ((bits >> 22) as u16 & 0x7FF).saturating_div(10);
                let nominal_full_pack_energy = ((bits & 0x7FF) as u16).saturating_div(10);
                data.soc_value =
                    expected_energy_remaining.saturating_div(nominal_full_pack_energy) * 100;
                info!("data.soc_value: {}", data.soc_value);
            }
            0x20a => {
                if frame.dlc() != 8 {
                    continue;
                }
                rx_check |= 1 << 4; // set internal read status high
                let bits = u64::from_le_bytes(payload[..8].try_into().unwrap());
                let _contactor = bits & 0x0F;
                let _hvil_status = (bits >> 40) & 0x0F;
                let _pack_cont_negative_state = bits & 0x07;
                let _pack_cont_positive_state = (bits >> 3) & 0x07;
                let _pack_contactor_set_state = (bits >> 8) & 0x0F;
                let pack_ctrs_closing_allowed = (bits >> 35) & 0x01 == 1;
                let pyro_test_in_progress = (bits >> 37) & 0x01 == 1;
                info!("Contactor state: {}", _contactor);
                if !pack_ctrs_closing_allowed {
                    warn!("check High Voltage Connectors or Briges on this connectors")
                }
                if pyro_test_in_progress {
                    warn!("PyroTest in progress: WAIT!")
                }
                let contactor_operation = match _contactor {
                    1 => "OPEN",
                    2 => "CLOSING ",
                    3 => "BLOCKED ",
                    4 => "OPENING ",
                    5 => "CLOSED  ",
                    6 => "UNKNOWN6",
                    7 => "WELDED  ",
                    8 => "POS CL  ",
                    9 => "NEG CL  ",
                    _ => "Unknown",
                };
                let contactor_state = match _pack_cont_positive_state {
                    0 => "SNA",
                    1 => "OPEN      ",
                    2 => "PRECHARGE ",
                    3 => "BLOCKED   ",
                    4 => "PULLED_IN ",
                    5 => "OPENING   ",
                    6 => "ECONOMIZED",
                    7 => "WELDED    ",
                    _ => "Unknown",
                };
                let hvil_state = match _hvil_status {
                    1 => "STATUS_OK",
                    2 => "CURRENT_SOURCE_FAULT",
                    3 => "INTERNAL_OPEN_FAULT",
                    4 => "VEHICLE_OPEN_FAULT",
                    5 => "PENTHOUSE_LID_OPEN_FAULT",
                    6 => "UNKNOWN_LOCATION_OPEN_FAULT",
                    7 => "VEHICLE_NODE_FAULT",
                    8 => "NO_12V_SUPPLY",
                    9 => "VEHICLE_OR_PENTHOUSE_LID_OPENFAULT",
                    _ => "Unknown",
                };
                info!("contactor_operation: {}", contactor_operation);
                info!("contactor_state: {}", contactor_state);
                info!("hvil_state: {}", hvil_state);
            }
            0x2d2 => {
                if frame.dlc() != 8 {
                    continue;
                }

                rx_check |= 1 << 5; // set internal read status high
                let bits = u64::from_le_bytes(payload[..8].try_into().unwrap());
                let _min_pack_voltage = (bits & 0xffff) as u16 / 100 * 2;
                let _max_pack_voltage = ((bits >> 16) & 0xffff) as u16 / 100 * 2;
                let _max_charge_current = ((bits >> 32) & 0x3fff) as f32 * 0.128;
                let _max_discharge_current = ((bits >> 48) & 0x3fff) as f32 * 0.1;
                info!("_min_pack_voltage: {}", _min_pack_voltage);
                info!("_max_pack_voltage: {}", _max_pack_voltage);
                info!("_max_charge_current: {}", _max_charge_current);
                info!("_min_discharge_current: {}", _max_discharge_current);
            }
            0x2b4 => {
                if frame.dlc() != 8 {
                    continue;
                }

                rx_check |= 1 << 6; // set internal read status high
                let bits = u64::from_le_bytes(payload[..8].try_into().unwrap());
                let high_voltage = ((bits >> 10) & 0xFFF) as f32 * 0.146484;
                let low_voltage = (bits & 0x3FF) as f32 * 0.0390625;
                let output_current = (((bits >> 24) & 0xFFF) as i16).saturating_div(100);
                info!("high_voltage: {}", high_voltage);
                info!("low_voltage: {}", low_voltage);
                info!("output_current: {}", output_current);
            }
            0x3d2 => {
                if frame.dlc() != 8 {
                    continue;
                }

                rx_check |= 1 << 7; // set internal read status high
                let bits = u64::from_le_bytes(payload[..8].try_into().unwrap());
                let total_discharge = (bits & 0xFFFFFFFF) as u32;
                let total_charge = ((bits >> 32) & 0xFFFFFFFF) as u32;
                info!("total_discharge: {}", total_discharge);
                info!("total_charge: {}", total_charge);
            }
            _ => continue,
        }

        if rx_check == 0xF {
            let update = |mut bms: Bms| -> Result<(), BmsError> {
                bms.set_valid(false);
                bms.update_pack_volts(data.pack_volts)?;
                bms.update_current(data.current_value)?;
                bms.update_kwh(data.kwh_remaining);
                bms.update_soc(data.soc_value)?;
                bms.set_valid(true);

                Ok(())
            };
            *LAST_BMS_MESSAGE.lock().await = Instant::now();
            info!("BMS watchdog reset");
            let bms = BMS.lock().await;
            if let Err(_e) = update(*bms) {
                error!("BMS update failed");
            } else {
                info!("BMS updated")
                // push_bms_to_inverter(bms)
            };
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
/*
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
*/
#[derive(Debug)]
pub struct Data {
    pub cell_volts: [u16; 96],
    pub cell_bal: [bool; 96],
    pub soc_value: u16,
    current_offset: u16,
    pub current_value: i32, // 10 = 1A, 1=0.1A
    pub max_charge_amps: u16,
    pub kwh_remaining: u16,
    pub pack_temp: i16,
    pub pack_volts: u16, // 4000 = 400.0V
    // pub mode: BMSStatus,
    // pub req_mode: BattFunctions,
    pub req_code: u8,
    pub v_low_cell: u16,  //mV
    pub v_high_cell: u16, //mV
    pub soh: u16,
    pub hv_interlock: bool, // main external breaker
    pub temp_min: i16,
    pub temp_max: i16,
    pub temp_avg: i16,
}

impl Data {
    pub fn new() -> Self {
        Self {
            cell_volts: [0; 96],
            cell_bal: [false; 96],
            soc_value: 0,
            current_offset: 0,
            current_value: 0,
            max_charge_amps: 0,
            kwh_remaining: 0,
            pack_temp: 0,
            pack_volts: 0,
            // mode: BMSStatus::Offline,
            // req_mode: BattFunctions::Soc,
            req_code: 1,
            v_low_cell: 0,
            v_high_cell: 0,
            soh: 0,
            hv_interlock: false,
            temp_min: 0,
            temp_max: 0,
            temp_avg: 0,
        }
    }
}

fn frame_builder<T: embedded_hal::can::Frame + core::clone::Clone>(id: u16, framedata: &[u8]) -> T {
    use embedded_hal::can::{Id, StandardId};
    T::new(Id::Standard(StandardId::new(id).unwrap()), framedata).unwrap()
}
