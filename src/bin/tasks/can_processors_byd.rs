use crate::statics::*;
use defmt::error;
use defmt::info;
use defmt::warn;
use defmt::Debug2Format;
use embassy_stm32::can::bxcan::Frame;
use embedded_hal::can::Id;
use embedded_hal::can::StandardId;

// hacky
const SEND_EVERY_SECS: u64 = 1;
const CHARGER_VOLTS_HIGH_VAL: u16 = 6500;
const CHARGER_VOLTS_LOW_VAL: u16 = 4800;

// #[allow(unused_assignments)]
#[cfg(feature = "byd")]
#[embassy_executor::task]
pub async fn inverter_rx() -> ! {
    use embassy_time::{Duration, Timer};
    warn!("Starting Inverter Processor");
    let mut inverter_comms_valid = false;

    let recv = INVERTER_CHANNEL_RX.receiver();
    let trans = INVERTER_CHANNEL_TX.sender();
    loop {
        if let Ok(frame) = recv.try_recv() {
            warn!("Debug: Inv >> Gateway {}", Debug2Format(&frame))
        };
        Timer::after(Duration::from_secs(SEND_EVERY_SECS)).await;
        inverter_comms_valid = false;
        if LAST_BMS_MESSAGE.lock().await.elapsed().as_secs() > LAST_READING_TIMEOUT_SECS {
            error!("BMS last update timeout, inverter communications stopped");
            CONTACTOR_STATE.signal(inverter_comms_valid);
            continue;
        };
        inverter_comms_valid = true;
        let bmsdata = { *BMS.lock().await };

        let charge_volts_high = CHARGER_VOLTS_HIGH_VAL.to_le_bytes();

        let charge_volts_low = CHARGER_VOLTS_LOW_VAL.to_le_bytes();
        let charge_current = bmsdata.charge_max.to_le_bytes();
        let discharge_current = bmsdata.discharge_max.to_le_bytes();
        let soc = bmsdata.kwh_remaining.to_le_bytes();
        #[cfg(feature = "v65")]
        let pack_volts = (bmsdata.pack_volts / 6).to_le_bytes();
        #[cfg(not(feature = "v65"))]
        let pack_volts = bmsdata.pack_volts.to_le_bytes();
        let current = bmsdata.current.to_le_bytes();
        let temp = bmsdata.temp_avg.to_le_bytes();

        let cap = (52000u16 / 65).to_le_bytes(); // capacity vs voltage
        let temp_high = bmsdata.temp_max.to_le_bytes();
        let temp_low = bmsdata.temp_min.to_le_bytes();

        let frames: [Frame; 10] = [
            frame_builder(0x618, &[0x0, b'B', b'Y', b'D', 0x0, 0x0, 0x0, 0x0]),
            frame_builder(0x5d8, &[0x0, b'B', b'Y', b'D', 0x0, 0x0, 0x0, 0x0]),
            frame_builder(0x558, &[0x3, 0x13, 0x0, 0x4, cap[1], cap[0], 0x5, 0x7]),
            frame_builder(0x598, &[0x0, 0x0, 0x12, 0x34, 0x0, 0x0, 0x4, 0x4f]),
            frame_builder(
                0x358,
                &[
                    charge_volts_high[1],
                    charge_volts_high[0],
                    charge_volts_low[1],
                    charge_volts_low[0],
                    discharge_current[1],
                    discharge_current[0],
                    charge_current[1],
                    charge_current[0],
                ],
            ),
            frame_builder(
                0x3d8,
                &[soc[1], soc[0], soc[1], soc[0], 0x0, 0x0, 0xf9, 0x0],
            ), // 4 & 5 are ampsecond * 0.00277 ??
            frame_builder(0x458, &[0x0, 0x0, 0x12, 0x34, 0x0, 0x0, 0x56, 0x78]),
            frame_builder(
                0x518,
                &[
                    temp_high[1],
                    temp_high[0],
                    temp_low[1],
                    temp_low[0],
                    0xff,
                    0xff,
                    0xff,
                    0xff,
                ],
            ),
            frame_builder(
                0x4d8,
                &[
                    pack_volts[1],
                    pack_volts[0],
                    current[1],
                    current[0],
                    temp[1],
                    temp[0],
                    0x3,
                    0x8,
                ],
            ),
            frame_builder(0x158, &[0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0]),
        ];

        // drops mutex
        for frame in frames.into_iter() {
            info!("Send frame {:?}", frame.data());
            trans.send(frame).await;
        }
        CONTACTOR_STATE.signal(inverter_comms_valid);
    }
}

fn frame_builder<T: embedded_hal::can::Frame + core::clone::Clone>(id: u16, framedata: &[u8]) -> T {
    T::new(Id::Standard(StandardId::new(id).unwrap()), framedata).unwrap()
}
