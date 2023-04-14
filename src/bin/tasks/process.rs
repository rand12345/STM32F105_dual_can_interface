use crate::statics::{BMS_CHANNEL_RX, BMS_CHANNEL_TX, INVERTER_CHANNEL_RX, INVERTER_CHANNEL_TX};
use embassy_futures::select::{select3, Either3::*};
use embassy_stm32::can::bxcan::{Frame, Id, StandardId};

#[embassy_executor::task]
pub async fn parse() {
    // let tx1 = INVERTER_CHANNEL_TX.sender();
    let tx2 = BMS_CHANNEL_TX.sender();
    let rx1 = INVERTER_CHANNEL_RX.receiver();
    let rx2 = BMS_CHANNEL_RX.receiver();
    defmt::warn!("Starting parser");
    let canid = |frame: &Frame| -> u16 {
        match frame.id() {
            embassy_stm32::can::bxcan::Id::Standard(id) => id.as_raw(),
            embassy_stm32::can::bxcan::Id::Extended(_) => 0,
        }
    };
    //02
    // 21 (= read block command?)
    // "01: ??? (6 lines)
    // 02: cellpair data (29 lines)
    // 03: Vmin, Max, ??? (5 lines)
    // 04: Temperature (3 lines)
    // 05: ??? (11 lines)
    // 06: balancing shunts"
    let init_frame = || {
        Frame::new_data(
            Id::Standard(StandardId::new(0x79b).unwrap()),
            [0x02, 0x21, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff],
        )
    };
    let continue_frame = || {
        Frame::new_data(
            Id::Standard(StandardId::new(0x79b).unwrap()),
            [0x30, 0x01, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff],
        )
    };

    let mut tick = embassy_time::Ticker::every(embassy_time::Duration::from_secs(5));
    let mut soc: f32 = 0.0;
    loop {
        let frame = match select3(rx1.recv(), rx2.recv(), tick.next()).await {
            First(frame) => frame,
            Second(frame) => frame,
            Third(_) => {
                // if let Err(_) = tx1.try_send(init_frame()) {
                //     defmt::error!("tx1 i error")
                // };
                if let Err(_) = tx2.try_send(init_frame()) {
                    defmt::error!("tx2 i error")
                };
                continue;
            }
        };
        let id = canid(&frame);
        if id != 0x7bb {
            continue;
        };

        let data = frame.data().unwrap();
        if matches!(data[0], 0x24) {
            if let Some(array) = data[4..8].try_into().ok() {
                soc = u32::from_be_bytes(array) as f32 / 10000.0;
                defmt::info!("SoC = {}", soc);
            };
        };

        // defmt::info!("ID {:x} {:x}", id, data);
        // if let Err(_) = tx1.try_send(continue_frame()) {
        //     defmt::error!("tx1 error")
        // };
        if let Err(_) = tx2.try_send(continue_frame()) {
            defmt::error!("tx2 error")
        };
    }
}
