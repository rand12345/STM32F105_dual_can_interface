use crate::statics::*;
use defmt::warn;
use embassy_stm32::can::{
    bxcan::{Id::*, *},
    Can,
};
use embassy_stm32::peripherals::*;
use embassy_time::{Duration, Timer};
use nb::Error::*;

// const Ids: [u16; 3] = [0x1, 0x2, 0x3];

#[embassy_executor::task]
pub async fn can2_task(mut can: Can<'static, CAN2>) {
    let rx = EV_CAN_RX.sender();
    let tx = EV_CAN_TX.receiver();
    // use embassy_stm32::can::bxcan::Id::*;
    // Wait for Can1 to initalise
    CAN_READY.wait().await;
    let _canid = |frame: &Frame| -> u16 {
        match frame.id() {
            Standard(id) => id.as_raw(),
            Id::Extended(_) => 0,
        }
    };

    can.modify_config()
        .set_bit_timing(BITTIMINGS) // http://www.bittiming.can-wiki.info/
        .set_loopback(false) // Receive own frames
        .set_silent(false)
        // .set_automatic_retransmit(false)
        .enable();
    warn!("Starting Inverter Can2");

    loop {
        // yield_now().await;
        Timer::after(Duration::from_millis(1)).await;
        if let Ok(frame) = can.receive() {
            // if [0x1db, 0x5bc, 0x5b9].contains(&canid(&frame)) {
            rx.send(frame).await
            // }
        };
        let Ok(frame) = tx.try_recv() else { continue };
        match can.transmit(&frame) {
            Ok(_) => {
                // defmt::info!("CAN2 Tx: {}", Debug2Format(&(frame.id(), frame.data())));
                while !can.is_transmitter_idle() {
                    Timer::after(Duration::from_millis(1)).await;
                }
            }
            Err(WouldBlock) => (),
            Err(Other(_)) => defmt::error!("Inv Tx error"),
        }
    }
}
#[embassy_executor::task]
pub async fn can1_task(mut can: Can<'static, CAN1>) {
    // BMS Filter ============================================
    // can.modify_filters()
    //     .set_split(1)
    //     .enable_bank(0, Fifo::Fifo0, filter::Mask32::accept_all());

    // Inverter Filter ============================================
    can.modify_filters()
        .set_split(2)
        .enable_bank(
            0,
            Fifo::Fifo0,
            filter::Mask32::frames_with_std_id(
                StandardId::new(0x7bb).unwrap(),
                StandardId::new(0x7ff).unwrap(),
            ),
        )
        .enable_bank(
            1,
            Fifo::Fifo0,
            filter::Mask32::frames_with_std_id(
                StandardId::new(0x1db).unwrap(),
                StandardId::new(0x7ff).unwrap(),
            ),
        );

    can.modify_filters()
        .slave_filters()
        .enable_bank(
            2,
            Fifo::Fifo0,
            filter::Mask32::frames_with_std_id(
                StandardId::new(0x7bb).unwrap(),
                StandardId::new(0x7ff).unwrap(),
            ),
        )
        .enable_bank(
            3,
            Fifo::Fifo0,
            filter::Mask32::frames_with_std_id(
                StandardId::new(0x1db).unwrap(),
                StandardId::new(0x7ff).unwrap(),
            ),
        );

    can.modify_config()
        .set_bit_timing(BITTIMINGS) // http://www.bittiming.can-wiki.info/
        .set_loopback(false) // Receive own frames
        .set_silent(false)
        .enable();
    warn!("Starting Can1");

    // Signal to other can bus that filters have been applied
    CAN_READY.signal(true);

    let rx = CAR_CAN_RX.sender();
    let tx = CAR_CAN_TX.receiver();
    let _canid = |frame: &Frame| -> u16 {
        match frame.id() {
            Standard(id) => id.as_raw(),
            Id::Extended(_) => 0,
        }
    };

    loop {
        // WDT.signal(true); // temp whilst testing

        Timer::after(Duration::from_millis(1)).await;
        if let Ok(frame) = can.receive() {
            // if let embassy_stm32::can::bxcan::Id::Extended(id) = frame.id() {
            // if id.as_raw() == 0x18DAF1DB {
            // defmt::info!("EVCAN>>STM {:?}", Debug2Format(&(frame.id(), frame.data())));
            // if [0x7bb, 0x5bc, 0x5b9].contains(&canid(&frame)) {
            rx.send(frame).await
            // }
            // };
            // }
        };
        let Ok(frame) = tx.try_recv() else { continue };
        match can.transmit(&frame) {
            Ok(_) => {
                // defmt::info!("CAN1>>BMS: {}", Debug2Format(&(frame.id(), frame.data())));

                while !can.is_transmitter_idle() {
                    Timer::after(Duration::from_millis(1)).await;
                }
            }
            Err(WouldBlock) => (),
            Err(Other(_)) => defmt::error!("BMS Tx error"),
        }
    }
}
