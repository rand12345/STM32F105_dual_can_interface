use crate::statics::*;
use defmt::{warn, Debug2Format};
use embassy_futures::yield_now;
use embassy_stm32::can::{bxcan::*, Can};
use embassy_stm32::peripherals::*;
use nb::Error::*;

#[embassy_executor::task]
pub async fn inverter_task(mut can: Can<'static, CAN2>) {
    let rx = INVERTER_CHANNEL_RX.sender();
    let tx = INVERTER_CHANNEL_TX.receiver();
    // use embassy_stm32::can::bxcan::Id::*;
    // Wait for Can1 to initalise
    CAN_READY.wait().await;

    can.modify_config()
        .set_bit_timing(BITTIMINGS) // http://www.bittiming.can-wiki.info/
        .set_loopback(false) // Receive own frames
        .set_silent(false)
        // .set_automatic_retransmit(false)
        .enable();
    warn!("Starting Inverter Can2");

    // #[cfg(feature = "solax")]
    // let canid = |frame: &Frame| -> u32 {
    //     if let Extended(id) = frame.id() {
    //         id.as_raw()
    //     } else {
    //         0
    //     }
    // };

    // #[cfg(feature = "pylontech")]
    // let canid = |frame: &Frame| -> u16 {
    //     if let Standard(id) = frame.id() {
    //         id.as_raw()
    //     } else {
    //         0
    //     }
    // };

    loop {
        yield_now().await;
        if let Ok(frame) = can.receive() {
            // #[cfg(feature = "solax")]
            // if canid(&frame) == 0x1871 {
            //     rx.send(frame).await
            // };

            // #[cfg(feature = "pylontech")]
            rx.send(frame).await
        };
        let Ok(frame) = tx.try_recv() else { continue };
        match can.transmit(&frame) {
            Ok(_) => {
                defmt::info!("Inv Tx: {}", Debug2Format(&(frame.id(), frame.data())));
                while !can.is_transmitter_idle() {
                    yield_now().await
                }
            }
            Err(WouldBlock) => (),
            Err(Other(_)) => defmt::error!("Inv Tx error"),
        }
    }
}
#[embassy_executor::task]
pub async fn bms_task(mut can: Can<'static, CAN1>) {
    #[cfg(feature = "ze50")]
    use embassy_stm32::can::bxcan::ExtendedId;

    // BMS Filter ============================================
    #[cfg(feature = "ze50")]
    can.modify_filters().set_split(1).enable_bank(
        0,
        Fifo::Fifo0,
        filter::Mask32::frames_with_ext_id(
            ExtendedId::new(0x18DAF1DB).unwrap(),
            ExtendedId::new(0x1ffffff).unwrap(),
        ),
    );

    #[cfg(not(feature = "ze50"))]
    can.modify_filters()
        .set_split(1)
        .enable_bank(0, Fifo::Fifo0, filter::Mask32::accept_all());

    // Inverter Filter ============================================
    can.modify_filters()
        .slave_filters()
        .enable_bank(1, Fifo::Fifo0, filter::Mask32::accept_all());

    can.modify_config()
        .set_bit_timing(BITTIMINGS) // http://www.bittiming.can-wiki.info/
        .set_loopback(false) // Receive own frames
        .set_silent(false)
        .enable();
    warn!("Starting BMS Can1");

    // Signal to other can bus that filters have been applied
    CAN_READY.signal(true);

    let rx = BMS_CHANNEL_RX.sender();
    let tx = BMS_CHANNEL_TX.receiver();
    // let canid = |frame: &Frame| -> u16 {
    //     match frame.id() {
    //         Standard(id) => id.as_raw(),
    //         Id::Extended(_) => 0,
    //     }
    // };

    loop {
        // WDT.signal(true); // temp whilst testing
        yield_now().await;
        if let Ok(frame) = can.receive() {
            // defmt::println!("BMS: Rx {:?}", frame);
            // if let embassy_stm32::can::bxcan::Id::Extended(id) = frame.id() {
            // if id.as_raw() == 0x18DAF1DB {
            defmt::info!("BMS>>STM {:?}", Debug2Format(&(frame.id(), frame.data())));
            rx.send(frame).await;
            // };
            // }
        };
        let Ok(frame) = tx.try_recv() else { continue };
        match can.transmit(&frame) {
            Ok(_) => {
                defmt::info!("STM>>BMS: {}", Debug2Format(&(frame.id(), frame.data())));

                while !can.is_transmitter_idle() {
                    yield_now().await
                }
            }
            Err(WouldBlock) => (),
            Err(Other(_)) => defmt::error!("BMS Tx error"),
        }
    }
}
