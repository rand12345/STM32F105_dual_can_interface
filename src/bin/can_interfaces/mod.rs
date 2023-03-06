use crate::statics::*;
use crate::BITTIMINGS;
use defmt::{warn, Debug2Format};
use embassy_futures::yield_now;
use embassy_stm32::can::{bxcan::*, Can};
use embassy_stm32::peripherals::*;
use nb::Error::*;

#[embassy_executor::task]
pub async fn inverter_task(mut can: Can<'static, CAN2>) {
    let rx = INVERTER_CHANNEL_RX.sender();
    let tx = INVERTER_CHANNEL_TX.receiver();
    use embassy_stm32::can::bxcan::Id::*;
    // Wait for Can1 to initalise
    CAN_READY.wait().await;

    can.modify_config()
        .set_bit_timing(BITTIMINGS) // http://www.bittiming.can-wiki.info/
        .set_loopback(false) // Receive own frames
        .set_silent(false)
        // .set_automatic_retransmit(false)
        .enable();
    warn!("Starting Inverter Can2");
    let canid = |frame: &Frame| -> u32 {
        match frame.id() {
            Standard(_) => 0,
            Extended(id) => id.as_raw(),
        }
    };

    loop {
        yield_now().await;
        if let Ok(frame) = can.receive() {
            if canid(&frame) == 0x1871 {
                rx.send(frame).await
            };
        };
        if let Ok(frame) = tx.try_recv() {
            match can.transmit(&frame) {
                Ok(_) => {
                    defmt::info!("Can1 Tx: {:?}", frame);
                    while !can.is_transmitter_idle() {
                        yield_now().await
                    }
                }
                Err(WouldBlock) => (),
                Err(Other(e)) => defmt::error!("Inv Tx error {:?}", Debug2Format(&e)),
            }
        }
    }
}
#[embassy_executor::task]
pub async fn bms_task(mut can: Can<'static, CAN1>) {
    use embassy_stm32::can::bxcan::Id::Standard;
    can.modify_filters()
        .set_split(1)
        .enable_bank(0, Fifo::Fifo0, filter::Mask32::accept_all())
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
    let canid = |frame: &Frame| -> u16 {
        match frame.id() {
            Standard(id) => id.as_raw(),
            Id::Extended(_) => 0,
        }
    };

    loop {
        WDT.signal(true); // temp whilst testing
        yield_now().await;
        if let Ok(frame) = can.receive() {
            // defmt::println!("BMS: Rx {:?}", frame);
            if [0x155, 0x424, 0x425, 0x4ae, 0x7bb].contains(&canid(&frame)) {
                rx.send(frame).await;
            };
        };
        if let Ok(frame) = tx.try_recv() {
            match can.transmit(&frame) {
                Ok(_) => {
                    defmt::info!("BMS Tx: {}", Debug2Format(&(frame.id(), frame.data())));

                    while !can.is_transmitter_idle() {
                        yield_now().await
                    }
                }
                Err(WouldBlock) => (),
                Err(Other(e)) => defmt::error!("BMS Tx error {:?}", Debug2Format(&e)),
            }
        }
    }
}
