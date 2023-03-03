use crate::BITTIMINGS;
use defmt::{info, warn, Debug2Format};
use embassy_futures::yield_now;
use embassy_stm32::can::{bxcan::*, Can};
use embassy_stm32::peripherals::*;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use lazy_static::lazy_static;
use nb::Error::*;

pub type InverterChannelRx = Channel<CriticalSectionRawMutex, Frame, 2>;
pub type InverterChannelTx = Channel<CriticalSectionRawMutex, Frame, 20>;
pub type BmsChannelRx = Channel<CriticalSectionRawMutex, Frame, 10>;
pub type BmsChannelTx = Channel<CriticalSectionRawMutex, Frame, 2>;

pub type Status = Signal<CriticalSectionRawMutex, bool>;
pub type State = Signal<CriticalSectionRawMutex, CommsState>;

lazy_static! {
    pub static ref INVERTER_CHANNEL_RX: InverterChannelRx = Channel::new();
    pub static ref INVERTER_CHANNEL_TX: InverterChannelTx = Channel::new();
    pub static ref BMS_CHANNEL_RX: BmsChannelRx = Channel::new();
    pub static ref BMS_CHANNEL_TX: BmsChannelTx = Channel::new();
    pub static ref CAN_READY: Status = Signal::new();
    pub static ref CONTACTOR_STATE: Status = Signal::new();
    pub static ref STATE: State = Signal::new();
}
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum CommsState {
    CAN1RX,
    CAN1TX,
    CAN2RX,
    CAN2TX,
    Update,
}
#[embassy_executor::task]
pub async fn inverter_rx_processor() {
    //mut contactor: ContactorType<'_>
    // let timeout = std::time::Duration::from_secs(30);
    warn!("Starting Inverter Processor");
    // let mut communications_valid = Safe::No;
    // let mut contactor_state = ContactorState::default();

    let recv = INVERTER_CHANNEL_RX.receiver();
    let trans = INVERTER_CHANNEL_TX.sender();

    loop {
        // This is a server
        let frame = recv.recv().await;
        info!(
            "Inv proc Rx: {:?}",
            Debug2Format(&(frame.id(), frame.data()))
        );
        let sendframe = Frame::new_data(StandardId::new(4).unwrap(), [4, 4, 4, 4, 4, 4, 4, 4]);
        trans.send(sendframe).await;
    }
}

#[embassy_executor::task]
pub async fn bms_rx_processor() {
    //mut contactor: ContactorType<'_>
    // let timeout = std::time::Duration::from_secs(30);
    warn!("Starting BMS Processor");
    // let mut communications_valid = Safe::No;
    // let mut contactor_state = ContactorState::default();
    use kangoo_battery::{Bms, Data};
    let rx = BMS_CHANNEL_RX.receiver();
    let tx = BMS_CHANNEL_TX.sender();
    let mut bms_validated = Bms::new();
    let mut data = Data::default();
    let mut _update_inverter = false;
    warn!("Starting BMS Rx Processor");

    loop {
        let frame = rx.recv().await;

        // Process 10ms data
        let id = if let Id::Standard(i) = frame.id() {
            i.as_raw()
        } else {
            continue;
        };
        if ![0x155, 0x424, 0x425, 0x4ae, 0x7bb].contains(&id) {
            warn!("Bad BMS ID {}", id);
            continue;
        }

        if id != 0x7bb {
            _update_inverter = match data.rapid_data_processor(frame) {
                Ok(state) => state, // data can be parsed without diag data is true - use a different validity checker
                Err(e) => {
                    warn!("Rapid data parsing error: {:?}", Debug2Format(&e));
                    continue;
                }
            }
            //else process diag data
        } else {
            _update_inverter = match data.diag_data_processor(frame) {
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
        if _update_inverter {
            // push_bms_to_inverter(bms_validated).await;
            info!("Pushed values to Inverter data store");
            // push_all_to_mqtt(bms_validated).await;
            info!("Push values to MQTT data store");
            STATE.signal(CommsState::Update);
            _update_inverter = false;
        }
    }
}

#[embassy_executor::task]
pub async fn inverter_task(mut can: Can<'static, CAN2>) {
    // Wait for Can1 to initalise
    CAN_READY.wait().await;

    can.modify_config()
        .set_bit_timing(BITTIMINGS) // http://www.bittiming.can-wiki.info/
        .set_loopback(false) // Receive own frames
        .set_silent(false)
        // .set_automatic_retransmit(false)
        .enable();
    warn!("Starting Inverter Can2");

    let rx = INVERTER_CHANNEL_RX.sender();
    let tx = INVERTER_CHANNEL_TX.receiver();
    loop {
        yield_now().await;
        if let Ok(frame) = can.receive() {
            defmt::println!("Inv: Rx {:?}", frame);
            rx.send(frame).await
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
    loop {
        yield_now().await;
        if let Ok(frame) = can.receive() {
            // defmt::println!("BMS: Rx {:?}", frame);
            rx.send(frame).await
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
