use crate::types::*;
use embassy_sync::{channel::Channel, signal::Signal};
use lazy_static::lazy_static;

lazy_static! {
    pub static ref INVERTER_CHANNEL_RX: InverterChannelRx = Channel::new();
    pub static ref INVERTER_CHANNEL_TX: InverterChannelTx = Channel::new();
    pub static ref BMS_CHANNEL_RX: BmsChannelRx = Channel::new();
    pub static ref BMS_CHANNEL_TX: BmsChannelTx = Channel::new();
    pub static ref CAN_READY: Status = Signal::new();
    pub static ref WDT: Status = Signal::new();
    pub static ref CONTACTOR_STATE: Status = Signal::new();
    pub static ref STATE: State = Signal::new();
}
