use embassy_stm32::can::bxcan::Frame;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;

use crate::async_tasks::CommsState;

pub type InverterChannelRx = Channel<CriticalSectionRawMutex, Frame, 2>;
pub type InverterChannelTx = Channel<CriticalSectionRawMutex, Frame, 20>;
pub type BmsChannelRx = Channel<CriticalSectionRawMutex, Frame, 20>;
pub type BmsChannelTx = Channel<CriticalSectionRawMutex, Frame, 20>;

pub type Status = Signal<CriticalSectionRawMutex, bool>;
pub type State = Signal<CriticalSectionRawMutex, CommsState>;
