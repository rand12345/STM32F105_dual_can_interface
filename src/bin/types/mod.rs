use crate::tasks::mqtt::MqttFormat;
use embassy_stm32::can::bxcan::Frame;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::Instant;

pub type InverterChannelRx = Channel<CriticalSectionRawMutex, Frame, 2>;
pub type InverterChannelTx = Channel<CriticalSectionRawMutex, Frame, 20>;
pub type BmsChannelRx = Channel<CriticalSectionRawMutex, Frame, 20>;
pub type BmsChannelTx = Channel<CriticalSectionRawMutex, Frame, 20>;
pub type Elapsed = Mutex<CriticalSectionRawMutex, Instant>;
pub type Status = Signal<CriticalSectionRawMutex, bool>;
pub type InverterDataMutex =
    embassy_sync::mutex::Mutex<CriticalSectionRawMutex, solax_can_bus::SolaxBms>;
pub type MqttFmtMutex = embassy_sync::mutex::Mutex<CriticalSectionRawMutex, MqttFormat>;
