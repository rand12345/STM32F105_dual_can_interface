use crate::config::Config;
use crate::tasks::mqtt::MqttFormat;
use embassy_stm32::can::bxcan::Frame;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex as _Mutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::Instant;

pub type InverterChannelRx = Channel<_Mutex, Frame, 2>;
pub type InverterChannelTx = Channel<_Mutex, Frame, 20>;
pub type BmsChannelRx = Channel<_Mutex, Frame, 20>;
pub type BmsChannelTx = Channel<_Mutex, Frame, 20>;
pub type Elapsed = Mutex<_Mutex, Option<Instant>>;
pub type MqttFmtMutex = embassy_sync::mutex::Mutex<_Mutex, MqttFormat>;
pub type ConfigType = embassy_sync::mutex::Mutex<_Mutex, Config>;
pub type Status = Signal<_Mutex, bool>;

#[cfg(feature = "solax")]
pub type InverterDataMutex = embassy_sync::mutex::Mutex<_Mutex, solax_can_bus::SolaxBms>;
#[cfg(feature = "pylontech")]
pub type InverterDataMutex = embassy_sync::mutex::Mutex<_Mutex, pylontech_protocol::PylontechBms>;
// #[cfg(feature = "byd")]
// pub type InverterDataMutex =
//     embassy_sync::mutex::Mutex<_Mutex, crate::tasks::can_processors_byd::Bms>;

#[cfg(feature = "ze50")]
pub type Ze50DataMutex = embassy_sync::mutex::Mutex<_Mutex, renault_zoe_ph2_battery::Data>;
// #[cfg(feature = "ze50")]
// pub type Ze50BmsMutex = embassy_sync::mutex::Mutex<_Mutex, renault_zoe_ph2_battery::bms::Bms>;
#[cfg(feature = "tesla_m3")]
pub type TeslaM3DataMutex =
    embassy_sync::mutex::Mutex<_Mutex, crate::tasks::can_processors_tesla_m3::Data>;

pub type BmsType = embassy_sync::mutex::Mutex<_Mutex, bms_standard::Bms>;
