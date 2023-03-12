use crate::{config::Config, tasks::mqtt::MqttFormat, types::*};
use embassy_sync::{channel::Channel, mutex::Mutex, signal::Signal};
use embassy_time::Instant;
use lazy_static::lazy_static;
use solax_can_bus::SolaxBms;

lazy_static! {
    pub static ref INVERTER_CHANNEL_RX: InverterChannelRx = Channel::new();
    pub static ref INVERTER_CHANNEL_TX: InverterChannelTx = Channel::new();
    pub static ref BMS_CHANNEL_RX: BmsChannelRx = Channel::new();
    pub static ref BMS_CHANNEL_TX: BmsChannelTx = Channel::new();
    pub static ref CAN_READY: Status = Signal::new();
    pub static ref LAST_BMS_MESSAGE: Elapsed = Mutex::new(Instant::now());
    // pub static ref WDT: Status = Signal::new();
    pub static ref CONTACTOR_STATE: Status = Signal::new();
    pub static ref SEND_MQTT: Status = Signal::new();
    pub static ref MQTTFMT: MqttFmtMutex = embassy_sync::mutex::Mutex::new(MqttFormat::default());
    pub static ref INVERTER_DATA: InverterDataMutex =
        embassy_sync::mutex::Mutex::new(SolaxBms::default());
    pub static ref CONFIG: ConfigType = embassy_sync::mutex::Mutex::new(Config::default());
}
// pub const BITTIMINGS: u32 = 0x001c0000; // 500kps @ 8MHz // config.rcc.sys_ck = Some(mhz(64)); config.rcc.pclk1 = Some(mhz(24)); << experimental >>
pub const BITTIMINGS: u32 = 0x00050007; // 500kps @ 32Mhz // config.rcc.sys_ck = Some(mhz(64)); config.rcc.pclk1 = Some(mhz(24)); << experimental >>
                                        // pub const BITTIMINGS: u32 = 0x00050005; // 500kps @ 24Mhz
                                        // pub const BITTIMINGS: u32 = 0x00050008; // 500kps @ 36Mhz
pub const LAST_READING_TIMEOUT_SECS: u64 = 10;
// pub const MQTT_FREQUENCY_SECS: u64 = 10;
