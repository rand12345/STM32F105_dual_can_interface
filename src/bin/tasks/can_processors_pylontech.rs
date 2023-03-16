use crate::statics::*;
use defmt::error;
use defmt::info;
use defmt::warn;
use defmt::Debug2Format;
use embassy_stm32::can::bxcan::Frame;

#[allow(unused_assignments)]
#[cfg(feature = "pylontech")]
#[embassy_executor::task]
pub async fn inverter_rx() -> ! {
    use embassy_time::{Duration, Timer};
    warn!("Starting Inverter Processor");
    let mut inverter_comms_valid = false;

    let recv = INVERTER_CHANNEL_RX.receiver();
    let trans = INVERTER_CHANNEL_TX.sender();
    loop {
        if let Ok(frame) = recv.try_recv() {
            warn!("Debug: Inv >> Gateway {}", Debug2Format(&frame))
        };
        Timer::after(Duration::from_secs(1)).await;
        inverter_comms_valid = false;
        if LAST_BMS_MESSAGE.lock().await.elapsed().as_secs() > LAST_READING_TIMEOUT_SECS {
            error!("BMS last update timeout, inverter communications stopped");
            CONTACTOR_STATE.signal(inverter_comms_valid);
            continue;
        };
        let frames: pylontech_protocol::Vec<Frame, 7> = {
            let mut inverter = INVERTER_DATA.lock().await;
            match inverter.parser() {
                Ok(f) => f,
                Err(e) => {
                    match e {
                        pylontech_protocol::messages::CanError::UnknownMessageId(e) => {
                            error!("Unknown message Id {}", e)
                        }
                        pylontech_protocol::messages::CanError::ParameterOutOfRange {
                            message_id,
                        } => {
                            error!("Parameter out of range  {}", message_id)
                        }
                        pylontech_protocol::messages::CanError::InvalidPayloadSize => {
                            error!("Invalid payload size")
                        }
                        pylontech_protocol::messages::CanError::InvalidMultiplexor {
                            message_id,
                            multiplexor,
                        } => error!("Invalid multiplexor {} {}", message_id, multiplexor),
                    };
                    continue;
                }
            }
        };
        // drops mutex
        for frame in frames.into_iter() {
            info!("Send frame {:?}", frame.data());
            trans.send(frame).await;
        }
        CONTACTOR_STATE.signal(inverter_comms_valid);
    }
}
