use crate::statics::*;
use defmt::error;
use defmt::info;
use defmt::warn;
use defmt::Debug2Format;
use embassy_stm32::can::bxcan::Frame;

#[allow(unused_assignments)]
#[embassy_executor::task]
pub async fn inverter_rx() -> ! {
    use embassy_stm32::can::bxcan::Id::*;
    warn!("Starting Inverter Processor");
    let mut inverter_comms_valid = false;

    let recv = INVERTER_CHANNEL_RX.receiver();
    let trans = INVERTER_CHANNEL_TX.sender();

    let canid = |frame: &Frame| -> u32 {
        if let Extended(id) = frame.id() {
            id.as_raw()
        } else {
            0
        }
    };
    loop {
        let frame = recv.recv().await;
        inverter_comms_valid = false;
        if canid(&frame) == 0x1871 {
            rx.send(frame).await
        };

        if LAST_BMS_MESSAGE.lock().await.elapsed().as_secs() > LAST_READING_TIMEOUT_SECS {
            error!("BMS last update timeout, inverter communications stopped");
            CONTACTOR_STATE.signal(inverter_comms_valid);
            continue;
        };

        let response = {
            let mut solax = INVERTER_DATA.lock().await;
            solax.parser(frame)
        };

        inverter_comms_valid = match response {
            Ok(frames) => {
                info!("Sending {} frames to inverter", frames.len());
                for frame in frames {
                    trans.send(frame).await;
                }
                // Send signal to push json data to UART
                SEND_MQTT.signal(true);
                true
            }
            Err(e) => {
                match e {
                    solax_can_bus::SolaxError::InvalidData => {
                        error!("Invalid data");
                        warn!(
                            "Inverter data: {:?}",
                            Debug2Format(&*INVERTER_DATA.lock().await)
                        );
                        true
                    }
                    solax_can_bus::SolaxError::BadId(id) => {
                        error!("Critical: unexpected frame in inverter can data {:02x}", id);
                        false // disable contactor
                    }
                    solax_can_bus::SolaxError::InvalidFrameEncode(id) => {
                        error!("Critical: frame encoding failed for {:02x}", id);
                        false // disable contactor
                    }
                    solax_can_bus::SolaxError::TimeStamp(time) => {
                        info!("Inverter time: {}", time);
                        true
                    }
                    solax_can_bus::SolaxError::InvalidTimeData => {
                        warn!("InvalidTimeData");
                        true // not critical
                    }
                    solax_can_bus::SolaxError::UnwantedFrame => true,
                    // _ => true,
                }
            }
        };

        CONTACTOR_STATE.signal(inverter_comms_valid);
    }
}
