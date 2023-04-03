use crate::statics::*;
use defmt::{error, info, warn};
use embassy_stm32::can::bxcan::Frame;

#[allow(unused_assignments)]
#[embassy_executor::task]
pub async fn inverter_rx() -> ! {
    use embassy_stm32::can::bxcan::Id::*;
    use solax_can_bus::SolaxError::*;
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
        let frame: Frame = recv.recv().await;

        if canid(&frame) != 0x1871 {
            continue;
        };
        inverter_comms_valid = false;
        {
            // need to init bms time starting with timeout
            if let Some(time) = *LAST_BMS_MESSAGE.lock().await {
                if time.elapsed().as_secs() > LAST_READING_TIMEOUT_SECS {
                    error!("BMS last update timeout, inverter communications stopped");
                    CONTACTOR_STATE.signal(inverter_comms_valid);
                    continue;
                };
            };
        }

        let response = {
            let mut solax = INVERTER_DATA.lock().await;
            let bms = BMS.lock().await;
            info!("BMS data: {:#}", *bms);
            solax.parser(frame, &bms, true) // add missing options
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
                    InvalidFrameEncode(id) => {
                        error!("Critical: frame encoding failed for {:02x}", id);
                        false // disable contactor
                    }
                    BadId(id) => {
                        error!("Critical: unexpected frame in inverter can data {:02x}", id);
                        false // disable contactor
                    }
                    x => {
                        warn!("{}", x);
                        true
                    }
                }
            }
        };

        CONTACTOR_STATE.signal(inverter_comms_valid);
    }
}
