use crate::errors::StmError;
use miniserde::__private::String;
use miniserde::{json, Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)] // references only
pub struct Config {
    pack_volts: MinMax<u16>,
    cell_millivolts: MinMax<u16>,
    pack_temperature: MinMax<i16>,
    cell_temperature: MinMax<i16>,
    current_amps: MinMax<i16>,
    pub dod: MinMax<u8>,
    timeout_secs: u8,
    mqtt_rate_secs: u32,
    state: State,
}

impl Config {
    pub fn update_from_json(&mut self, slice: &[u8]) -> Result<(), StmError> {
        *self = json::from_str::<Self>(
            core::str::from_utf8(slice).map_err(|_e| StmError::InvalidConfigData)?,
        )
        .map_err(|_e| StmError::InvalidConfigData)?;
        Ok(())
    }
    pub fn dump_to_json(&self) -> String {
        json::to_string(&self)
    }
    pub fn state(&self) -> &State {
        &self.state
    }

    pub fn set_state(&mut self, state: State) {
        self.state = state;
    }

    pub fn pack_volts(&self) -> &MinMax<u16> {
        &self.pack_volts
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            pack_volts: MinMax::int(300, 400),
            cell_millivolts: MinMax::int(3000, 4200),
            pack_temperature: MinMax::signed(-20, 50),
            cell_temperature: MinMax::signed(-20, 50),
            current_amps: MinMax::signed(-50, 50),
            dod: MinMax::int(0, 99),
            timeout_secs: 60,
            mqtt_rate_secs: 10,
            state: State::Offline,
        }
    }
}

/*

{"pack_volts":{"min":300,"max":400},"cell_millivolts":{"min":3000,"max":4200},"pack_temperature":{"min":-20,"max":50},"cell_temperature":{"min":-20,"max":50},"current_amps":{"min":-50,"max":50},"dod":{"min":0,"max":99},"timeout_secs":60,"mqtt_rate_secs":10,"state":"Offline"}
{"pack_volts":{"min":300,"max":400}}
*/

#[derive(Serialize, Deserialize, Debug)]
pub struct MinMax<T> {
    min: T,
    max: T,
}

impl<T: Copy> MinMax<T> {
    fn int(min: T, max: T) -> Self {
        Self {
            min: min as T,
            max: max as T,
        }
    }
    fn signed(min: T, max: T) -> Self {
        Self {
            min: min as T,
            max: max as T,
        }
    }

    pub fn min(&self) -> T {
        self.min
    }

    pub fn max(&self) -> T {
        self.max
    }
}

#[derive(Serialize, Deserialize, Debug, Default)]
pub enum State {
    Online,
    InvFault,
    BmsFault,
    #[default]
    Offline,
}
