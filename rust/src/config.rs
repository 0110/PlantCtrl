use std::fmt;

use serde::{Serialize, Deserialize};

use crate::PLANT_COUNT;


#[derive(Serialize, Deserialize)]
pub struct Config {
    tank_sensor_enabled: bool,
    tank_full_ml: u32,
    tank_warn_percent: u8,

    plantcount: u16,

    night_lamp_hour_start: u8,
    night_lamp_hour_end: u8,
    night_lamp_only_when_dark: u8,

    plants: [Plant;PLANT_COUNT]
}

#[derive(Serialize, Deserialize)]
pub struct Plant{
    target_moisture: u8,
    pump_time_s: u16,
    pump_cooldown_min: u16,
    pump_hour_start: heapless::String<5>,
    pump_hour_end: heapless::String<5>
}
#[derive(Serialize, Deserialize)]
#[derive(Debug)]
pub struct WifiConfig {
    pub ssid: heapless::String<32>,
    pub password: Option<heapless::String<64>>,
}

impl fmt::Display for WifiConfig {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "({}, ****)", self.ssid)
    }
}