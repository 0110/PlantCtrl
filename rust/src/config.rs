use std::fmt;

use serde::{Serialize, Deserialize};

use crate::PLANT_COUNT;


#[derive(Serialize, Deserialize)]
#[derive(Debug)]
pub struct Config {
    tank_sensor_enabled: bool,
    tank_full_ml: u32,
    tank_warn_percent: u8,

    night_lamp_hour_start: u8,
    night_lamp_hour_end: u8,
    night_lamp_only_when_dark: bool,

    plants: [Plant;PLANT_COUNT]
}

impl Default for Config {
    fn default() -> Self { 
        Self { tank_sensor_enabled: true, 
            tank_full_ml: 5000, 
            tank_warn_percent: 50, 
            night_lamp_hour_start: 21, 
            night_lamp_hour_end: 2, 
            night_lamp_only_when_dark: true, 
            plants: [Plant::default();PLANT_COUNT]
        }
    }
}

#[derive(Serialize, Deserialize, Copy, Clone)]
#[derive(Debug)]
pub struct Plant{
    target_moisture: u8,
    pump_time_s: u16,
    pump_cooldown_min: u16,
    pump_hour_start: u8,
    pump_hour_end: u8
}
impl Default for Plant {
    fn default() -> Self { 
        Self { target_moisture: 40, pump_time_s: 60, pump_cooldown_min: 60, pump_hour_start: 8, pump_hour_end: 20 }
    }
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