use std::fmt;

use serde::{Deserialize, Serialize};

use crate::PLANT_COUNT;

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct Config {
    pub mqtt_url: heapless::String<128>,
    pub base_topic: heapless::String<64>,
    pub max_consecutive_pump_count: u8,

    pub tank_allow_pumping_if_sensor_error: bool,
    pub tank_sensor_enabled: bool,
    pub tank_useable_ml: u32,
    pub tank_warn_percent: u8,
    pub tank_empty_mv: f32,
    pub tank_full_mv: f32,

    pub night_lamp_hour_start: u8,
    pub night_lamp_hour_end: u8,
    pub night_lamp_only_when_dark: bool,

    pub plants: [Plant; PLANT_COUNT],
}

impl Default for Config {
    fn default() -> Self {
        Self {
            base_topic: "plant/one".into(),
            mqtt_url: "mqtt://192.168.1.1:1883".into(),
            tank_allow_pumping_if_sensor_error: true,
            tank_sensor_enabled: true,
            tank_warn_percent: 50,
            night_lamp_hour_start: 21,
            night_lamp_hour_end: 2,
            night_lamp_only_when_dark: true,
            plants: [Plant::default(); PLANT_COUNT],
            max_consecutive_pump_count: 15,
            tank_useable_ml: 5000,
            tank_empty_mv: 0.1,
            tank_full_mv: 3.3,
        }
    }
}
#[derive(Serialize, Deserialize, Copy, Clone, Debug, PartialEq)]
pub enum Mode {
    OFF,
    TargetMoisture,
    TimerOnly,
}

#[derive(Serialize, Deserialize, Copy, Clone, Debug, PartialEq)]
pub struct Plant {
    pub mode: Mode,
    pub target_moisture: u8,
    pub pump_time_s: u16,
    pub pump_cooldown_min: u16,
    pub pump_hour_start: u8,
    pub pump_hour_end: u8,
}
impl Default for Plant {
    fn default() -> Self {
        Self {
            target_moisture: 40,
            pump_time_s: 60,
            pump_cooldown_min: 60,
            pump_hour_start: 8,
            pump_hour_end: 20,
            mode: Mode::OFF,
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct WifiConfig {
    pub ssid: heapless::String<32>,
    pub password: Option<heapless::String<64>>,
}

impl fmt::Display for WifiConfig {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "({}, ****)", self.ssid)
    }
}
