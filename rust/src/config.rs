

use crate::PLANT_COUNT;
pub struct Config {
    ssid: heapless::String<32>,
    password: Option<heapless::String<64>>,

    tank_sensor_enabled: bool,
    tank_full_ml: u32,
    tank_warn_percent: u8,

    plantcount: u16,

    pump_duration_ms: [u16;PLANT_COUNT],
    pump_cooldown_min: [u16;PLANT_COUNT],
    pump_hour_start: [u8;PLANT_COUNT],
    pump_hour_end: [u8;PLANT_COUNT],

    night_lamp_hour_start: u8,
    night_lamp_hour_end: u8,
    night_lamp_only_when_dark: u8
    
    
}