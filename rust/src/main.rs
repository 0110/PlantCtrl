use std::{
    env,
    sync::{atomic::AtomicBool, Arc, Mutex},
};

use anyhow::{Result, bail};
use chrono::{Datelike, Duration, NaiveDateTime, Timelike, DateTime};
use chrono_tz::{Europe::Berlin, Tz};
use esp_idf_hal::delay::Delay;
use esp_idf_sys::{esp_restart, vTaskDelay, CONFIG_FREERTOS_HZ, esp_deep_sleep};
use esp_ota::rollback_and_reboot;
use log::error;
use once_cell::sync::Lazy;
use plant_hal::{CreatePlantHal, PlantCtrlBoard, PlantCtrlBoardInteraction, PlantHal, PLANT_COUNT};
use serde::{Deserialize, Serialize};

use crate::{
    config::{Config, WifiConfig},
    webserver::webserver::{httpd, httpd_initial},
};
pub mod bq34z100;
mod config;
pub mod plant_hal;

const MOIST_SENSOR_MAX_FREQUENCY: u32 = 5200; // 60kHz (500Hz margin)
const MOIST_SENSOR_MIN_FREQUENCY: u32 = 500; // 0.5kHz (500Hz margin)

const FROM: (f32, f32) = (
    MOIST_SENSOR_MIN_FREQUENCY as f32,
    MOIST_SENSOR_MAX_FREQUENCY as f32,
);
const TO: (f32, f32) = (0_f32, 100_f32);

mod webserver {
    pub mod webserver;
}

#[derive(Serialize, Deserialize, Copy, Clone, Debug, PartialEq)]
enum OnlineMode {
    Offline,
    Wifi,
    SnTp,
}

#[derive(Serialize, Deserialize, Copy, Clone, Debug, PartialEq)]
enum WaitType {
    InitialConfig,
    FlashError,
    NormalConfig,
    StayAlive,
}

#[derive(Serialize, Deserialize, Copy, Clone, Debug, PartialEq, Default)]
struct LightState{
    active: bool,
    out_of_work_hour: bool,
    battery_low: bool,
    is_day: bool
}

#[derive(Serialize, Deserialize, Copy, Clone, Debug, PartialEq, Default)]
struct PlantState {
    a: Option<u8>,
    b: Option<u8>,
    p: Option<u8>,
    after_p: Option<u8>,
    do_water: bool,
    dry: bool,
    active: bool,
    pump_error: bool,
    not_effective: bool,
    cooldown: bool,
    no_water: bool,
    sensor_error_a: bool,
    sensor_error_b: bool,
    sensor_error_p: bool,
    out_of_work_hour: bool
}

fn wait_infinity(wait_type: WaitType, reboot_now: Arc<AtomicBool>) -> ! {
    let delay = match wait_type {
        WaitType::InitialConfig => 250_u32,
        WaitType::FlashError => 100_u32,
        WaitType::NormalConfig => 500_u32,
        WaitType::StayAlive => 1000_u32,
    };
    let led_count = match wait_type {
        WaitType::InitialConfig => 8,
        WaitType::FlashError => 8,
        WaitType::NormalConfig => 4,
        WaitType::StayAlive => 2,
    };
    loop {
        unsafe {
            //do not trigger watchdog
            for i in 0..8 {
                BOARD_ACCESS.lock().unwrap().fault(i, i < led_count);
            }
            BOARD_ACCESS.lock().unwrap().general_fault(true);
            vTaskDelay(delay);
            BOARD_ACCESS.lock().unwrap().general_fault(false);
            for i in 0..8 {
                BOARD_ACCESS.lock().unwrap().fault(i, false);
            }
            vTaskDelay(delay);
            if wait_type == WaitType::StayAlive
                && !STAY_ALIVE.load(std::sync::atomic::Ordering::Relaxed)
            {
                reboot_now.store(true, std::sync::atomic::Ordering::Relaxed);
            }
            if reboot_now.load(std::sync::atomic::Ordering::Relaxed) {
                println!("Rebooting");
                esp_restart();
            }
        }
    }
}

pub static BOARD_ACCESS: Lazy<Mutex<PlantCtrlBoard>> = Lazy::new(|| PlantHal::create().unwrap());
pub static STAY_ALIVE: Lazy<AtomicBool> = Lazy::new(|| AtomicBool::new(false));

fn map_range(from_range: (f32, f32), s: f32) -> Result<f32> {
    if s < from_range.0 {
        bail!("Value out of range, min {} but current is {}", from_range.0, s);
    }
    if s > from_range.1 {
        bail!("Value out of range, max {} but current is {}", from_range.1, s);
    }
    return Ok(TO.0 + (s - from_range.0) * (TO.1 - TO.0) / (from_range.1 - from_range.0));
}

fn map_range_moisture(s: f32) -> Result<u8> {
    if s < FROM.0 {
        bail!("Value out of range, min {} but current is {}", FROM.0, s);
    }
    if s > FROM.1 {
        bail!("Value out of range, max {} but current is {}", FROM.1, s);
    }
    let tmp = TO.0 + (s - FROM.0) * (TO.1 - TO.0) / (FROM.1 - FROM.0);
    return Ok(tmp as u8);
}

fn in_time_range(cur: DateTime<Tz>, start:u8, end:u8) -> bool{
    let curhour = cur.hour() as u8;
    //eg 10-14
    if start < end {
        return curhour > start && curhour < end;
    } else {
        //eg 20-05
        return curhour > start || curhour < end;
    }
}

fn determine_next_plant(plantstate: &mut [PlantState;PLANT_COUNT],cur: DateTime<Tz>, enough_water: bool, tank_sensor_error: bool, config: &Config, board: &mut std::sync::MutexGuard<'_, PlantCtrlBoard<'_>>) -> Option<usize> {
    for plant in 0..PLANT_COUNT {
        let state = &mut plantstate[plant];
        let plant_config = config.plants[plant];
        match plant_config.mode {
            config::Mode::OFF => {

            },
            config::Mode::TargetMoisture => {
                match board.measure_moisture_hz(plant, plant_hal::Sensor::A).and_then (|moist| map_range_moisture(moist as f32)) {
                    Ok(a) => state.a = Some(a),
                    Err(err) => {
                        board.fault(plant, true);
                        println!("Could not determine Moisture A for plant {} due to {}", plant, err);
                        state.a  = None;
                        state.sensor_error_a = true;
                    }
                }
                match board.measure_moisture_hz(plant, plant_hal::Sensor::B).and_then (|moist| map_range_moisture(moist as f32)) {
                    Ok(b) => state.b = Some(b),
                    Err(err) => {
                        board.fault(plant, true);
                        println!("Could not determine Moisture B for plant {} due to {}", plant, err);
                        state.b  = None;
                        state.sensor_error_b = true;
                    }
                }
                        //FIXME how to average analyze whatever?
                let a_low = state.a.is_some() && state.a.unwrap() < plant_config.target_moisture;
                let b_low = state.b.is_some() && state.b.unwrap() < plant_config.target_moisture;
                
                if a_low || b_low {
                    state.dry = true;
                    if tank_sensor_error && !config.tank_allow_pumping_if_sensor_error || !enough_water {
                        state.no_water = true;
                    }
                }
                let duration = Duration::minutes((60 * plant_config.pump_cooldown_min).into());
                let next_pump = board.last_pump_time(plant) + duration;
                if next_pump > cur {
                    state.cooldown = true;
                }
                if !in_time_range(cur, plant_config.pump_hour_start, plant_config.pump_hour_end) {
                    state.out_of_work_hour = true;
                }

                if state.dry && !state.no_water && !state.cooldown && !state.out_of_work_hour {
                    state.do_water = true;
                }
            },
            config::Mode::TimerOnly => {
                let duration = Duration::minutes((60 * plant_config.pump_cooldown_min).into());
                let next_pump = board.last_pump_time(plant) + duration;
                if next_pump > cur {
                    state.cooldown = true;
                } else {
                    state.do_water = true;
                }
            },
            config::Mode::TimerAndDeadzone => {
                let duration = Duration::minutes((60 * plant_config.pump_cooldown_min).into());
                let next_pump = board.last_pump_time(plant) + duration;
                if next_pump > cur {
                    state.cooldown = true;
                } 
                if !in_time_range(cur, plant_config.pump_hour_start, plant_config.pump_hour_end) {
                    state.out_of_work_hour = true;
                }
                if !state.cooldown && !state.out_of_work_hour {
                    state.do_water = true;
                }
            },
        }
        //FIXME publish state here!
        if state.do_water{
            if board.consecutive_pump_count(plant) > config.max_consecutive_pump_count.into() {
                state.not_effective = true;
                board.fault(plant, true);
            }
        } else {
            board.store_consecutive_pump_count(plant, 0);
        }
        println!("Plant {} state is {:?}", plant, state);
    }
    for plant in 0..PLANT_COUNT {
        let state = &plantstate[plant];
        println!("Checking for water plant {} with state {}", plant, state.do_water);
        if state.do_water {
            return Some(plant);
        }
    }
    println!("No plant needs water");
    return None
}

fn safe_main() -> Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    if esp_idf_sys::CONFIG_MAIN_TASK_STACK_SIZE < 20000 {
        error!(
            "stack too small: {} bail!",
            esp_idf_sys::CONFIG_MAIN_TASK_STACK_SIZE
        );
        return Ok(());
    }

    log::info!("Startup Rust");

    let git_hash = env!("VERGEN_GIT_DESCRIBE");
    println!("Version useing git has {}", git_hash);

    let mut partition_state: embedded_svc::ota::SlotState = embedded_svc::ota::SlotState::Unknown;
    match esp_idf_svc::ota::EspOta::new() {
         Ok(ota) => {
             //match ota.get_running_slot(){
               //  Ok(slot) => {
    //                 partition_state = slot.state;
    //                 println!(
    //                     "Booting from {} with state {:?}",
    //                     slot.label, partition_state
    //                 );
                 //},
                // Err(err) => {
                  //   println!("Error getting running slot {}", err);
                // },
             //}
         },
         Err(err) => {
             println!("Error obtaining ota info {}", err);
         },
    }

    println!("Board hal init");
    let mut board: std::sync::MutexGuard<'_, PlantCtrlBoard<'_>> = BOARD_ACCESS.lock().unwrap();

    println!("Mounting filesystem");
    board.mount_file_system()?;
    let free_space = board.file_system_size()?;
    println!(
        "Mounted, total space {} used {} free {}",
        free_space.total_size, free_space.used_size, free_space.free_size
    );

    let time = board.time();
    let mut cur = match time {
        Ok(cur) => cur,
        Err(err) => {
            log::error!("time error {}", err);
            NaiveDateTime::from_timestamp_millis(0).unwrap().and_utc()
        }
    };
    //check if we know the time current > 2020
    if cur.year() < 2020 {
        if board.is_day() {
            //assume TZ safe times ;)
            cur = *cur.with_hour(15).get_or_insert(cur);
        } else {
            cur = *cur.with_hour(3).get_or_insert(cur);
        }
    }

    println!("cur is {}", cur);

    if board.is_config_reset() {
        board.general_fault(true);
        println!("Reset config is pressed, waiting 5s");
        for _i in 0..25 {
            board.general_fault(true);
            Delay::new_default().delay_ms(50);
            board.general_fault(false);
            Delay::new_default().delay_ms(50);
        }

        if board.is_config_reset() {
            println!("Reset config is still pressed, deleting configs and reboot");
            match board.remove_configs() {
                Ok(case) => {
                    println!("Succeeded in deleting config {}", case);
                }
                Err(err) => {
                    println!("Could not remove config files, system borked {}", err);
                    //terminate main app and freeze

                    wait_infinity(WaitType::FlashError, Arc::new(AtomicBool::new(false)));
                }
            }
        } else {
            board.general_fault(false);
        }
    }

    let mut online_mode = OnlineMode::Offline;
    let wifi_conf = board.get_wifi();
    let wifi: WifiConfig;
    match wifi_conf {
        Ok(conf) => {
            wifi = conf;
        }
        Err(err) => {
            if board.is_wifi_config_file_existant() {
                match partition_state {
                    embedded_svc::ota::SlotState::Invalid
                    | embedded_svc::ota::SlotState::Unverified => {
                        println!("Config seem to be unparsable after upgrade, reverting");
                        rollback_and_reboot()?;
                    }
                    _ => {}
                }
            }
            println!("Missing wifi config, entering initial config mode {}", err);
            board.wifi_ap().unwrap();
            //config upload will trigger reboot!
            drop(board);
            let reboot_now = Arc::new(AtomicBool::new(false));
            let _webserver = httpd_initial(reboot_now.clone());
            wait_infinity(WaitType::InitialConfig, reboot_now.clone());
        }
    };

    println!("attempting to connect wifi");
    match board.wifi(&wifi.ssid, wifi.password.as_deref(), 10000) {
        Ok(_) => {
            online_mode = OnlineMode::Wifi;
        }
        Err(_) => {
            println!("Offline mode");
            board.general_fault(true);
        }
    }

    if online_mode == OnlineMode::Wifi {
        match board.sntp(1000 * 120) {
            Ok(new_time) => {
                cur = new_time;
                online_mode = OnlineMode::SnTp;
            }
            Err(err) => {
                println!("sntp error: {}", err);
                board.general_fault(true);
            }
        }
    }

    println!("Running logic at utc {}", cur);
    let europe_time = cur.with_timezone(&Berlin);
    println!("Running logic at europe/berlin {}", europe_time);

    let config: Config;
    match board.get_config() {
        Ok(valid) => {
            config = valid;
        }
        Err(err) => {
            println!("Missing normal config, entering config mode {}", err);
            //config upload will trigger reboot!
            drop(board);
            let reboot_now = Arc::new(AtomicBool::new(false));
            let _webserver = httpd(reboot_now.clone());
            wait_infinity(WaitType::NormalConfig, reboot_now.clone());
        }
    }

    //do mqtt before config check, as mqtt might configure
    if online_mode == OnlineMode::SnTp {
        match board.mqtt(&config) {
            Ok(_) => {
                println!("Mqtt connection ready");
            }
            Err(err) => {
                println!("Could not connect mqtt due to {}", err);
            }
        }
    }

    match board.battery_state() {
        Ok(_state) => {}
        Err(err) => {
            board.general_fault(true);
            println!("Could not read battery state, assuming low power {}", err);
        }
    }

    let mut enough_water = true;
    let mut tank_sensor_error = false;
    if config.tank_sensor_enabled {
        let mut tank_value_r = 0;

        let success = board.tank_sensor_mv().and_then(|raw| {
            tank_value_r = raw;
            return map_range(
                (config.tank_empty_mv as f32, config.tank_full_mv as f32),
                raw as f32,
            );
        }).and_then(|percent| {
            let left_ml = ((percent / 100_f32) * config.tank_useable_ml as f32) as u32;
            println!(
                "Tank sensor returned mv {} as {}% leaving {} ml useable",
                tank_value_r, percent as u8, left_ml
            );
            if config.tank_warn_percent > percent as u8 {
                board.general_fault(true);
                println!(
                    "Low water, current percent is {}, minimum warn level is {}",
                    percent as u8, config.tank_warn_percent
                );
            }
            if config.tank_warn_percent <= 0 {
                enough_water = false;
            }
            return Ok(());
        });
        match success {
            Err(err) => {
                println!("Could not determine tank value due to {}", err);
                board.general_fault(true);
                tank_sensor_error = true;
            }
            Ok(_) => {},
        }
    }

    let mut water_frozen = false;
    for _attempt in 0..5 {
        let water_temperature = board.water_temperature_c();
        match water_temperature {
            Ok(temp) => {
                //FIXME mqtt here
                println!("Water temp is {}", temp);
                if temp < 4_f32 {
                    water_frozen = true;
                }
                break;
            },
            Err(err) => {
                println!("Could not get water temp {}", err)
            },
        }
    }
    
    let mut plantstate = [PlantState {
        ..Default::default()
    }; PLANT_COUNT];
    let plant_to_pump = determine_next_plant(&mut plantstate, europe_time, enough_water, tank_sensor_error, &config, &mut board);

    if STAY_ALIVE.load(std::sync::atomic::Ordering::Relaxed) {
        drop(board);
        let reboot_now = Arc::new(AtomicBool::new(false));
        let _webserver = httpd(reboot_now.clone());
        wait_infinity(WaitType::StayAlive, reboot_now.clone());
    }
    
    match plant_to_pump {
        Some(plant) => {
            let mut state = plantstate[plant];
            let consecutive_pump_count = board.consecutive_pump_count(plant) + 1;
            board.store_consecutive_pump_count(plant, consecutive_pump_count);
            let plant_config = config.plants[plant];
            println!("Trying to pump for {}s with pump {} now", plant_config.pump_time_s,plant);
            
            board.any_pump(true)?;
            board.store_last_pump_time(plant, cur);
            board.pump(plant, true)?;
            board.last_pump_time(plant);
            state.active = true;
            //FIXME do periodic pump test here and state update
            unsafe { vTaskDelay(plant_config.pump_time_s as u32*CONFIG_FREERTOS_HZ) };
            match map_range_moisture(board.measure_moisture_hz(plant, plant_hal::Sensor::PUMP)? as f32) {
                Ok(p) => state.after_p = Some(p),
                Err(err) => {
                    board.fault(plant, true);
                    println!("Could not determine Moisture P after for plant {} due to {}", plant, err);
                    state.after_p  = None;
                    state.sensor_error_p = true;
                }
            }
            if state.after_p.is_none() || state.p.is_none() || state.after_p.unwrap() < state.p.unwrap() + 5 {
                state.pump_error = true;
                board.fault(plant, true);
            }
        },
        None => {
            println!("Nothing to do");
        }
        ,
    }

    let mut light_state = LightState{ ..Default::default() };
    light_state.is_day = board.is_day();
    light_state.out_of_work_hour = !in_time_range(europe_time, config.night_lamp_hour_start, config.night_lamp_hour_end);    
    if !light_state.out_of_work_hour {
        if config.night_lamp_only_when_dark {
            if !light_state.is_day {
                board.light(true).unwrap();
            }
        }else {
            board.light(true).unwrap();
        }
    } else {
        board.light(false).unwrap();
    }
    println!("Lightstate is {:?}", light_state);
    
    //check if during light time
        //lightstate += out of worktime
        //check battery level
        //lightstate += battery empty
        //check solar level if config requires
        //lightstate += stillday
        //if no preventing lightstate, enable light
        //lightstate = active

    //deepsleep here?
    unsafe { esp_deep_sleep(1000*1000*10) };
}

fn main(){
    let result = safe_main();
    result.unwrap();
}
//error codes
//error_reading_config_after_upgrade
//error_no_config_after_upgrade
//error_tank_sensor_fault
