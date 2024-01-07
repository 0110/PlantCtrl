use std::{
    env,
    sync::{atomic::AtomicBool, Arc, Mutex},
};

use anyhow::Result;
use chrono::{Datelike, Duration, NaiveDateTime, Timelike};
use chrono_tz::Europe::Berlin;
use esp_idf_hal::delay::Delay;
use esp_idf_sys::{esp_restart, uxTaskGetStackHighWaterMark, vTaskDelay};
use esp_ota::rollback_and_reboot;
use log::error;
use once_cell::sync::Lazy;
use plant_hal::{CreatePlantHal, PlantCtrlBoard, PlantCtrlBoardInteraction, PlantHal, PLANT_COUNT};
use serde::{Deserialize, Serialize};

use crate::{
    config::{Config, WifiConfig},
    webserver::webserver::{httpd, httpd_initial},
};
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
struct PlantState {
    a: u8,
    b: u8,
    p: u8,
    after_p: u8,
    dry: bool,
    active: bool,
    pump_error: bool,
    not_effective: bool,
    cooldown: bool,
    no_water: bool,
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
            {}
            if reboot_now.load(std::sync::atomic::Ordering::Relaxed) {
                println!("Rebooting");
                esp_restart();
            }
        }
    }
}

pub static BOARD_ACCESS: Lazy<Mutex<PlantCtrlBoard>> = Lazy::new(|| PlantHal::create().unwrap());
pub static STAY_ALIVE: Lazy<AtomicBool> = Lazy::new(|| AtomicBool::new(false));

fn map_range(from_range: (f32, f32), to_range: (f32, f32), s: f32) -> f32 {
    to_range.0 + (s - from_range.0) * (to_range.1 - to_range.0) / (from_range.1 - from_range.0)
}

fn main() -> Result<()> {
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
    // match esp_idf_svc::ota::EspOta::new() {
    //     Ok(ota) => {
    //         match ota.get_running_slot(){
    //             Ok(slot) => {
    //                 partition_state = slot.state;
    //                 println!(
    //                     "Booting from {} with state {:?}",
    //                     slot.label, partition_state
    //                 );
    //             },
    //             Err(err) => {
    //                 println!("Error getting running slot {}", err);
    //             },
    //         }
    //     },
    //     Err(err) => {
    //         println!("Error obtaining ota info {}", err);
    //     },
    // }

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
        for i in 0..25 {
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
        println!("Running logic at utc {}", cur);
        let europe_time = cur.with_timezone(&Berlin);
        println!("Running logic at europe/berlin {}", europe_time);
    }

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
    if config.tank_sensor_enabled {
        let tank_value = board.tank_sensor_mv();
        match tank_value {
            Ok(tank_raw) => {
                //FIXME clear
                let percent = map_range(
                    (config.tank_empty_mv, config.tank_full_mv),
                    (0_f32, 100_f32),
                    tank_raw.into(),
                );
                let left_ml = ((percent / 100_f32) * config.tank_useable_ml as f32) as u32;
                println!(
                    "Tank sensor returned mv {} as {}% leaving {} ml useable",
                    tank_raw, percent as u8, left_ml
                );
                if config.tank_warn_percent > percent as u8 {
                    board.general_fault(true);
                    println!(
                        "Low water, current percent is {}, minimum warn level is {}",
                        percent as u8, config.tank_warn_percent
                    );
                    //FIXME warn here
                }
                if config.tank_warn_percent <= 0 {
                    enough_water = false;
                }
            }
            Err(_) => {
                board.general_fault(true);
                if !config.tank_allow_pumping_if_sensor_error {
                    enough_water = false;
                }
                //set tank sensor state to fault
            }
        }
    }

    let plantstate = [PlantState {
        ..Default::default()
    }; PLANT_COUNT];
    for plant in 0..PLANT_COUNT {
        let mut state = plantstate[plant];
        //return mapf(mMoisture_raw.getMedian(), MOIST_SENSOR_MIN_FRQ, MOIST_SENSOR_MAX_FRQ, 0, 100);
        state.a = map_range(
            FROM,
            TO,
            board.measure_moisture_hz(plant, plant_hal::Sensor::A)? as f32,
        ) as u8;
        state.b = map_range(
            FROM,
            TO,
            board.measure_moisture_hz(plant, plant_hal::Sensor::B)? as f32,
        ) as u8;
        state.p = map_range(
            FROM,
            TO,
            board.measure_moisture_hz(plant, plant_hal::Sensor::PUMP)? as f32,
        ) as u8;
        let plant_config = config.plants[plant];

        //FIXME how to average analyze whatever?
        if state.a < plant_config.target_moisture || state.b < plant_config.target_moisture {
            state.dry = true;
            if !enough_water {
                state.no_water = true;
            }
        }

        let duration = Duration::minutes((60 * plant_config.pump_cooldown_min).into());
        if (board.last_pump_time(plant)? + duration) > cur {
            state.cooldown = true;
        }

        if state.dry {
            let consecutive_pump_count = board.consecutive_pump_count(plant) + 1;
            board.store_consecutive_pump_count(plant, consecutive_pump_count);
            if consecutive_pump_count > config.max_consecutive_pump_count.into() {
                state.not_effective = true;
                board.fault(plant, true);
            }
        } else {
            board.store_consecutive_pump_count(plant, 0);
        }

        //TODO update mqtt state here!
    }

    if (STAY_ALIVE.load(std::sync::atomic::Ordering::Relaxed)) {
        drop(board);
        let reboot_now = Arc::new(AtomicBool::new(false));
        let _webserver = httpd(reboot_now.clone());
        wait_infinity(WaitType::StayAlive, reboot_now.clone());
    }

    'eachplant: for plant in 0..PLANT_COUNT {
        let mut state = plantstate[plant];
        if (state.dry && !state.cooldown) {
            println!("Trying to pump with pump {} now", plant);
            let plant_config = config.plants[plant];

            board.any_pump(true)?;
            board.store_last_pump_time(plant, cur);
            board.pump(plant, true)?;
            board.last_pump_time(plant)?;
            state.active = true;
            unsafe { vTaskDelay(plant_config.pump_time_s.into()) };
            state.after_p = map_range(
                FROM,
                TO,
                board.measure_moisture_hz(plant, plant_hal::Sensor::PUMP)? as f32,
            ) as u8;
            if state.after_p < state.p + 5 {
                state.pump_error = true;
                board.fault(plant, true);
            }
            break 'eachplant;
        }
    }

    /*

        //check if during light time
        //lightstate += out of worktime
        //check battery level
        //lightstate += battery empty
        //check solar level if config requires
        //lightstate += stillday
        //if no preventing lightstate, enable light
        //lightstate = active

        //keep webserver in scope
        let webserver = httpd(true);
        let delay = Delay::new_default();
        loop {
            //let freertos do shit
            delay.delay_ms(1001);
        }
    */
    //deepsleep here?
    Ok(())
}

//error codes
//error_reading_config_after_upgrade
//error_no_config_after_upgrade
//error_tank_sensor_fault
