use std::{sync::{Arc, Mutex, atomic::AtomicBool}, env};

use chrono::{Datelike, NaiveDateTime, Timelike};
use once_cell::sync::Lazy;
use anyhow::Result;
use chrono_tz::Europe::Berlin;
use esp_idf_hal::delay::Delay;
use esp_idf_sys::{esp_restart, vTaskDelay};
use plant_hal::{CreatePlantHal, PlantCtrlBoard, PlantCtrlBoardInteraction, PlantHal, PLANT_COUNT};

use crate::{config::{Config, WifiConfig}, webserver::webserver::{httpd_initial, httpd}};
mod config;
pub mod plant_hal;
mod webserver {
    pub mod webserver;
}

#[derive(PartialEq)]
enum OnlineMode {
    Offline,
    Wifi,
    SnTp,
    Mqtt,
    MqttRoundtrip
}

enum WaitType{
    InitialConfig,
    FlashError,
    NormalConfig
}

fn wait_infinity(wait_type:WaitType, reboot_now:Arc<AtomicBool>)  -> !{
    let delay = match wait_type {
        WaitType::InitialConfig => 250_u32,
        WaitType::FlashError => 100_u32,
        WaitType::NormalConfig => 500_u32
    };
    let led_count = match wait_type {
        WaitType::InitialConfig => 8,
        WaitType::FlashError => 8,
        WaitType::NormalConfig => 4
    };
    BOARD_ACCESS.lock().unwrap().light(true).unwrap();
    loop {
        unsafe {
            //do not trigger watchdog
            for i in 0..8 {
                BOARD_ACCESS.lock().unwrap().fault(i, i <led_count);    
            }
            BOARD_ACCESS.lock().unwrap().general_fault(true);
            vTaskDelay(delay);
            BOARD_ACCESS.lock().unwrap().general_fault(false);
            for i in 0..8 {
                BOARD_ACCESS.lock().unwrap().fault(i, false);    
            }
            vTaskDelay(delay);
            if reboot_now.load(std::sync::atomic::Ordering::Relaxed) {
                println!("Rebooting");
                esp_restart();
            }
        }
    }
}

pub static BOARD_ACCESS: Lazy<Mutex<PlantCtrlBoard>> = Lazy::new(|| {
    PlantHal::create().unwrap()
});

fn main() -> Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Startup Rust");

    let git_hash = env!("VERGEN_GIT_DESCRIBE");
    println!("Version useing git has {}", git_hash);

    println!("Board hal init");
    let mut board = BOARD_ACCESS.lock().unwrap();
    println!("Mounting filesystem");
    board.mountFileSystem()?;
    let free_space = board.fileSystemSize()?;
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
        println!("Reset config is pressed, waiting 5s");
        Delay::new_default().delay_ms(5000);
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
        }
    }

    let mut online_mode = OnlineMode::Offline;
    let wifi_conf = board.get_wifi();
    let wifi: WifiConfig;
    match wifi_conf{
        Ok(conf) => {
            wifi = conf;
        },
        Err(err) => {
            println!("Missing wifi config, entering initial config mode {}", err);
            board.wifi_ap().unwrap();
            //config upload will trigger reboot!
            drop(board);
            let reboot_now = Arc::new(AtomicBool::new(false));
            let _webserver = httpd_initial(reboot_now.clone());
            wait_infinity(WaitType::InitialConfig, reboot_now.clone());
        },
    };


    //check if we have a config file
    // if not found or parsing error -> error very fast blink general fault
    //if this happens after a firmeware upgrade (check image state), mark as invalid
    //blink general fault error_reading_config_after_upgrade, reboot after
    // open accesspoint with webserver for wlan mqtt setup
    //blink general fault error_no_config_after_upgrade
    //once config is set store it and reboot

    //if proceed.tank_sensor_enabled() {

    //}
    //is tank sensor enabled in config?
    //measure tank level (without wifi due to interference)
    //TODO this should be a result// detect invalid measurement value
    let tank_value = board.tank_sensor_mv();
    match tank_value {
        Ok(tank_raw) => {
            println!("Tank sensor returned {}", tank_raw);
        }
        Err(_) => {
            //if not possible value, blink general fault error_tank_sensor_fault
            board.general_fault(true);
            //set general fault persistent
            //set tank sensor state to fault
        }
    }

    //measure each plant moisture
    let mut initial_measurements_a: [i32; PLANT_COUNT] = [0; PLANT_COUNT];
    let mut initial_measurements_b: [i32; PLANT_COUNT] = [0; PLANT_COUNT];
    let mut initial_measurements_p: [i32; PLANT_COUNT] = [0; PLANT_COUNT];
    for plant in 0..PLANT_COUNT {
        initial_measurements_a[plant] = board.measure_moisture_hz(plant, plant_hal::Sensor::A)?;
        initial_measurements_b[plant] = board.measure_moisture_hz(plant, plant_hal::Sensor::B)?;
        initial_measurements_p[plant] =
            board.measure_moisture_hz(plant, plant_hal::Sensor::PUMP)?;
    }

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
            },
            Err(err) => {
                println!("sntp error: {}", err);
                board.general_fault(true);
            }
        }
        println!("Running logic at utc {}", cur);
        let europe_time = cur.with_timezone(&Berlin);
        println!("Running logic at europe/berlin {}", europe_time);
    }

    let config:Config;
    match (board.get_config()){
        Ok(valid) => {
            config = valid;
        },
        Err(err) => {
            println!("Missing normal config, entering config mode {}", err);
            //config upload will trigger reboot!
            drop(board);
            let reboot_now = Arc::new(AtomicBool::new(false));
            let _webserver = httpd(reboot_now.clone());
            wait_infinity(WaitType::NormalConfig, reboot_now.clone());
        },
    }

    if online_mode == OnlineMode::SnTp {
        //mqtt here
    }
    if online_mode == OnlineMode::Mqtt {
        //mqtt roundtrip here
    }
    //TODO configmode webserver logic here

    /*
        

        //if config battery mode
        //read battery level
        //if not possible set general fault persistent, but do continue
        //else
        //assume 12v and max capacity

        //if tank sensor is enabled
        //if tank sensor fault abort if config require is set
        //check if water is > minimum allowed || fault
        //if not, set all plants requiring water to persistent fault

        //for each plant
        //check if moisture is < target
        //state  += dry
        //check if in cooldown
        //state += cooldown
        //check if consecutive pumps > limit
        //state += notworking
        //set plant fault persistent

        //pump one cycle
        // set last pump time to now
        //during pump state += active
        //after pump check if Pump moisture value is increased by config delta x
        // state -= active
        // state += cooldown
        // if not set plant error persistent fault
        // state += notworking
        //set consecutive pumps+=1

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
    return Ok(());
}

//error codes
//error_reading_config_after_upgrade
//error_no_config_after_upgrade
//error_tank_sensor_fault
