use chrono::{Datelike, Timelike, NaiveDateTime};
use build_time::build_time_utc;

use chrono_tz::Europe::Berlin;
use esp_idf_hal::delay::Delay;
use esp_idf_svc::http::server::EspHttpServer;
use plant_hal::{PlantCtrlBoardInteraction, PlantHal, CreatePlantHal, PLANT_COUNT};
use anyhow::{Context, Result};
use webserver::webserver::httpd;
pub mod plant_hal;
mod config;
mod webserver {
    pub mod webserver;
}

fn main() -> Result<()>{
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Startup Rust");

    let utc_build_time = build_time_utc!();
    println!("Version was build {}", utc_build_time);
    

    let mut board = PlantHal::create()?;     
    
    println!("Board hal init");
     
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

    //continous/interrupt?
        //check if boot button is pressed, if longer than 5s delete config and reboot into config mode


    let config = board.get_config();
    match config {
        Ok(conf) => {

        },
        Err(err) => todo!(),
    }

    let proceed = config.unwrap();
    
    //check if we have a config file
        // if not found or parsing error -> error very fast blink general fault
            //if this happens after a firmeware upgrade (check image state), mark as invalid
                //blink general fault error_reading_config_after_upgrade, reboot after
            // open accesspoint with webserver for wlan mqtt setup
                //blink general fault error_no_config_after_upgrade
                    //once config is set store it and reboot

    if proceed.tank_sensor_enabled() {
        
    }
    //is tank sensor enabled in config?
        //measure tank level (without wifi due to interference)
        //TODO this should be a result// detect invalid measurement value
        let tank_value = board.tank_sensor_mv();
        match tank_value {
            Ok(tank_raw) => {
                println!("Tank sensor returned {}", tank_raw);
            },
            Err(_) => {
                //if not possible value, blink general fault error_tank_sensor_fault
                board.general_fault(true);
                //set general fault persistent
                //set tank sensor state to fault
            },
        }
            

    //measure each plant moisture
    let mut initial_measurements_a: [i32;PLANT_COUNT] = [0;PLANT_COUNT];
    let mut initial_measurements_b: [i32;PLANT_COUNT] = [0;PLANT_COUNT];
    let mut initial_measurements_p: [i32;PLANT_COUNT] = [0;PLANT_COUNT];
    for plant in 0..PLANT_COUNT {
        initial_measurements_a[plant] = board.measure_moisture_hz(plant, plant_hal::Sensor::A)?;
        initial_measurements_b[plant] = board.measure_moisture_hz(plant, plant_hal::Sensor::B)?;
        initial_measurements_p[plant] = board.measure_moisture_hz(plant, plant_hal::Sensor::PUMP)?;
    }

    match board.wifi("C3MA", Some("chaosimquadrat"), 10000) {
        Ok(_) => println!("online mode"),
        Err(_) => {

            println!("Offline mode");
        },
    }
    //try connect wifi and do mqtt roundtrip
        // if no wifi, set general fault persistent
            //if no mqtt, set general fault persistent

    
    match board.sntp(1000*120) {
        Ok(new_time) => cur = new_time,
        Err(err) => {
            println!("sntp error: {}", err);
        }
    }
    println!("Running logic at utc {}", cur);
    let europe_time = cur.with_timezone(&Berlin);
    println!("Running logic at europe/berlin {}", europe_time);



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
        
        return Ok(())

}

//error codes
//error_reading_config_after_upgrade
//error_no_config_after_upgrade
//error_tank_sensor_fault