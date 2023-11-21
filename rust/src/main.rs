
use std::sync::{Mutex, Arc};

use chrono::{Utc, NaiveDateTime, DateTime};

use ds18b20::Ds18b20;
use embedded_hal::digital::v1_compat::OldOutputPin;
use embedded_hal::digital::v2::OutputPin;
use esp_idf_hal::adc::config::Config;
use esp_idf_hal::adc::{AdcDriver, AdcChannelDriver, attenuation};
use esp_idf_hal::delay::Delay;
use esp_idf_hal::reset::ResetReason;
use esp_idf_sys::EspError;
use one_wire_bus::OneWire;
use shift_register_driver::sipo::ShiftRegister24;
use esp_idf_hal::gpio::{PinDriver, Gpio39, Gpio4};
use esp_idf_hal::prelude::Peripherals;

const PLANT_COUNT:usize = 8;
const PINS_PER_PLANT:usize = 5;
const PLANT_PUMP_OFFSET:usize = 0;
const PLANT_FAULT_OFFSET:usize = 1;
const PLANT_MOIST_PUMP_OFFSET:usize = 2;
const PLANT_MOIST_B_OFFSET:usize = 3;
const PLANT_MOIST_A_OFFSET:usize = 4;


#[link_section = ".rtc.data"]
static mut LAST_WATERING_TIMESTAMP: [i64; PLANT_COUNT] = [0; PLANT_COUNT];
#[link_section = ".rtc.data"]
static mut CONSECUTIVE_WATERING_PLANT: [u32; PLANT_COUNT] = [0; PLANT_COUNT];
#[link_section = ".rtc.data"]
static mut LOW_VOLTAGE_DETECTED:bool = false;

struct BatteryState {
    state_charge_percent: u8,
    max_error_percent: u8,
    remaining_milli_ampere_hour: u32,
    max_milli_ampere_hour: u32,
    design_milli_ampere_hour:u32,
    voltage_milli_volt: u16,
    average_current_milli_ampere: u16,
    temperature_tenth_kelvin: u32,
    average_time_to_empty_minute: u16,
    average_time_to_full_minute: u16,
    average_discharge_power_cycle_milli_watt: u16,
    cycle_count: u16,
    state_health_percent: u8
}
trait PlantCtrlBoardInteraction{
    fn battery_state(&mut self,) -> BatteryState;
    
    fn is_day(&self,) -> bool;
    fn water_temperature_c(&mut self,) -> Option<f32>;
    fn tank_sensor_mv(&mut self,) -> u16;

    fn set_low_voltage_in_cycle(&mut self,);
    fn clear_low_voltage_in_cycle(&mut self,);
    fn low_voltage_in_cycle(&mut self) -> bool;
    fn any_pump(&mut self, enabled:bool);

    //keep state during deepsleep
    fn light(&self,enable:bool);

    fn plant_count(&self,) -> usize;
    fn measure_moisture_b_hz(&self,plant:usize) -> i16;
    fn measure_moisture_a_hz(&self,plant:usize) -> i16;
    fn measure_moisture_p_hz(&self,plant:usize) -> i16;
    fn pump(&self,plant:usize, enable:bool);
    fn last_pump_time(&self,plant:usize) -> chrono::DateTime<Utc>;
    fn store_last_pump_time(&mut self,plant:usize, time: chrono::DateTime<Utc>);
    fn store_consecutive_pump_count(&mut self,plant:usize, count:u32);
    fn consecutive_pump_count(&mut self,plant:usize) -> u32;

    //keep state during deepsleep
    fn fault(&self,plant:usize, enable:bool);
    
    fn default() -> Self;
}


struct PlantCtrlBoard<'a>{
    shift_register: ShiftRegister24<OldOutputPin<PinDriver<'a, esp_idf_hal::gpio::Gpio21, esp_idf_hal::gpio::Output>>, OldOutputPin<PinDriver<'a, esp_idf_hal::gpio::Gpio22, esp_idf_hal::gpio::Output>>, OldOutputPin<PinDriver<'a, esp_idf_hal::gpio::Gpio19, esp_idf_hal::gpio::Output>>>,
    consecutive_watering_plant: Mutex<[u32; PLANT_COUNT]>,
    last_watering_timestamp: Mutex<[i64; PLANT_COUNT]>,
    low_voltage_detected: Mutex<bool>,
    tank_driver: AdcDriver<'a, esp_idf_hal::adc::ADC1>,
    tank_channel: esp_idf_hal::adc::AdcChannelDriver<'a, { attenuation::DB_11 }, Gpio39 >,
    solar_is_day: PinDriver<'a, esp_idf_hal::gpio::Gpio25, esp_idf_hal::gpio::Input>,
    water_temp_sensor: Option<Ds18b20>,
    one_wire_bus: OneWire<PinDriver<'a, Gpio4, esp_idf_hal::gpio::InputOutput>>
}

impl PlantCtrlBoardInteraction for PlantCtrlBoard<'_> {
    fn default() -> Self {
        let peripherals = Peripherals::take().unwrap(); 
        
        let clock = OldOutputPin::from(PinDriver::output(peripherals.pins.gpio21).unwrap());
        let latch = OldOutputPin::from(PinDriver::output(peripherals.pins.gpio22).unwrap());
        let data = OldOutputPin::from(PinDriver::output(peripherals.pins.gpio19).unwrap());


        let one_wire_pin = PinDriver::input_output_od(peripherals.pins.gpio4).unwrap();

        let mut one_wire_bus: OneWire<PinDriver<'_, Gpio4, esp_idf_hal::gpio::InputOutput>> = OneWire::new(one_wire_pin).unwrap();
        let mut delay = Delay::new_default();

        if one_wire_bus.reset(&mut delay).is_err() {
            //TODO check a lot of one wire error conditions here
        }

        let device_address = one_wire_bus.devices(false, &mut delay).next().unwrap().unwrap();
        let water_temp_sensor: Option<Ds18b20> = Ds18b20::new::<EspError>(device_address).ok();
        //TODO make to none if not possible to init

        //init,reset rtc memory depending on cause
        let reasons = ResetReason::get();
        let reset_store = match reasons {
            ResetReason::Software => false,
            ResetReason::ExternalPin => false,
            ResetReason::Watchdog => true,
            ResetReason::Sdio => true,
            ResetReason::Panic => true,
            ResetReason::InterruptWatchdog => true,
            ResetReason::PowerOn => true,
            ResetReason::Unknown => true,
            ResetReason::Brownout => true,
            ResetReason::TaskWatchdog => true,
            ResetReason::DeepSleep => false,
        };
        if reset_store {
            println!("Clear and reinit RTC store");
            unsafe {
                LAST_WATERING_TIMESTAMP = [0; PLANT_COUNT];
                CONSECUTIVE_WATERING_PLANT = [0; PLANT_COUNT];
                LOW_VOLTAGE_DETECTED = false;
            };
        } else {
            println!("Keeping RTC store");
        }

        Self { 
            shift_register : ShiftRegister24::new(clock, latch, data),
            last_watering_timestamp : Mutex::new(unsafe { LAST_WATERING_TIMESTAMP }),
            consecutive_watering_plant : Mutex::new(unsafe { CONSECUTIVE_WATERING_PLANT }),
            low_voltage_detected : Mutex::new(unsafe { LOW_VOLTAGE_DETECTED }),
            tank_driver : AdcDriver::new(peripherals.adc1, &Config::new().calibration(true)).unwrap(),
            tank_channel: AdcChannelDriver::new(peripherals.pins.gpio39).unwrap(),
            solar_is_day : PinDriver::input(peripherals.pins.gpio25).unwrap(),
            water_temp_sensor : water_temp_sensor,
            one_wire_bus: one_wire_bus,
        }
    }

    fn battery_state(&mut self,) -> BatteryState {
        todo!()
    }

    fn is_day(&self,) -> bool {
        return self.solar_is_day.get_level().into();
    }

    fn water_temperature_c(&mut self,) -> Option<f32> {
        return match &self.water_temp_sensor{
            Some(sensor) => {
                let mut delay = Delay::new_default();
                sensor.start_temp_measurement(&mut self.one_wire_bus, &mut delay);
                ds18b20::Resolution::Bits12.delay_for_measurement_time(&mut delay);    
                let sensor_data  = sensor.read_data(&mut self.one_wire_bus, &mut delay).unwrap();
                println!("Water Temp is {}°C", sensor_data.temperature);
                if sensor_data.temperature == 85_f32 {
                    return Option::None;
                } else {
                    Some(sensor_data.temperature)
                }
            },
            None => Option::None,
        }
    }

    fn tank_sensor_mv(&mut self,) -> u16 {
        return self.tank_driver.read(&mut self.tank_channel).unwrap();
    }

    fn set_low_voltage_in_cycle(&mut self,) {
        *self.low_voltage_detected.get_mut().unwrap() = true;
    }

    fn clear_low_voltage_in_cycle(&mut self,) {
        *self.low_voltage_detected.get_mut().unwrap() = false;
    }

    fn light(&self,enable:bool) {
        todo!()
    }

    fn plant_count(&self,) -> usize {
        PLANT_COUNT
    }

    fn measure_moisture_b_hz(&self,plant:usize) -> i16 {
        todo!()
    }

    fn measure_moisture_a_hz(&self,plant:usize) -> i16 {
        todo!()
    }

    fn measure_moisture_p_hz(&self,plant:usize) -> i16 {
        todo!()
    }

    fn pump(&self,plant:usize, enable:bool) {
        let index = plant*PINS_PER_PLANT*PLANT_PUMP_OFFSET;
        self.shift_register.decompose()[index].set_state(enable.into()).unwrap()
    }

    fn last_pump_time(&self,plant:usize) -> chrono::DateTime<Utc> {
        let ts = unsafe { LAST_WATERING_TIMESTAMP }[plant];
        let timestamp = NaiveDateTime::from_timestamp_millis(ts).unwrap();
        return DateTime::<Utc>::from_naive_utc_and_offset(timestamp, Utc);
    }

    fn store_last_pump_time(&mut self,plant:usize, time: chrono::DateTime<Utc>) {
        self.last_watering_timestamp.get_mut().unwrap()[plant] = time.timestamp_millis();
    }

    fn store_consecutive_pump_count(&mut self,plant:usize, count:u32) {
        self.consecutive_watering_plant.get_mut().unwrap()[plant] = count;
    }

    fn consecutive_pump_count(&mut self,plant:usize) -> u32 {
        return self.consecutive_watering_plant.get_mut().unwrap()[plant]
    }

    fn fault(&self,plant:usize, enable:bool) {
        let index = plant*PINS_PER_PLANT*PLANT_FAULT_OFFSET;
        self.shift_register.decompose()[index].set_state(enable.into()).unwrap()
    }

    fn low_voltage_in_cycle(&mut self) -> bool {
        return *self.low_voltage_detected.get_mut().unwrap()
    }

    fn any_pump(&mut self, enabled:bool) {

        todo!()
    }


}






fn main() {

    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello, world!");



    let board = PlantCtrlBoard::default();


    //check if we know the time current > 2020
        //if failed assume its 1.1.1970 
            //12:00 if solar reports day
            //00:00 if solar repors night
    
    //continous/interrupt?
        //check if boot button is pressed, if longer than 5s delete config and reboot into config mode

    //check if we have a config file
        // if not found or parsing error -> error very fast blink general fault
            //if this happens after a firmeware upgrade (check image state), mark as invalid
                //blink general fault error_reading_config_after_upgrade, reboot after
            // open accesspoint with webserver for wlan mqtt setup
                //blink general fault error_no_config_after_upgrade
                    //once config is set store it and reboot

    //is tank sensor enabled in config?
        //measure tank level (without wifi due to interference)
            //if not possible value, blink general fault error_tank_sensor_fault
                //set general fault persistent
                //set tank sensor state to fault
        
    
    //try connect wifi and do mqtt roundtrip
        // if no wifi, set general fault persistent
            //if no mqtt, set general fault persistent

    //measure each plant moisture

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
    
        
        




}

//error codes
//error_reading_config_after_upgrade
//error_no_config_after_upgrade
//error_tank_sensor_fault