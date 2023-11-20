use chrono::Utc;
use embedded_hal::digital::v1_compat::OldOutputPin;
use esp_idf_hal::adc::config::Config;
use esp_idf_hal::adc::{AdcDriver, AdcChannelDriver, attenuation};
use esp_idf_hal::reset::ResetReason;
use shift_register_driver::sipo::{ShiftRegister24, ShiftRegisterPin};
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::prelude::Peripherals;

const PLANT_COUNT:usize = 8;
#[link_section = ".rtc.data"]
static mut LAST_WATERING_TIMESTAMP: [u64; PLANT_COUNT] = [0; PLANT_COUNT];
#[link_section = ".rtc.data"]
static mut CONSECUTIVE_WATERING_PLANT: [u64; PLANT_COUNT] = [0; PLANT_COUNT];
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
    fn battery_state(&self,) -> BatteryState;
    
    fn is_day(&self,) -> bool;
    fn water_temperature_c(&self,) -> u16;
    fn tank_sensor_mv(&self,) -> u16;

    fn set_low_voltage_in_cycle(&self,);
    fn clear_low_voltage_in_cycle(&self,);
    fn low_voltage_in_cycle(&self) -> bool;
    fn any_pump(&self, enabled:bool);

    //keep state during deepsleep
    fn light(&self,enable:bool);

    fn plant_count(&self,) -> i8;
    fn measure_moisture_b_hz(&self,plant:i8) -> i16;
    fn measure_moisture_a_hz(&self,plant:i8) -> i16;
    fn measure_moisture_p_hz(&self,plant:i8) -> i16;
    fn pump(&self,plant:i8, enable:bool);
    fn last_pump_time(&self,plant:i8) -> chrono::DateTime<Utc>;
    fn store_last_pump_time(&self,plant:i8, time: chrono::DateTime<Utc>);
    fn store_consecutive_pump_count(&self,plant:i8, count:i16);
    fn consecutive_pump_count(&self,plant:i8) -> i16;

    //keep state during deepsleep
    fn fault(&self,plant:i8, enable:bool);
    
    fn default() -> Self;
}

trait Plant{
    fn setPump(pump:bool);
    

}

struct PlantHal<'d>{
    pump:ShiftRegisterPin<'d>
}

struct PlantCtrlBoard{
    dummy:i32
}

impl PlantCtrlBoardInteraction for PlantCtrlBoard {
    fn default() -> Self {
        let peripherals = Peripherals::take().unwrap(); 

        let mut adc = AdcDriver::new(peripherals.adc1, &Config::new().calibration(true)).unwrap();
        let mut adc_pin: esp_idf_hal::adc::AdcChannelDriver<{ attenuation::DB_11 }, _> = AdcChannelDriver::new(peripherals.pins.gpio39).unwrap();
        let analog_value = adc.read(&mut adc_pin);
        
        let clock = OldOutputPin::from(PinDriver::output(peripherals.pins.gpio21).unwrap());
        let latch = OldOutputPin::from(PinDriver::output(peripherals.pins.gpio22).unwrap());
        let data = OldOutputPin::from(PinDriver::output(peripherals.pins.gpio19).unwrap());

        
        let shift_register = ShiftRegister24::new(clock, latch, data);
        let registerOutput = shift_register.decompose();
        


        Self { dummy: 12 }
    }

    fn battery_state(&self,) -> BatteryState {
        todo!()
    }

    fn is_day(&self,) -> bool {
        todo!()
    }

    fn water_temperature_c(&self,) -> u16 {
        todo!()
    }

    fn tank_sensor_mv(&self,) -> u16 {
        todo!()
    }

    fn set_low_voltage_in_cycle(&self,) {
        unsafe {
            LOW_VOLTAGE_DETECTED = true;
        }
    }

    fn clear_low_voltage_in_cycle(&self,) {
        unsafe {
            LOW_VOLTAGE_DETECTED = false;
        }
    }

    fn light(&self,enable:bool) {
        todo!()
    }

    fn plant_count(&self,) -> i8 {
        todo!()
    }

    fn measure_moisture_b_hz(&self,plant:i8) -> i16 {
        todo!()
    }

    fn measure_moisture_a_hz(&self,plant:i8) -> i16 {
        todo!()
    }

    fn measure_moisture_p_hz(&self,plant:i8) -> i16 {
        todo!()
    }

    fn pump(&self,plant:i8, enable:bool) {
        todo!()
    }

    fn last_pump_time(&self,plant:i8) -> chrono::DateTime<Utc> {
        todo!()
    }

    fn store_last_pump_time(&self,plant:i8, time: chrono::DateTime<Utc>) {
        todo!()
    }

    fn store_consecutive_pump_count(&self,plant:i8, count:i16) {
        todo!()
    }

    fn consecutive_pump_count(&self,plant:i8) -> i16 {
        todo!()
    }

    fn fault(&self,plant:i8, enable:bool) {
        todo!()
    }

    fn low_voltage_in_cycle(&self) -> bool {
        unsafe {
            return LOW_VOLTAGE_DETECTED;
        }
    }


}






fn main() {

    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello, world!");


    let reasons = ResetReason::get();
    //init,reset rtc memory depending on cause

    let board = PlantCtrlBoard::default();

    //check if we know the time current > 2020
        //if failed assume its 1.1.1970 
            //12:00 if solar reports day
            //00:00 if solar repors night
    
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
    //check which plants need to be watered 
        //()

    //if tank sensor is enabled
        //if tank sensor fault abort if config require is set
        //check if water is > minimum allowed || fault
            //if not, set all plants requiring water to persistent fault
        // pump water for first plant update last water timestamp
        // wait for config time per plant
        //




}

//error codes
//error_reading_config_after_upgrade
//error_no_config_after_upgrade
//error_tank_sensor_fault