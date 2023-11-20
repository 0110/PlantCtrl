use chrono::Utc;
use embedded_hal::digital::v1_compat::OldOutputPin;
use esp_idf_hal::adc::config::Config;
use esp_idf_hal::adc::{AdcDriver, AdcChannelDriver, attenuation};
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
    fn battery_state() -> BatteryState;
    
    fn is_day() -> bool;
    fn water_temperature_c() -> u16;
    fn tank_sensor_mv() -> u16;
    
    fn set_low_voltage_in_cycle();
    fn clear_low_voltage_in_cycle();
    fn low_voltage_in_cycle(low_voltage:bool);


    //keep state during deepsleep
    fn light(enable:bool);

    fn plant_count() -> i8;
    fn measure_moisture_b_hz(plant:i8) -> i16;
    fn measure_moisture_a_hz(plant:i8) -> i16;
    fn measure_moisture_p_hz(plant:i8) -> i16;
    fn pump(plant:i8, enable:bool);
    fn last_pump_time(plant:i8) -> chrono::DateTime<Utc>;
    fn store_last_pump_time(plant:i8, time: chrono::DateTime<Utc>);
    fn store_consecutive_pump_count(plant:i8, count:i16);
    fn consecutive_pump_count(plant:i8) -> i16;

    //keep state during deepsleep
    fn fault(plant:i8, enable:bool);
    
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

    fn plant_count() -> i8 {
        todo!()
    }

    fn fault(plant:i8, state:bool) {
        todo!()
    }

    fn pump(plant:i8, state:bool) {
        todo!()
    }

    fn battery_state() -> BatteryState {
        todo!()
    }

    fn is_day() -> bool {
        todo!()
    }

    fn water_temperature_c() -> u16 {
        todo!()
    }

    fn tank_sensor_mv() -> u16 {
        todo!()
    }

    fn light(enable:bool) {
        todo!()
    }

    fn measure_moisture_b_hz(plant:i8) -> i16 {
        todo!()
    }

    fn measure_moisture_a_hz(plant:i8) -> i16 {
        todo!()
    }

    fn measure_moisture_p_hz(plant:i8) -> i16 {
        todo!()
    }

    fn last_watering_time(plant:i8) -> chrono::DateTime<Utc> {
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

}
