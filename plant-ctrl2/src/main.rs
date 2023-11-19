use embedded_hal::digital::v1_compat::OldOutputPin;
use shift_register_driver::sipo::{ShiftRegister24, ShiftRegisterPin};
use esp_idf_hal::gpio::{PinDriver};
use esp_idf_hal::prelude::Peripherals;
use dummy_pin::DummyPin;
use embedded_hal::digital::v2::OutputPin;

#[macro_use]
extern crate shift_register_driver;

trait PlantCtrlBoardInteraction{
    fn measure_moisture() -> i16;
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
    fn measure_moisture() -> i16 {
        //pcnr here
        return 1_i16;
    }


    fn default() -> Self {
        let peripherals = Peripherals::take().unwrap(); 

        let clock = OldOutputPin::from(PinDriver::output(peripherals.pins.gpio21).unwrap());
        let latch = OldOutputPin::from(PinDriver::output(peripherals.pins.gpio22).unwrap());
        let data = OldOutputPin::from(PinDriver::output(peripherals.pins.gpio19).unwrap());

        
        let shift_register = ShiftRegister24::new(clock, latch, data);
        let registerOutput = shift_register.decompose();
        


        Self { dummy: 12 }
    }
}

const PLANT_COUNT:usize = 8;
#[link_section = ".rtc.data"]
static mut LAST_WATERING_TIMESTAMP: [u64; PLANT_COUNT] = [0; PLANT_COUNT];
#[link_section = ".rtc.data"]
static mut CONSECUTIVE_WATERING_PLANT: [u64; PLANT_COUNT] = [0; PLANT_COUNT];
#[link_section = ".rtc.data"]
static mut LOW_VOLTAGE_DETECTED:bool = false;




fn main() {

    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello, world!");


    let board = PlantCtrlBoard::default();

}
