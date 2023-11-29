use embedded_svc::wifi::{Configuration, ClientConfiguration, AuthMethod};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::EspWifi;

use std::sync::Mutex;
use anyhow::{Context, Result, bail};
use anyhow::anyhow;

use chrono::{Utc, NaiveDateTime, DateTime};
use ds18b20::Ds18b20;
use embedded_hal::digital::v1_compat::OldOutputPin;
use embedded_hal::digital::v2::OutputPin;
use esp_idf_hal::adc::config::Config;
use esp_idf_hal::adc::{AdcDriver, AdcChannelDriver, attenuation};
use esp_idf_hal::delay::Delay;
use esp_idf_hal::pcnt::{PcntDriver, PcntChannel, PinIndex, PcntChannelConfig, PcntControlMode, PcntCountMode};
use esp_idf_hal::reset::ResetReason;
use esp_idf_svc::sntp::{self, SyncStatus};
use esp_idf_svc::systime::EspSystemTime;
use esp_idf_sys::EspError;
use one_wire_bus::OneWire;
use shift_register_driver::sipo::ShiftRegister24;
use esp_idf_hal::gpio::{PinDriver, Gpio39, Gpio4, AnyInputPin};
use esp_idf_hal::prelude::Peripherals;

pub const PLANT_COUNT:usize = 8;
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


pub struct BatteryState {
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

pub enum Sensor{
    A,
    B,
    PUMP
}
pub trait PlantCtrlBoardInteraction{
    fn time(&mut self) -> Result<chrono::DateTime<Utc>>;
    fn wifi(&mut self, ssid:&str, password:Option<&str>, max_wait:u32) -> Result<()>;
    fn sntp(&mut self, max_wait:u32) -> Result<chrono::DateTime<Utc>>;

    fn battery_state(&mut self) -> Result<BatteryState>;

    fn general_fault(&mut self, enable: bool);
    
    fn is_day(&self,) -> bool;
    fn water_temperature_c(&mut self,) -> Result<f32>;
    fn tank_sensor_mv(&mut self,) -> Result<u16>;

    fn set_low_voltage_in_cycle(&mut self,);
    fn clear_low_voltage_in_cycle(&mut self,);
    fn low_voltage_in_cycle(&mut self) -> bool;
    fn any_pump(&mut self, enabled:bool)  -> Result<()>;

    //keep state during deepsleep
    fn light(&mut self,enable:bool) -> Result<()>;

    fn measure_moisture_hz(&self, plant:usize, sensor:Sensor) -> Result<i32>;
    fn pump(&self,plant:usize, enable:bool) -> Result<()>;
    fn last_pump_time(&self,plant:usize) -> Result<chrono::DateTime<Utc>>;
    fn store_last_pump_time(&mut self,plant:usize, time: chrono::DateTime<Utc>);
    fn store_consecutive_pump_count(&mut self,plant:usize, count:u32);
    fn consecutive_pump_count(&mut self,plant:usize) -> u32;

    //keep state during deepsleep
    fn fault(&self,plant:usize, enable:bool);
}

pub trait CreatePlantHal<'a> {
    fn create()-> Result<PlantCtrlBoard<'static>>;
}

pub struct PlantHal {

}



impl CreatePlantHal<'_> for PlantHal{
    fn create() -> Result<PlantCtrlBoard<'static>> {
        let peripherals = Peripherals::take()?; 
        
        let clock = OldOutputPin::from(PinDriver::output(peripherals.pins.gpio21)?);
        let latch = OldOutputPin::from(PinDriver::output(peripherals.pins.gpio22)?);
        let data = OldOutputPin::from(PinDriver::output(peripherals.pins.gpio19)?);

        let one_wire_pin = PinDriver::input_output_od(peripherals.pins.gpio4)?;
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

        let mut counter_unit1 = PcntDriver::new(
            peripherals.pcnt0,
            Some(peripherals.pins.gpio18),
            Option::<AnyInputPin>::None,
            Option::<AnyInputPin>::None,
            Option::<AnyInputPin>::None,
        )?;

        counter_unit1.channel_config(
            PcntChannel::Channel0,
            PinIndex::Pin0,
            PinIndex::Pin1,
            &PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse,
                hctrl_mode: PcntControlMode::Keep,
                pos_mode: PcntCountMode::Decrement,
                neg_mode: PcntCountMode::Increment,
                counter_h_lim: i16::MAX,
                counter_l_lim: 0,
            },
        )?;

        //TODO validate filter value! currently max allowed value
        counter_unit1.set_filter_value(1023)?;
        counter_unit1.filter_enable()?;


        
        let sys_loop = EspSystemEventLoop::take()?;
        let nvs = EspDefaultNvsPartition::take()?;
         let wifi_driver = EspWifi::new(
            peripherals.modem,
            sys_loop,
            Some(nvs)
        )?; 

        return Ok(PlantCtrlBoard { 
            shift_register : ShiftRegister24::new(clock, latch, data),
            last_watering_timestamp : Mutex::new(unsafe { LAST_WATERING_TIMESTAMP }),
            consecutive_watering_plant : Mutex::new(unsafe { CONSECUTIVE_WATERING_PLANT }),
            low_voltage_detected : Mutex::new(unsafe { LOW_VOLTAGE_DETECTED }),
            tank_driver : AdcDriver::new(peripherals.adc1, &Config::new().calibration(true))?,
            tank_channel: AdcChannelDriver::new(peripherals.pins.gpio39)?,
            solar_is_day : PinDriver::input(peripherals.pins.gpio25)?,
            light: PinDriver::output(peripherals.pins.gpio26)?,
            main_pump: PinDriver::output(peripherals.pins.gpio23)?,
            tank_power: PinDriver::output(peripherals.pins.gpio27)?,
            general_fault: PinDriver::output(peripherals.pins.gpio13)?,
            one_wire_bus: OneWire::new(one_wire_pin).map_err(|err| -> anyhow::Error {anyhow!("Missing attribute: {:?}", err)})?,
            signal_counter : counter_unit1,
            wifi_driver : wifi_driver
        });
    }
}


pub struct PlantCtrlBoard<'a>{
    shift_register: ShiftRegister24<OldOutputPin<PinDriver<'a, esp_idf_hal::gpio::Gpio21, esp_idf_hal::gpio::Output>>, OldOutputPin<PinDriver<'a, esp_idf_hal::gpio::Gpio22, esp_idf_hal::gpio::Output>>, OldOutputPin<PinDriver<'a, esp_idf_hal::gpio::Gpio19, esp_idf_hal::gpio::Output>>>,
    consecutive_watering_plant: Mutex<[u32; PLANT_COUNT]>,
    last_watering_timestamp: Mutex<[i64; PLANT_COUNT]>,
    low_voltage_detected: Mutex<bool>,
    tank_driver: AdcDriver<'a, esp_idf_hal::adc::ADC1>,
    tank_channel: esp_idf_hal::adc::AdcChannelDriver<'a, { attenuation::DB_11 }, Gpio39 >,
    solar_is_day: PinDriver<'a, esp_idf_hal::gpio::Gpio25, esp_idf_hal::gpio::Input>,
    signal_counter: PcntDriver<'a>,
    light: PinDriver<'a, esp_idf_hal::gpio::Gpio26, esp_idf_hal::gpio::Output>,
    main_pump: PinDriver<'a, esp_idf_hal::gpio::Gpio23, esp_idf_hal::gpio::Output>,
    tank_power: PinDriver<'a, esp_idf_hal::gpio::Gpio27, esp_idf_hal::gpio::Output>,
    general_fault: PinDriver<'a, esp_idf_hal::gpio::Gpio13, esp_idf_hal::gpio::Output>,
    wifi_driver: EspWifi<'a>,
    one_wire_bus: OneWire<PinDriver<'a, Gpio4, esp_idf_hal::gpio::InputOutput>>,
}

impl PlantCtrlBoardInteraction for PlantCtrlBoard<'_> {
    fn battery_state(&mut self,) -> Result<BatteryState> {
        todo!()
    }

    fn is_day(&self,) -> bool {
        return self.solar_is_day.get_level().into();
    }

    fn water_temperature_c(&mut self,) -> Result<f32> {
        let mut delay = Delay::new_default();

        self.one_wire_bus.reset(&mut delay).map_err(|err| -> anyhow::Error {anyhow!("Missing attribute: {:?}", err)})?;
        let first = self.one_wire_bus.devices(false, &mut delay).next();
        if first.is_none() {
            bail!("Not found any one wire  Ds18b20");
        }
        let device_address = first.unwrap().map_err(|err| -> anyhow::Error {anyhow!("Missing attribute: {:?}", err)})?;

        let water_temp_sensor = Ds18b20::new::<EspError>(device_address).map_err(|err| -> anyhow::Error {anyhow!("Missing attribute: {:?}", err)})?;

        water_temp_sensor.start_temp_measurement(&mut self.one_wire_bus, &mut delay).map_err(|err| -> anyhow::Error {anyhow!("Missing attribute: {:?}", err)})?;
        ds18b20::Resolution::Bits12.delay_for_measurement_time(&mut delay);    
        let sensor_data  = water_temp_sensor.read_data(&mut self.one_wire_bus, &mut delay).map_err(|err| -> anyhow::Error {anyhow!("Missing attribute: {:?}", err)})?;
        if sensor_data.temperature == 85_f32 {
            bail!("Ds18b20 dummy temperature returned");
        }
        return Ok(sensor_data.temperature);
    }

    fn tank_sensor_mv(&mut self,) -> Result<u16> {
        let delay = Delay::new_default();
        self.tank_power.set_high()?;
        //let stabilize
        delay.delay_ms(100);
        let value = self.tank_driver.read(&mut self.tank_channel)?;
        self.tank_power.set_low()?;
        return Ok(value);
    }

    fn set_low_voltage_in_cycle(&mut self,) {
        *self.low_voltage_detected.get_mut().unwrap() = true;
    }

    fn clear_low_voltage_in_cycle(&mut self,) {
        *self.low_voltage_detected.get_mut().unwrap() = false;
    }

    fn light(&mut self,enable:bool) -> Result<()>{
        self.light.set_state(enable.into())?;
        Ok(())
    }

    fn pump(&self,plant:usize, enable:bool) -> Result<()> {
        let index = plant*PINS_PER_PLANT*PLANT_PUMP_OFFSET;
        //currently infailable error, keep for future as result anyway
        self.shift_register.decompose()[index].set_state(enable.into()).unwrap();
        Ok(())
    }

    fn last_pump_time(&self,plant:usize) -> Result<chrono::DateTime<Utc>> {
        let ts = unsafe { LAST_WATERING_TIMESTAMP }[plant];
        let timestamp = NaiveDateTime::from_timestamp_millis(ts).ok_or(anyhow!("could not convert timestamp"))?;
        return Ok(DateTime::<Utc>::from_naive_utc_and_offset(timestamp, Utc));
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

    fn any_pump(&mut self, enable:bool) -> Result<()> {
        return Ok(self.main_pump.set_state(enable.into()).unwrap());
    }

    fn time(&mut self) -> Result<chrono::DateTime<Utc>> {
        let time = EspSystemTime{}.now().as_millis();
        let smaller_time = time as i64;
        let local_time = NaiveDateTime::from_timestamp_millis(smaller_time).ok_or(anyhow!("could not convert timestamp"))?;
        return Ok(local_time.and_utc());
    }

    fn sntp(&mut self, max_wait_ms:u32) -> Result<chrono::DateTime<Utc>> {
        let sntp = sntp::EspSntp::new_default()?;
        let mut counter = 0;
        while sntp.get_sync_status() != SyncStatus::Completed{
            let delay = Delay::new_default();
            delay.delay_ms(100);
            counter += 100;
            if counter > max_wait_ms {
                bail!("Reached sntp timeout, aborting")
            }
        }

        return self.time();
    }

    fn measure_moisture_hz(&self, plant:usize, sensor:Sensor) -> Result<i32> {
        self.signal_counter.counter_pause()?;
        self.signal_counter.counter_clear()?;
        //
        let offset = match sensor {
            Sensor::A => PLANT_MOIST_A_OFFSET,
            Sensor::B => PLANT_MOIST_B_OFFSET,
            Sensor::PUMP => PLANT_MOIST_PUMP_OFFSET,
        };
        let index = plant*PINS_PER_PLANT*offset;
        
        let delay = Delay::new_default();
        let measurement = 100;
        let factor = 1000/100;

        self.shift_register.decompose()[index].set_high().unwrap();
        //give some time to stabilize
        delay.delay_ms(10);
        self.signal_counter.counter_resume()?;
        delay.delay_ms(measurement);
        self.signal_counter.counter_pause()?;
        self.shift_register.decompose()[index].set_low().unwrap();
        let unscaled = self.signal_counter.get_counter_value()? as i32;
        return Ok(unscaled*factor);
    }

    fn general_fault(&mut self, enable:bool) {
        self.general_fault.set_state(enable.into()).unwrap();
    }

    fn wifi(&mut self, ssid:&str, password:Option<&str>,max_wait:u32) -> Result<()> {
        match password{
            Some(pw) => {
                //TODO expect error due to invalid pw or similar! //call this during configuration and check if works, revert to config mode if not
                self.wifi_driver.set_configuration(&Configuration::Client(ClientConfiguration{
                    ssid: ssid.into(),
                    password: pw.into(),
                    ..Default::default()
                }))?;
            },
            None => {
                self.wifi_driver.set_configuration(&Configuration::Client(ClientConfiguration {
                    ssid: ssid.into(),
                    auth_method: AuthMethod::None,
                    ..Default::default()
                })).unwrap();
            },
        }

        self.wifi_driver.start().unwrap();
        self.wifi_driver.connect().unwrap();
        
        let delay = Delay::new_default();
        let mut counter = 0_u32;
        while !self.wifi_driver.is_connected().unwrap(){
            let config = self.wifi_driver.get_configuration().unwrap();
            println!("Waiting for station {:?}", config);
            //TODO blink status?
            delay.delay_ms(250);
            counter += 250;
            if counter > max_wait {
                //ignore these errors, wifi will not be used this
                self.wifi_driver.disconnect().unwrap_or(());
                self.wifi_driver.stop().unwrap_or(());
                bail!("Did not manage wifi connection within timeout");
            }
        }
        println!("Should be connected now");
        let address = self.wifi_driver.sta_netif().get_ip_info().unwrap();
        println!("IP info: {:?}", address);
        return Ok(());
    }


} 