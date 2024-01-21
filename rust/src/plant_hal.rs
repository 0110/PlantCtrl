//mod config;

use bit_field::BitField;
use embedded_hal::blocking::i2c::Operation;
use embedded_svc::wifi::{
    AccessPointConfiguration, AccessPointInfo, AuthMethod, ClientConfiguration, Configuration,
};

use esp_idf_hal::i2c::{I2cConfig, I2cDriver, APBTickType};
use esp_idf_hal::units::FromValueType;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::mqtt::client::QoS::ExactlyOnce;
use esp_idf_svc::mqtt::client::{EspMqttClient, MqttClientConfiguration};
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::config::{ScanConfig, ScanType};
use esp_idf_svc::wifi::EspWifi;
use measurements::{Measurement, Temperature};
use plant_ctrl2::sipo::ShiftRegister40;

use anyhow::anyhow;
use anyhow::{bail, Ok, Result};
use serde::{Deserialize, Serialize};
use std::ffi::CString;
use std::fs::File;
use std::path::Path;

use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use chrono::{DateTime, NaiveDateTime, Utc};
use ds18b20::Ds18b20;

use embedded_hal::digital::v2::OutputPin;
use esp_idf_hal::adc::{attenuation, AdcChannelDriver, AdcDriver};
use esp_idf_hal::delay::Delay;
use esp_idf_hal::gpio::{AnyInputPin, Gpio39, Gpio4, Level, PinDriver, Pull, InputOutput};
use esp_idf_hal::pcnt::{
    PcntChannel, PcntChannelConfig, PcntControlMode, PcntCountMode, PcntDriver, PinIndex,
};
use esp_idf_hal::prelude::Peripherals;
use esp_idf_hal::reset::ResetReason;
use esp_idf_svc::sntp::{self, SyncStatus};
use esp_idf_svc::systime::EspSystemTime;
use esp_idf_sys::{vTaskDelay, EspError, esp};
use one_wire_bus::OneWire;

use crate::config::{self, Config, WifiConfig};
use crate::STAY_ALIVE;
use crate::bq34z100::{Bq34z100g1Driver, Bq34z100g1};

pub const PLANT_COUNT: usize = 8;
const PINS_PER_PLANT: usize = 5;
const PLANT_PUMP_OFFSET: usize = 0;
const PLANT_FAULT_OFFSET: usize = 1;
const PLANT_MOIST_PUMP_OFFSET: usize = 2;
const PLANT_MOIST_B_OFFSET: usize = 3;
const PLANT_MOIST_A_OFFSET: usize = 4;

const SPIFFS_PARTITION_NAME: &str = "storage";
const WIFI_CONFIG_FILE: &str = "/spiffs/wifi.cfg";
const CONFIG_FILE: &str = "/spiffs/config.cfg";

#[link_section = ".rtc.data"]
static mut LAST_WATERING_TIMESTAMP: [i64; PLANT_COUNT] = [0; PLANT_COUNT];
#[link_section = ".rtc.data"]
static mut CONSECUTIVE_WATERING_PLANT: [u32; PLANT_COUNT] = [0; PLANT_COUNT];
#[link_section = ".rtc.data"]
static mut LOW_VOLTAGE_DETECTED: bool = false;

#[derive(Serialize, Deserialize, Debug)]
pub struct BatteryState {
    pub state_charge_percent: u8,
    max_error_percent: u8,
    remaining_milli_ampere_hour: u32,
    max_milli_ampere_hour: u32,
    design_milli_ampere_hour: u32,
    voltage_milli_volt: u16,
    average_current_milli_ampere: u16,
    temperature_tenth_kelvin: u32,
    average_time_to_empty_minute: u16,
    average_time_to_full_minute: u16,
    average_discharge_power_cycle_milli_watt: u16,
    cycle_count: u16,
    state_health_percent: u8,
}

impl Default for BatteryState {
    fn default() -> Self {
        BatteryState {
            state_charge_percent: 50,
            max_error_percent: 100,
            remaining_milli_ampere_hour: 100,
            max_milli_ampere_hour: 200,
            design_milli_ampere_hour: 200,
            voltage_milli_volt: 12,
            average_current_milli_ampere: 50,
            temperature_tenth_kelvin: 1337,
            average_time_to_empty_minute: 123,
            average_time_to_full_minute: 123,
            average_discharge_power_cycle_milli_watt: 123,
            cycle_count: 123,
            state_health_percent: 90,
        }
    }
}

pub struct FileSystemSizeInfo {
    pub total_size: usize,
    pub used_size: usize,
    pub free_size: usize,
}

#[derive(strum::Display)]
pub enum ClearConfigType {
    WifiConfig,
    Config,
    None,
}

#[derive(Debug)]
pub enum Sensor {
    A,
    B,
    PUMP,
}
pub trait PlantCtrlBoardInteraction {
    fn time(&mut self) -> Result<chrono::DateTime<Utc>>;
    fn wifi(&mut self, ssid: &str, password: Option<&str>, max_wait: u32) -> Result<()>;
    fn sntp(&mut self, max_wait: u32) -> Result<chrono::DateTime<Utc>>;
    fn mount_file_system(&mut self) -> Result<()>;
    fn file_system_size(&mut self) -> Result<FileSystemSizeInfo>;

    fn battery_state(&mut self) -> Result<BatteryState>;

    fn general_fault(&mut self, enable: bool);

    fn is_day(&self) -> bool;
    fn water_temperature_c(&mut self) -> Result<f32>;
    fn tank_sensor_mv(&mut self) -> Result<u16>;

    fn set_low_voltage_in_cycle(&mut self);
    fn clear_low_voltage_in_cycle(&mut self);
    fn low_voltage_in_cycle(&mut self) -> bool;
    fn any_pump(&mut self, enabled: bool) -> Result<()>;

    //keep state during deepsleep
    fn light(&mut self, enable: bool) -> Result<()>;

    fn measure_moisture_hz(&self, plant: usize, sensor: Sensor) -> Result<i32>;
    fn pump(&self, plant: usize, enable: bool) -> Result<()>;
    fn last_pump_time(&self, plant: usize) -> chrono::DateTime<Utc>;
    fn store_last_pump_time(&mut self, plant: usize, time: chrono::DateTime<Utc>);
    fn store_consecutive_pump_count(&mut self, plant: usize, count: u32);
    fn consecutive_pump_count(&mut self, plant: usize) -> u32;

    //keep state during deepsleep
    fn fault(&self, plant: usize, enable: bool);

    //config
    fn is_config_reset(&mut self) -> bool;
    fn remove_configs(&mut self) -> Result<ClearConfigType>;
    fn get_config(&mut self) -> Result<config::Config>;
    fn set_config(&mut self, wifi: &Config) -> Result<()>;
    fn get_wifi(&mut self) -> Result<config::WifiConfig>;
    fn set_wifi(&mut self, wifi: &WifiConfig) -> Result<()>;
    fn wifi_ap(&mut self) -> Result<()>;
    fn wifi_scan(&mut self) -> Result<Vec<AccessPointInfo>>;
    fn test(&mut self) -> Result<()>;
    fn is_wifi_config_file_existant(&mut self) -> bool;
    fn mqtt(&mut self, config: &Config) -> Result<()>;
}

pub trait CreatePlantHal<'a> {
    fn create() -> Result<Mutex<PlantCtrlBoard<'static>>>;
}

pub struct PlantHal {}

pub struct PlantCtrlBoard<'a> {
    shift_register: ShiftRegister40<
        PinDriver<'a, esp_idf_hal::gpio::Gpio21, InputOutput>,
        PinDriver<'a, esp_idf_hal::gpio::Gpio22, InputOutput>,
        PinDriver<'a, esp_idf_hal::gpio::Gpio19, InputOutput>,
    >,
    consecutive_watering_plant: Mutex<[u32; PLANT_COUNT]>,
    last_watering_timestamp: Mutex<[i64; PLANT_COUNT]>,
    low_voltage_detected: Mutex<bool>,
    tank_driver: AdcDriver<'a, esp_idf_hal::adc::ADC1>,
    tank_channel: esp_idf_hal::adc::AdcChannelDriver<'a, { attenuation::DB_11 }, Gpio39>,
    solar_is_day: PinDriver<'a, esp_idf_hal::gpio::Gpio25, esp_idf_hal::gpio::Input>,
    boot_button: PinDriver<'a, esp_idf_hal::gpio::Gpio0, esp_idf_hal::gpio::Input>,
    signal_counter: PcntDriver<'a>,
    light: PinDriver<'a, esp_idf_hal::gpio::Gpio26, InputOutput>,
    main_pump: PinDriver<'a, esp_idf_hal::gpio::Gpio23, InputOutput>,
    tank_power: PinDriver<'a, esp_idf_hal::gpio::Gpio27, InputOutput>,
    general_fault: PinDriver<'a, esp_idf_hal::gpio::Gpio13, InputOutput>,
    pub wifi_driver: EspWifi<'a>,
    one_wire_bus: OneWire<PinDriver<'a, Gpio4, esp_idf_hal::gpio::InputOutput>>,
    mqtt_client: Option<EspMqttClient<'a>>,
}

impl PlantCtrlBoardInteraction for PlantCtrlBoard<'_> {
    fn battery_state(&mut self) -> Result<BatteryState> {
        Ok(BatteryState::default())
    }

    fn is_day(&self) -> bool {
        self.solar_is_day.get_level().into()
    }

    fn water_temperature_c(&mut self) -> Result<f32> {
        let mut delay = Delay::new_default();

        self.one_wire_bus
            .reset(&mut delay)
            .map_err(|err| -> anyhow::Error { anyhow!("Missing attribute: {:?}", err) })?;
        let first = self.one_wire_bus.devices(false, &mut delay).next();
        if first.is_none() {
            bail!("Not found any one wire  Ds18b20");
        }
        let device_address = first
            .unwrap()
            .map_err(|err| -> anyhow::Error { anyhow!("Missing attribute: {:?}", err) })?;

        let water_temp_sensor = Ds18b20::new::<EspError>(device_address)
            .map_err(|err| -> anyhow::Error { anyhow!("Missing attribute: {:?}", err) })?;

        water_temp_sensor
            .start_temp_measurement(&mut self.one_wire_bus, &mut delay)
            .map_err(|err| -> anyhow::Error { anyhow!("Missing attribute: {:?}", err) })?;
        ds18b20::Resolution::Bits12.delay_for_measurement_time(&mut delay);
        let sensor_data = water_temp_sensor
            .read_data(&mut self.one_wire_bus, &mut delay)
            .map_err(|err| -> anyhow::Error { anyhow!("Missing attribute: {:?}", err) })?;
        if sensor_data.temperature == 85_f32 {
            bail!("Ds18b20 dummy temperature returned");
        }
        Ok(sensor_data.temperature/10_f32)
    }

    fn tank_sensor_mv(&mut self) -> Result<u16> {
        let delay = Delay::new_default();
        self.tank_power.set_high()?;
        //let stabilize
        delay.delay_ms(100);
        let value = self.tank_driver.read(&mut self.tank_channel)?;
        self.tank_power.set_low()?;
        Ok(value)
    }

    fn set_low_voltage_in_cycle(&mut self) {
        *self.low_voltage_detected.get_mut().unwrap() = true;
    }

    fn clear_low_voltage_in_cycle(&mut self) {
        *self.low_voltage_detected.get_mut().unwrap() = false;
    }

    fn light(&mut self, enable: bool) -> Result<()> {
        self.light.set_state(enable.into())?;
        Ok(())
    }

    fn pump(&self, plant: usize, enable: bool) -> Result<()> {
        let index = plant * PINS_PER_PLANT + PLANT_PUMP_OFFSET;
        //currently infailable error, keep for future as result anyway
        self.shift_register.decompose()[index]
            .set_state(enable.into())
            .unwrap();
        Ok(())
    }

    fn last_pump_time(&self, plant: usize) -> chrono::DateTime<Utc> {
        let ts = unsafe { LAST_WATERING_TIMESTAMP }[plant];
        let timestamp = NaiveDateTime::from_timestamp_millis(ts).unwrap();
        DateTime::<Utc>::from_naive_utc_and_offset(timestamp, Utc)
    }

    fn store_last_pump_time(&mut self, plant: usize, time: chrono::DateTime<Utc>) {
        self.last_watering_timestamp.get_mut().unwrap()[plant] = time.timestamp_millis();
    }

    fn store_consecutive_pump_count(&mut self, plant: usize, count: u32) {
        self.consecutive_watering_plant.get_mut().unwrap()[plant] = count;
    }

    fn consecutive_pump_count(&mut self, plant: usize) -> u32 {
        return self.consecutive_watering_plant.get_mut().unwrap()[plant];
    }

    fn fault(&self, plant: usize, enable: bool) {
        let index = plant * PINS_PER_PLANT + PLANT_FAULT_OFFSET;
        self.shift_register.decompose()[index]
            .set_state(enable.into())
            .unwrap()
    }

    fn low_voltage_in_cycle(&mut self) -> bool {
        return *self.low_voltage_detected.get_mut().unwrap();
    }

    fn any_pump(&mut self, enable: bool) -> Result<()> {
        {
            self.main_pump.set_state(enable.into()).unwrap();
            Ok(())
        }
    }

    fn time(&mut self) -> Result<chrono::DateTime<Utc>> {
        let time = EspSystemTime {}.now().as_millis();
        let smaller_time = time as i64;
        let local_time = NaiveDateTime::from_timestamp_millis(smaller_time)
            .ok_or(anyhow!("could not convert timestamp"))?;
        Ok(local_time.and_utc())
    }

    fn sntp(&mut self, max_wait_ms: u32) -> Result<chrono::DateTime<Utc>> {
        let sntp = sntp::EspSntp::new_default()?;
        let mut counter = 0;
        while sntp.get_sync_status() != SyncStatus::Completed {
            let delay = Delay::new_default();
            delay.delay_ms(100);
            counter += 100;
            if counter > max_wait_ms {
                bail!("Reached sntp timeout, aborting")
            }
        }

        self.time()
    }

    fn measure_moisture_hz(&self, plant: usize, sensor: Sensor) -> Result<i32> {
        self.signal_counter.counter_pause()?;
        self.signal_counter.counter_clear()?;
        //
        let offset = match sensor {
            Sensor::A => PLANT_MOIST_A_OFFSET,
            Sensor::B => PLANT_MOIST_B_OFFSET,
            Sensor::PUMP => PLANT_MOIST_PUMP_OFFSET,
        };
        let index = plant * PINS_PER_PLANT + offset;

        let delay = Delay::new_default();
        let measurement = 100;
        let factor = 1000 / 100;

        self.shift_register.decompose()[index].set_high().unwrap();
        //give some time to stabilize
        delay.delay_ms(10);
        self.signal_counter.counter_resume()?;
        delay.delay_ms(measurement);
        self.signal_counter.counter_pause()?;
        self.shift_register.decompose()[index].set_low().unwrap();
        let unscaled = self.signal_counter.get_counter_value()? as i32;
        let hz = unscaled * factor;
        println!("Measuring {:?} @ {} with {}", sensor, plant, hz);
        Ok(hz)
    }

    fn general_fault(&mut self, enable: bool) {
        self.general_fault.set_state(enable.into()).unwrap();
    }

    fn wifi_ap(&mut self) -> Result<()> {
        let apconfig = AccessPointConfiguration {
            ssid: "PlantCtrl".into(),
            auth_method: AuthMethod::None,
            ssid_hidden: false,
            ..Default::default()
        };
        let clientconfig = ClientConfiguration::default();
        self.wifi_driver
            .set_configuration(&Configuration::Mixed(clientconfig, apconfig))?;
        self.wifi_driver.start()?;
        Ok(())
    }

    fn wifi(&mut self, ssid: &str, password: Option<&str>, max_wait: u32) -> Result<()> {
        match password {
            Some(pw) => {
                //TODO expect error due to invalid pw or similar! //call this during configuration and check if works, revert to config mode if not
                self.wifi_driver.set_configuration(&Configuration::Client(
                    ClientConfiguration {
                        ssid: ssid.into(),
                        password: pw.into(),
                        ..Default::default()
                    },
                ))?;
            }
            None => {
                self.wifi_driver
                    .set_configuration(&Configuration::Client(ClientConfiguration {
                        ssid: ssid.into(),
                        auth_method: AuthMethod::None,
                        ..Default::default()
                    }))
                    .unwrap();
            }
        }

        self.wifi_driver.start()?;
        self.wifi_driver.connect()?;

        let delay = Delay::new_default();
        let mut counter = 0_u32;
        while !self.wifi_driver.is_connected()? {
            println!("Waiting for station connection");
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

        while !self.wifi_driver.is_up().unwrap() {
            println!("Waiting for network being up");
            delay.delay_ms(250);
            counter += 250;
            if counter > max_wait {
                //ignore these errors, wifi will not be used this
                self.wifi_driver.disconnect().unwrap_or(());
                self.wifi_driver.stop().unwrap_or(());
                bail!("Did not manage wifi connection within timeout");
            }
        }
        //update freertos registers ;)
        let address = self.wifi_driver.sta_netif().get_ip_info().unwrap();
        println!("IP info: {:?}", address);
        Ok(())
    }

    fn mount_file_system(&mut self) -> Result<()> {
        let base_path = CString::new("/spiffs")?;
        let storage = CString::new(SPIFFS_PARTITION_NAME)?;
        let conf = esp_idf_sys::esp_vfs_spiffs_conf_t {
            base_path: base_path.as_ptr(),
            partition_label: storage.as_ptr(),
            max_files: 2,
            format_if_mount_failed: true,
        };

        unsafe {
            esp_idf_sys::esp!(esp_idf_sys::esp_vfs_spiffs_register(&conf))?;
            Ok(())
        }
    }

    fn file_system_size(&mut self) -> Result<FileSystemSizeInfo> {
        let storage = CString::new(SPIFFS_PARTITION_NAME)?;
        let mut total_size = 0;
        let mut used_size = 0;
        unsafe {
            esp_idf_sys::esp!(esp_idf_sys::esp_spiffs_info(
                storage.as_ptr(),
                &mut total_size,
                &mut used_size
            ))?;
        }
        Ok(FileSystemSizeInfo {
            total_size,
            used_size,
            free_size: total_size - used_size,
        })
    }

    fn is_config_reset(&mut self) -> bool {
        self.boot_button.get_level() == Level::Low
    }

    fn remove_configs(&mut self) -> Result<ClearConfigType> {
        let config = Path::new(CONFIG_FILE);
        if config.exists() {
            println!("Removing config");
            std::fs::remove_file(config)?;
            return Ok(ClearConfigType::Config);
        }

        let wifi_config = Path::new(WIFI_CONFIG_FILE);
        if wifi_config.exists() {
            println!("Removing wifi config");
            std::fs::remove_file(wifi_config)?;
            return Ok(ClearConfigType::WifiConfig);
        }

        Ok(ClearConfigType::None)
    }

    fn get_wifi(&mut self) -> Result<config::WifiConfig> {
        let cfg = File::open(WIFI_CONFIG_FILE)?;
        let config: WifiConfig = serde_json::from_reader(cfg)?;
        Ok(config)
    }

    fn set_wifi(&mut self, wifi: &WifiConfig) -> Result<()> {
        let mut cfg = File::create(WIFI_CONFIG_FILE)?;
        serde_json::to_writer(&mut cfg, &wifi)?;
        println!("Wrote wifi config {}", wifi);
        Ok(())
    }

    fn get_config(&mut self) -> Result<config::Config> {
        let cfg = File::open(CONFIG_FILE)?;
        let config: Config = serde_json::from_reader(cfg)?;
        Ok(config)
    }

    fn set_config(&mut self, config: &Config) -> Result<()> {
        let mut cfg = File::create(CONFIG_FILE)?;
        serde_json::to_writer(&mut cfg, &config)?;
        println!("Wrote config config {:?}", config);
        Ok(())
    }

    fn wifi_scan(&mut self) -> Result<Vec<AccessPointInfo>> {
        //remove this parts
        for i in 1..11 {
            println!("Scanning channel {}", i);
            self.wifi_driver.start_scan(
                &ScanConfig {
                    scan_type: ScanType::Passive(Duration::from_secs(1)),
                    show_hidden: false,
                    channel: Some(i),
                    ..Default::default()
                },
                true,
            )?;
            let sr = self.wifi_driver.get_scan_result()?;
            for r in sr.iter() {
                println!("Found wifi {}", r.ssid);
            }
        }

        self.wifi_driver.start_scan(
            &ScanConfig {
                scan_type: ScanType::Passive(Duration::from_secs(1)),
                show_hidden: false,
                ..Default::default()
            },
            true,
        )?;
        Ok(self.wifi_driver.get_scan_result()?)
    }

    fn test(&mut self) -> Result<()> {
        self.general_fault(true);
        unsafe { vTaskDelay(100) };
        self.general_fault(false);
        unsafe { vTaskDelay(100) };
        self.any_pump(true)?;
        unsafe { vTaskDelay(500) };
        self.any_pump(false)?;
        unsafe { vTaskDelay(500) };
        self.light(true)?;
        unsafe { vTaskDelay(500) };
        self.light(false)?;
        unsafe { vTaskDelay(500) };
        for i in 0..8 {
            self.fault(i, true);
            unsafe { vTaskDelay(500) };
            self.fault(i, false);
            unsafe { vTaskDelay(500) };
        }
        for i in 0..8 {
            self.pump(i, true)?;
            unsafe { vTaskDelay(500) };
            self.pump(i, false)?;
            unsafe { vTaskDelay(500) };
        }
        for i in 0..8 {
            self.measure_moisture_hz(i, Sensor::A)?;
            unsafe { vTaskDelay(500) };
        }
        for i in 0..8 {
            self.measure_moisture_hz(i, Sensor::B)?;
            unsafe { vTaskDelay(500) };
        }
        for i in 0..8 {
            self.measure_moisture_hz(i, Sensor::PUMP)?;
            unsafe { vTaskDelay(500) };
        }

        Ok(())
    }

    fn is_wifi_config_file_existant(&mut self) -> bool {
        let config = Path::new(CONFIG_FILE);
        config.exists()
    }

    fn mqtt(&mut self, config: &Config) -> Result<()> {
        //FIXME testament
        let mqtt_client_config = MqttClientConfiguration {
            //room for improvement
            ..Default::default()
        };

        let round_trip_ok = Arc::new(AtomicBool::new(false));
        let round_trip_topic = format!("{}/internal/roundtrip", config.base_topic);
        let stay_alive_topic = format!("{}/stay_alive", config.base_topic);
        println!("Round trip topic is {}", round_trip_topic);
        println!("Stay alive topic is {}", stay_alive_topic);

        let stay_alive_topic_copy = stay_alive_topic.clone();
        let round_trip_topic_copy = round_trip_topic.clone();
        let round_trip_ok_copy = round_trip_ok.clone();
        let mut client =
            EspMqttClient::new(&config.mqtt_url, &mqtt_client_config, move |handler| {
                match handler {
                    Err(err) => println!("Ignoring damaged message {}", err),
                    core::result::Result::Ok(event) => {
                        match event {
                            embedded_svc::mqtt::client::Event::Received(msg) => {
                                let data = String::from_utf8_lossy(msg.data());
                                if let Some(topic) = msg.topic() {
                                    //todo use enums
                                    if topic.eq(round_trip_topic_copy.as_str()) {
                                        round_trip_ok_copy
                                            .store(true, std::sync::atomic::Ordering::Relaxed);
                                    } else if topic.eq(stay_alive_topic_copy.as_str()) {
                                        let value = data.eq_ignore_ascii_case("true")
                                            || data.eq_ignore_ascii_case("1");
                                        println!("Received stay alive with value {}", value);
                                        STAY_ALIVE
                                            .store(value, std::sync::atomic::Ordering::Relaxed);
                                    } else {
                                        println!("Unknown topic recieved {}", topic);
                                    }
                                }
                            }
                            _ => {}
                        }
                    }
                }
            })?;
        //subscribe to roundtrip

        client.subscribe(round_trip_topic.as_str(), ExactlyOnce)?;
        client.subscribe(stay_alive_topic.as_str(), ExactlyOnce)?;
        //publish to roundtrip
        client.publish(
            round_trip_topic.as_str(),
            ExactlyOnce,
            false,
            "online_test".as_bytes(),
        )?;

        let wait_for_roundtrip = 0;
        while wait_for_roundtrip < 100 {
            match round_trip_ok.load(std::sync::atomic::Ordering::Relaxed) {
                true => {
                    println!("Round trip registered, proceeding");
                    self.mqtt_client = Some(client);
                    return Ok(());
                }
                false => {
                    unsafe { vTaskDelay(10) };
                }
            }
        }
        bail!("Mqtt did not complete roundtrip in time");
    }
}

impl CreatePlantHal<'_> for PlantHal {
    fn create() -> Result<Mutex<PlantCtrlBoard<'static>>> {
        let peripherals = Peripherals::take()?;
                
        let i2c = peripherals.i2c1;
        let config = I2cConfig::new()
        .scl_enable_pullup(false)
        .sda_enable_pullup(false)
        .baudrate(10_u32.kHz().into());
        let scl = peripherals.pins.gpio16;
        let sda = peripherals.pins.gpio17;
        

        let driver = I2cDriver::new(i2c, sda, scl, &config).unwrap();
        let i2c_port = driver.port();
        let mut battery_driver :Bq34z100g1Driver<I2cDriver, Delay> = Bq34z100g1Driver{
            i2c :driver,
            delay: Delay::new_default(),
            flash_block_data : [0;32],
        };       

        let mut clock = PinDriver::input_output(peripherals.pins.gpio21)?;
        clock.set_pull(Pull::Floating);
        let mut latch = PinDriver::input_output(peripherals.pins.gpio22)?;
        latch.set_pull(Pull::Floating);
        let mut data = PinDriver::input_output(peripherals.pins.gpio19)?;
        data.set_pull(Pull::Floating);
        let shift_register = ShiftRegister40::new(clock.into(), latch.into(), data.into());
        for mut pin in shift_register.decompose() {
            pin.set_low().unwrap();
        }

        let mut one_wire_pin = PinDriver::input_output_od(peripherals.pins.gpio4)?;
        one_wire_pin.set_pull(Pull::Floating);
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

        println!("Channel config start");

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

        println!("Setup filter");

        //TODO validate filter value! currently max allowed value
        counter_unit1.set_filter_value(1023)?;
        counter_unit1.filter_enable()?;

        println!("Wifi start");

        let sys_loop = EspSystemEventLoop::take()?;
        let nvs = EspDefaultNvsPartition::take()?;
        let wifi_driver = EspWifi::new(peripherals.modem, sys_loop, Some(nvs))?;


        let last_watering_timestamp = Mutex::new(unsafe { LAST_WATERING_TIMESTAMP });
        let consecutive_watering_plant = Mutex::new(unsafe { CONSECUTIVE_WATERING_PLANT });
        let low_voltage_detected = Mutex::new(unsafe { LOW_VOLTAGE_DETECTED });
        let tank_driver =
            AdcDriver::new(peripherals.adc1, &esp_idf_hal::adc::config::Config::new())?;
        let tank_channel: AdcChannelDriver<'_, { attenuation::DB_11 }, Gpio39> =
            AdcChannelDriver::new(peripherals.pins.gpio39)?;

        let mut solar_is_day = PinDriver::input(peripherals.pins.gpio25)?;
        solar_is_day.set_pull(Pull::Floating)?;

        let mut boot_button = PinDriver::input(peripherals.pins.gpio0)?;
        boot_button.set_pull(Pull::Floating)?;
        let mut light = PinDriver::input_output(peripherals.pins.gpio26)?;
        light.set_pull(Pull::Floating)?;
        let mut main_pump = PinDriver::input_output(peripherals.pins.gpio23)?;
        main_pump.set_pull(Pull::Floating)?;
        main_pump.set_low()?;
        let mut tank_power = PinDriver::input_output(peripherals.pins.gpio27)?;
        tank_power.set_pull(Pull::Floating)?;
        let mut general_fault = PinDriver::input_output(peripherals.pins.gpio13)?;
        general_fault.set_pull(Pull::Floating)?;
        general_fault.set_low()?;
        let one_wire_bus = OneWire::new(one_wire_pin)
            .map_err(|err| -> anyhow::Error { anyhow!("Missing attribute: {:?}", err) })?;

        println!("After stuff");

        esp!(unsafe { esp_idf_sys::i2c_set_timeout(i2c_port, 1048000) }).unwrap();
        
        let fwversion = battery_driver.fw_version();
        println!("fw version is {}", fwversion);

        let design_capacity = battery_driver.design_capacity();
        println!("Design Capacity {}", design_capacity);
        if(design_capacity == 1000){
            println!("Still stock configuring battery");
        }

        //battery_driver.update_design_capacity(5999);


        //let mut success = battery_driver.update_design_capacity(6000);
        //if (!success){
        //    bail!("Error updating capacity");
        //}

        //success = battery_driver.update_q_max(6000);
        //if (!success){
        //    bail!("Error updating max q");
        //}   
                            
        //let energy = 25600;
        //success = battery_driver.update_design_energy(energy, 3);
        //if (!success){
        //    bail!("Error updating design energy");
        //}
        
        //success = battery_driver.update_cell_charge_voltage_range(3650,3650,3650);
        //if (!success){
        //    bail!("Error updating cell charge voltage");
        //}

        //success = battery_driver.update_number_of_series_cells(4);
        //if (!success){
        //    bail!("Error updating number of series");
        //}

        //charge termination here



        // //RESCAP CAL_EN SCALED RSVD VOLTSEL IWAKE RSNS1 RSNS0
        // //RFACTSTEP SLEEP RMFCC NiDT NiDV QPCCLEAR GNDSEL TEMPS
        // let mut conf: u16 = 0;
        // //RESCAP 
        // conf.set_bit(15, true);
        // //CAL_EN 
        // conf.set_bit(14, true);
        // //SCALED 
        // conf.set_bit(13, false);
        // //RSVD 
        // conf.set_bit(12, false);
        // //VOLTSEL 
        // conf.set_bit(11, true);
        // //IWAKE 
        // conf.set_bit(10, false);
        // //RSNS1 
        // conf.set_bit(9, false);
        // //RSNS0
        // conf.set_bit(8, true);

        // //RFACTSTEP 
        // conf.set_bit(7, true);
        // //SLEEP 
        // conf.set_bit(6, true);
        // //RMFCC 
        // conf.set_bit(5, true);
        // //NiDT 
        // conf.set_bit(4, false);
        // //NiDV 
        // conf.set_bit(3, false);
        // //QPCCLEAR 
        // conf.set_bit(2, false);
        // //GNDSEL 
        // conf.set_bit(1, true);
        // //TEMPS
        // conf.set_bit(0, false);



        // let mut success = battery_driver.update_pack_configuration(conf);
        // if (!success){
        //     bail!("Error updating pack config");
        // }

//        let mut success = battery_driver.update_charge_termination_parameters(100, 25, 100, 40, 99, 95, 100, 96);
//        if (!success){
//             bail!("Error updating pack config");
//         }

//calibration here

        //println!("Cc offset");
        //battery_driver.calibrate_cc_offset();
        //println!("board offset");
        //battery_driver.calibrate_board_offset();
        //println!("voltage divider");
        //battery_driver.calibrate_voltage_divider(15000.0, 4);
        
       //battery_driver.calibrate_sense_resistor(1520);

       loop {
            let chem_id = battery_driver.chem_id();
            let bat_temp = battery_driver.temperature();
            let temp_c = Temperature::from_kelvin(bat_temp as f64/10_f64).as_celsius();          
            let voltage = battery_driver.voltage();
            let current = battery_driver.current();
            let state = battery_driver.state_of_charge();
            let charge_voltage = battery_driver.charge_voltage();
            let charge_current = battery_driver.charge_current();
            println!("ChemId: {} Current voltage {} and current {} with charge {}% and temp {} CVolt: {} CCur {}", chem_id, voltage, current, state, temp_c, charge_voltage, charge_current);

            unsafe{
                vTaskDelay(1000);
            }

        }


        let rv = Mutex::new(PlantCtrlBoard {
            shift_register,
            last_watering_timestamp,
            consecutive_watering_plant,
            low_voltage_detected,
            tank_driver,
            tank_channel,
            solar_is_day,
            boot_button,
            light,
            main_pump,
            tank_power,
            general_fault,
            one_wire_bus,
            signal_counter: counter_unit1,
            wifi_driver,
            mqtt_client: None,
        });
        Ok(rv)
    }
}
