use anyhow::{Context, Error};
use esp_idf_hal::gpio::AnyInputPin;
use esp_idf_hal::pcnt::{PcntChannel, PcntChannelConfig, PcntControlMode, PcntCountMode, PcntDriver, PinIndex};
use esp_idf_hal::peripherals::Peripherals;

#[link_section = ".rtc.data"]
static mut MY_FANCY_VARIABLE: u32 = 0;


fn read_moisture() -> Result<i16,Error> {

    let p = Peripherals::take().unwrap();

    let counter1 = p.pcnt0;
    let mut counter_unit1 = PcntDriver::new(
        counter1,
        Some(p.pins.gpio19),
        Option::<AnyInputPin>::None,
        Option::<AnyInputPin>::None,
        Option::<AnyInputPin>::None,
    ).context("Could not obtain counter unit")?;

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
    ).context("Failed to configure pulse counter")?;

    counter_unit1.set_filter_value(u16::min(10 * 80, 1023))?;
    counter_unit1.filter_enable().context("Failed to enable pulse counter filter")?;

    counter_unit1.counter_pause().context("Failed to pause pulse counter")?;
    counter_unit1.counter_clear().context("Failed to clear pulse counter")?;
    counter_unit1.counter_resume().context("Failed to start pulse counter")?;

    let measurement = 100;
    let waitFor = 1000/100;
    //delay(measurement);

    counter_unit1.counter_pause().context("Failed to end pulse counter measurement")?;
    return Ok(counter_unit1.get_counter_value().context("Failed to read pulse counter value")?*waitFor);
}
fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello, world!");



}
