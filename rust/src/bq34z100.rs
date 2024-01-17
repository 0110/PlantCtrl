use embedded_hal::blocking::i2c::{WriteRead, Write, Read};

const BQ34Z100_G1_ADDRESS:u8 = 0x55;

//
//  bq34z100g1.cpp
//  SMC
//
//  Created by Empire-Phoenix,
//  directly ported from 
//  https://github.com/xkam1x/BQ34Z100G1/blob/master/bq34z100g1.cpp by Kamran Ahmad on 08/05/2019.
//

impl <I2C, E: std::fmt::Debug> Bq34z100g1 for Bq34z100g1Driver<I2C> where I2C: WriteRead<Error = E> + Write<Error = E> + Read<Error = E> { 
    fn read_register(&mut self, address:u8 , length:u8) -> u16 {
        let data: [u8;1] = [address];
        self.i2c.write(BQ34Z100_G1_ADDRESS, &data).unwrap();
        if length != 1 && length != 2{
            todo!();
        }
        if length == 2 {
            let mut buffer : [u8;2] = [0_u8,0_u8];
            let _ = self.i2c.read(BQ34Z100_G1_ADDRESS, &mut buffer);
            return ((buffer[1] as u16) << 8) | buffer[0] as u16;
        } else {
            let mut buffer : [u8;1] = [0_u8];
            let _ = self.i2c.read(BQ34Z100_G1_ADDRESS, &mut buffer);
            return buffer[0] as u16;
        }
        
        // Wire.beginTransmission(BQ34Z100_G1_ADDRESS);
        // Wire.write(address);
        // Wire.endTransmission(false);
        // Wire.requestFrom(BQ34Z100_G1_ADDRESS, length, true);
        
        // uint16_t temp = 0;
        // for (uint8_t i = 0; i < length; i++) {
        //     temp |= Wire.read() << (8 * i);
        // }
        // return temp;
    }

    fn read_control(&mut self,address_lsb:u8, address_msb: u8) -> u16 {
        let data: [u8;3] = [0x00_u8, address_lsb, address_msb];
        self.i2c.write(BQ34Z100_G1_ADDRESS, &data).unwrap();
        return self.read_register(0x00, 2);
        // Wire.beginTransmission(BQ34Z100_G1_ADDRESS);
        // Wire.write(0x00);
        // Wire.write(address_lsb);
        // Wire.write(address_msb);
        // Wire.endTransmission(true);
        // return read_register(0x00, 2);
    }

    fn read_flash_block(sub_class:u8, offset:u8) {
        todo!()
    }

    fn write_reg(address:u8, value:u8 ) {
        todo!()
    }

    fn write_flash_block(sub_class:u8 ,  offset:u8) {
        todo!()
    }

    fn flash_block_checksum() -> u8 {
        todo!()
    }

    fn xemics_to_f64( value: u32) -> f64  {
        todo!()
    }

    fn f64_to_xemics( value:f64) -> u32  {
        todo!()
    }

    fn unsealed() {
        todo!()
    }

    fn enter_calibration() {
        todo!()
    }

    fn exit_calibration() {
        todo!()
    }

    fn update_design_capacity(capacity:u16) -> bool {
        todo!()
    }

    fn update_q_max(capacity:i16) -> bool  {
        todo!()
    }

    fn update_design_energy(energy:i16) -> bool  {
        todo!()
    }

    fn update_cell_charge_voltage_range(t1_t2:u16, t2_t3:u16, t3_t4:u16)-> bool  {
        todo!()
    }

    fn update_number_of_series_cells(cells:u8)-> bool  {
        todo!()
    }

    fn update_pack_configuration(config:u16) -> bool  {
        todo!()
    }

    fn update_charge_termination_parameters(taper_current:i16, min_taper_capacity:i16, cell_taper_voltage:i16, 
        taper_window:u8, tca_set:i8, tca_clear:i8, fc_set:i8, fc_clear:i8) -> bool  {
        todo!()
    }

    fn calibrate_cc_offset() {
        todo!()
    }

    fn calibrate_board_offset() {
        todo!()
    }

    fn calibrate_voltage_divider(applied_voltage:u16,  cells_count:u8) {
        todo!()
    }

    fn calibrate_sense_resistor(applied_current:i16) {
        todo!()
    }

    fn set_current_deadband( deadband:u8) {
        todo!()
    }

    fn ready() {
        todo!()
    }

    fn control_status() -> u16 {
        todo!()
    }

    fn device_type() ->u16 {
        todo!()
    }

    fn fw_version(&mut self)->u16 {
        return self.read_control(0x02, 0x00);
    }

    fn hw_version()->u16 {
        todo!()
    }

    fn reset_data() -> u16  {
        todo!()
    }

    fn prev_macwrite() -> u16  {
        todo!()
    }

    fn chem_id() -> u16  {
        todo!()
    }

    fn board_offset() -> u16  {
        todo!()
    }

    fn cc_offset() -> u16  {
        todo!()
    }

    fn cc_offset_save() -> u16  {
        todo!()
    }

    fn df_version() -> u16  {
        todo!()
    }

    fn set_fullsleep() -> u16  {
        todo!()
    }

    fn static_chem_chksum() -> u16  {
        todo!()
    }

    fn sealed() -> u16  {
        todo!()
    }

    fn it_enable() -> u16  {
        todo!()
    }

    fn cal_enable() -> u16  {
        todo!()
    }

    fn reset() -> u16  {
        todo!()
    }

    fn exit_cal() -> u16  {
        todo!()
    }

    fn enter_cal() -> u16  {
        todo!()
    }

    fn offset_cal() -> u16  {
        todo!()
    }

    fn state_of_charge() -> u8  {
        todo!()
    }

    fn state_of_charge_max_error() -> u8  {
        todo!()
    }

    fn remaining_capacity() -> u16  {
        todo!()
    }

    fn full_charge_capacity() -> u16  {
        todo!()
    }

    fn voltage() -> u16  {
        todo!()
    }

    fn average_current() -> i16 {
        todo!()
    }

    fn temperature() -> u16  {
        todo!()
    }

    fn flags() -> u16  {
        todo!()
    }

    fn flags_b() -> u16  {
        todo!()
    }

    fn current() -> i16 {
        todo!()
    }

    fn average_time_to_empty() -> u16 {
        todo!()
    }

    fn average_time_to_full() -> u16 {
        todo!()
    }

    fn passed_charge() -> i16 {
        todo!()
    }

    fn do_d0_time() -> u16  {
        todo!()
    }

    fn available_energy() -> u16  {
        todo!()
    }

    fn average_power() -> u16  {
        todo!()
    }

    fn serial_number() -> u16  {
        todo!()
    }

    fn internal_temperature(&mut self) -> u16  {
        return self.read_register(0x2a, 2);
    }

    fn cycle_count() -> u16  {
        todo!()
    }

    fn state_of_health() -> u16  {
        todo!()
    }

    fn charge_voltage() -> u16  {
        todo!()
    }

    fn charge_current() -> u16 {
        todo!()
    }

    fn pack_configuration() -> u16  {
        todo!()
    }

    fn design_capacity() -> u16  {
        todo!()
    }

    fn grid_number() -> u8  {
        todo!()
    }

    fn learned_status() -> u8  {
        todo!()
    }

    fn dod_at_eoc() -> u16  {
        todo!()
    }

    fn  q_start() -> u16 {
        todo!()
    }

    fn  true_fcc() -> u16 {
        todo!()
    }

    fn state_time() -> u16  {
        todo!()
    }

    fn  q_max_passed_q() -> u16 {
        todo!()
    }

    fn  dod_0() -> u16 {
        todo!()
    }

    fn  q_max_dod_0() -> u16 {
        todo!()
    }

    fn  q_max_time() -> u16 {
        todo!()
    }
}

pub struct Bq34z100g1Driver<I2C>{
    pub i2c: I2C,
    pub flash_block_data: [u8;32],
}
pub trait Bq34z100g1 {
    fn read_register(&mut self, address:u8 , length:u8) -> u16;
    fn read_control(&mut self, address_lsb:u8, address_msb: u8) -> u16;
    fn read_flash_block(sub_class:u8, offset:u8);
    fn write_reg(address:u8, value:u8 );
    fn write_flash_block(sub_class:u8 ,  offset:u8);
    
    fn flash_block_checksum() -> u8;
    
    fn xemics_to_f64( value: u32) -> f64 ; 
    fn f64_to_xemics( value:f64) -> u32 ;
    
    fn unsealed();
    fn enter_calibration();
    fn exit_calibration();

    fn update_design_capacity(capacity:u16) -> bool;
    fn update_q_max(capacity:i16) -> bool ;
    fn update_design_energy(energy:i16) -> bool ;
    fn update_cell_charge_voltage_range(t1_t2:u16, t2_t3:u16, t3_t4:u16)-> bool ;
    fn update_number_of_series_cells(cells:u8)-> bool ;
    fn update_pack_configuration(config:u16) -> bool ;
    fn update_charge_termination_parameters(taper_current:i16, min_taper_capacity:i16, cell_taper_voltage:i16, 
        taper_window:u8, tca_set:i8, tca_clear:i8, fc_set:i8, fc_clear:i8) -> bool ;
    fn calibrate_cc_offset();
    fn calibrate_board_offset();
    fn calibrate_voltage_divider(applied_voltage:u16,  cells_count:u8);
    fn calibrate_sense_resistor(applied_current:i16);
    fn set_current_deadband( deadband:u8);
    fn ready();
    
    fn control_status() -> u16;
    fn device_type() ->u16;
    fn fw_version(&mut self)->u16;
    fn hw_version()->u16;
    fn reset_data() -> u16 ;
    fn prev_macwrite() -> u16 ;
    fn chem_id() -> u16 ;
    fn board_offset() -> u16 ;
    fn cc_offset() -> u16 ;
    fn cc_offset_save() -> u16 ;
    fn df_version() -> u16 ;
    fn set_fullsleep() -> u16 ;
    fn static_chem_chksum() -> u16 ;
    fn sealed() -> u16 ;
    fn it_enable() -> u16 ;
    fn cal_enable() -> u16 ;
    fn reset() -> u16 ;
    fn exit_cal() -> u16 ;
    fn enter_cal() -> u16 ;
    fn offset_cal() -> u16 ;
    
    fn state_of_charge() -> u8 ; // 0 to 100%
    fn state_of_charge_max_error() -> u8 ; // 1 to 100%
    fn remaining_capacity() -> u16 ; // mAh
    fn full_charge_capacity() -> u16 ; // mAh
    fn voltage() -> u16 ; // mV
    fn average_current() -> i16; // mA
    fn temperature() -> u16 ; // Unit of x10 K
    fn flags() -> u16 ;
    fn flags_b() -> u16 ;
    fn current() -> i16; // mA
    
    fn average_time_to_empty() -> u16; // Minutes
    fn average_time_to_full() -> u16; // Minutes
    fn passed_charge() -> i16; // mAh
    fn do_d0_time() -> u16 ; // Minutes
    fn available_energy() -> u16 ; // 10 mWh
    fn average_power() -> u16 ; // 10 mW
    fn serial_number() -> u16 ;
    fn internal_temperature(&mut self) -> u16 ; // Unit of x10 K
    fn cycle_count() -> u16 ; // Counts
    fn state_of_health() -> u16 ; // 0 to 100%
    fn charge_voltage() -> u16 ; // mV
    fn charge_current() -> u16; // mA
    fn pack_configuration() -> u16 ;
    fn design_capacity() -> u16 ; // mAh
    fn grid_number() -> u8 ;
    fn learned_status() -> u8 ;
    fn dod_at_eoc() -> u16 ;
    fn  q_start() -> u16; // mAh
    fn  true_fcc() -> u16; // mAh
    fn state_time() -> u16 ; // s
    fn  q_max_passed_q() -> u16; // mAh
    fn  dod_0() -> u16;
    fn  q_max_dod_0() -> u16;
    fn  q_max_time() -> u16;
}