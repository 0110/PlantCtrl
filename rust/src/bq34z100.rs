use embedded_hal::blocking::{i2c::{WriteRead, Write, Read}, delay::DelayMs};
use esp_idf_sys::vTaskDelay;

const BQ34Z100_G1_ADDRESS:u8 = 0x55;

//
//  bq34z100g1.cpp
//  SMC
//
//  Created by Empire-Phoenix,
//  directly ported from 
//  https://github.com/xkam1x/BQ34Z100G1/blob/master/bq34z100g1.cpp by Kamran Ahmad on 08/05/2019.
//  Xemics conversion from https://github.com/Ralim/BQ34Z100/blob/master/bq34z100.cpp

fn xemics_to_double(x:u32) -> f32 {
    let mut b_is_positive = false;
    let f_exponent :f32;
    let mut f_result :f32;
    let v_msbyte :u8 = (x >> 24) as u8;
    let mut v_mid_hi_byte :u8 = (x >> 16) as u8;
    let v_mid_lo_byte :u8 = (x >> 8) as u8;
    let v_lsbyte: u8 = x as u8;
    // Get the sign, its in the 0x00 80 00 00 bit
    if (v_mid_hi_byte & 128) == 0 { 
        b_is_positive = true; 
    }

    // Get the exponent, it's 2^(MSbyte - 0x80)
    f_exponent = 2.0_f32.powf(v_msbyte.wrapping_sub(128) as f32);
    // Or in 0x80 to the MidHiByte
    v_mid_hi_byte = (v_mid_hi_byte | 128) as u8;
    // get value out of midhi byte
    f_result = (v_mid_hi_byte as f32) * 65536.0;
    // add in midlow byte
    f_result = f_result + (v_mid_lo_byte as f32 * 256.0) as f32;
    // add in LS byte
    f_result = f_result + v_lsbyte as f32;
    // multiply by 2^-24 to get the actual fraction
    f_result = f_result * 2.0_f32.powf(-24.0);
    // multiply fraction by the ‘exponent’ part
    f_result = f_result * f_exponent;
    // Make negative if necessary
    if b_is_positive {
        return f_result;
    } else {
        return -f_result;
    }
        


        // bool bIsPositive = false;
        // float fExponent, fResult;
        // byte vMSByte = (byte)(X >> 24);
        // byte vMidHiByte = (byte)(X >> 16);
        // byte vMidLoByte = (byte)(X >> 8);
        // byte vLSByte = (byte)X;
        // // Get the sign, its in the 0x00 80 00 00 bit
        // if ((vMidHiByte & 128) == 0)
        // { bIsPositive = true; }

        // // Get the exponent, it's 2^(MSbyte - 0x80)
        // fExponent = pow(2, (vMSByte - 128));
        // // Or in 0x80 to the MidHiByte
        // vMidHiByte = (byte)(vMidHiByte | 128);
        // // get value out of midhi byte
        // fResult = (vMidHiByte) * 65536;
        // // add in midlow byte
        // fResult = fResult + (vMidLoByte * 256);
        // // add in LS byte
        // fResult = fResult + vLSByte;
        // // multiply by 2^-24 to get the actual fraction
        // fResult = fResult * pow(2, -24);
        // // multiply fraction by the ‘exponent’ part
        // fResult = fResult * fExponent;
        // // Make negative if necessary
        // if (bIsPositive)
        //     return fResult;
        // else
        //     return -fResult;
}

fn double_to_xemics(mut x:f32) -> u32 {
    let i_byte1:i16;
    let mut i_byte2:i16;
    let i_byte3: i16;
    let i_byte4: i16;
    let i_exp: i16;
    let mut b_negative = false;
    let mut f_mantissa: f32;
    // Don't blow up with logs of zero
    if x == 0.0 {
        x = 0.00001;
    } 
    if x < 0.0
    {
        b_negative = true;
        x = -x;
    }
    // find the correct exponent
    i_exp = (x.log2() + 1.0) as i16;// remember - log of any base is ln(x)/ln(base)

    // MS byte is the exponent + 0x80
    i_byte1 = i_exp + 128;
   
    // Divide input by this exponent to get mantissa
    f_mantissa = x / (2.0_f32.powf(i_exp as f32));
   
    // Scale it up
    f_mantissa = f_mantissa / (2.0_f32.powf(-24.0));
   
    // Split the mantissa into 3 bytes
    i_byte2 = (f_mantissa / (2.0_f32.powf(16.0))) as i16;
    
    i_byte3 = ((f_mantissa - (i_byte2 as f32 * (2.0_f32.powf(16.0)))) / (2.0_f32.powf(8.0))) as i16;
   
    i_byte4 = (f_mantissa - (i_byte2 as f32 * (2.0_f32.powf(16.0))) - (i_byte3 as f32 * (2.0_f32.powf(8.0)))) as i16;
   
    // subtract the sign bit if number is positive
    if b_negative == false
    {
        i_byte2 = i_byte2 & 0x7F;
    }
    return (i_byte1 as u8 as u32) << 24 | (i_byte2 as u8 as u32) << 16 | (i_byte3 as u8 as u32) << 8 | i_byte4 as u8 as u32;

    // int iByte1, iByte2, iByte3, iByte4, iExp;
    // bool bNegative = false;
    // float fMantissa;
    // // Don't blow up with logs of zero
    // if (X == 0) X = 0.00001F;
    // if (X < 0)
    // {
    //     bNegative = true;
    //     X = -X;
    // }
    // // find the correct exponent
    // iExp = (int)((log(X) / log(2)) + 1);// remember - log of any base is ln(x)/ln(base)

    // // MS byte is the exponent + 0x80
    // iByte1 = iExp + 128;
   
    // // Divide input by this exponent to get mantissa
    // fMantissa = X / (pow(2, iExp));
   
    // // Scale it up
    // fMantissa = fMantissa / (pow(2, -24));
   
    // // Split the mantissa into 3 bytes
    // iByte2 = (int)(fMantissa / (pow(2, 16)));
    
    // iByte3 = (int)((fMantissa - (iByte2 * (pow(2, 16)))) / (pow(2, 8)));
   
    // iByte4 = (int)(fMantissa - (iByte2 * (pow(2, 16))) - (iByte3 * (pow(2, 8))));
   
    // // subtract the sign bit if number is positive
    // if (bNegative == false)
    // {
    //     iByte2 = iByte2 & 0x7F;
    // }
    // return (uint32_t)((uint32_t)iByte1 << 24 | (uint32_t)iByte2 << 16 | (uint32_t)iByte3 << 8 | (uint32_t)iByte4);

}

impl <I2C,DELAY, E: std::fmt::Debug> Bq34z100g1 for Bq34z100g1Driver<I2C,DELAY> where I2C: WriteRead<Error = E> + Write<Error = E> + Read<Error = E>, DELAY: DelayMs<u32> { 
    fn read_register(&mut self, address:u8 , length:u8) -> u16 {
        println!("Reading register block {:#04x} with length {}", address, length);
        let data: [u8;1] = [address];
        if length != 1 && length != 2{
            todo!();
        }
        if length == 2 {
            let mut buffer : [u8;2] = [0_u8,0_u8];
            self.i2c.write_read(BQ34Z100_G1_ADDRESS, &data, &mut buffer).unwrap();
            return ((buffer[1] as u16) << 8) | buffer[0] as u16;
        } else {
            let mut buffer : [u8;1] = [0_u8];
            self.i2c.write_read(BQ34Z100_G1_ADDRESS, &data, &mut buffer).unwrap();
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
        println!("Reading controll {} {}", address_lsb, address_msb);
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

    fn internal_temperature(&mut self) -> u16  {
        return self.read_register(0x2a, 2);
    }

    fn read_flash_block(&mut self, sub_class:u8, offset:u8) {
        println!("Prepare reading block {}", sub_class);
        self.write_reg(0x61, 0x00); // Block control
        self.write_reg(0x3e, sub_class); // Flash class
        self.write_reg(0x3f, offset / 32); // Flash block
        
        let data: [u8;1] = [0x40];
        self.i2c.write(BQ34Z100_G1_ADDRESS, &data).unwrap();
        println!("Reading block {} now", sub_class);
        self.i2c.read(BQ34Z100_G1_ADDRESS, &mut self.flash_block_data).unwrap();

        // write_reg(0x61, 0x00); // Block control
        // write_reg(0x3e, sub_class); // Flash class
        // write_reg(0x3f, offset / 32); // Flash block
        
        // Wire.beginTransmission(BQ34Z100_G1_ADDRESS);
        // Wire.write(0x40); // Block data
        // for (uint8_t i = 0; i < 32; i++) {
        //     Wire.write(flash_block_data[i]); // Data
        // }
        // Wire.endTransmission(true);
    }

    fn write_reg(&mut self, address:u8, value:u8 ) {
        let data: [u8;2] = [address, value];
        self.i2c.write(BQ34Z100_G1_ADDRESS, &data).unwrap();

        println!("Writing register block {:#04x} with value {}", address, value);
        // Wire.beginTransmission(BQ34Z100_G1_ADDRESS);
        // Wire.write(addr);
        // Wire.write(val);
        // Wire.endTransmission(true);
    }

    fn write_flash_block(&mut self, sub_class:u8 ,  offset:u8) {
        self.write_reg(0x61, 0x00); // Block control
        self.write_reg(0x3e, sub_class); // Flash class
        self.write_reg(0x3f, offset / 32); // Flash block
        
        self.i2c.write(BQ34Z100_G1_ADDRESS, &self.flash_block_data).unwrap();
        // write_reg(0x61, 0x00); // Block control
        // write_reg(0x3e, sub_class); // Flash class
        // write_reg(0x3f, offset / 32); // Flash block
        
        // Wire.beginTransmission(BQ34Z100_G1_ADDRESS);
        // Wire.write(0x40); // Block data
        // for (uint8_t i = 0; i < 32; i++) {
        //     Wire.write(flash_block_data[i]); // Data
        // }
        // Wire.endTransmission(true);
    }

    fn flash_block_checksum(&mut self) -> u8 {
        let mut temp: u8 = 0;
        for i in self.flash_block_data.iter(){
            temp = u8::wrapping_add(temp, *i);
        }
        return u8::wrapping_sub(255,temp);
        // uint8_t temp = 0;
        // for (uint8_t i = 0; i < 32; i++) {
        //     temp += flash_block_data[i];
        // }
        // return 255 - temp;
    }

    fn unsealed(&mut self) {
        println!("Unsealing");
        let data : [u8;3] = [0x00, 0x14, 0x04];
        self.i2c.write(BQ34Z100_G1_ADDRESS, &data).unwrap();

        let data2 : [u8;3] = [0x00, 0x72, 0x36];
        self.i2c.write(BQ34Z100_G1_ADDRESS, &data2).unwrap();
        // Wire.beginTransmission(BQ34Z100_G1_ADDRESS);
        // Wire.write(0x00); // Control
        // Wire.write(0x14);
        // Wire.write(0x04);
        // Wire.endTransmission();
        
        // Wire.beginTransmission(BQ34Z100_G1_ADDRESS);
        // Wire.write(0x00); // Control
        // Wire.write(0x72);
        // Wire.write(0x36);
        // Wire.endTransmission();
    }

    fn enter_calibration(&mut self) {
        println!("enter_calibration");
        self.unsealed();
        loop {
            self.cal_enable();
            println!("Enable cal");
            self.enter_cal();
            self.delay.delay_ms(1000);
            if (self.control_status() & 0x1000 > 0) {
                break;
            }
        }; // CALEN
        // unsealed();
        // do {
        //     cal_enable();
        //     enter_cal();
        //     delay(1000);
        // } while (!(control_status() & 0x1000)); // CALEN
    }

    fn exit_calibration(&mut self) {
        loop{
            self.exit_cal();
            self.delay.delay_ms(1000);
            if self.control_status() & 0x1000 == 0 {
                break;
            }
        } // CALEN
        self.delay.delay_ms(150);
        self.reset();
        self.delay.delay_ms(150);
        // do {
        //     exit_cal();
        //     delay(1000);
        // } while (!(control_status() &~ 0x1000)); // CALEN
        
        // delay(150);
        // reset();
        // delay(150);
    }

    fn update_design_capacity(&mut self, capacity:u16) -> bool {
        self.unsealed();
        self.read_flash_block(48, 0);

        self.flash_block_data[6] = 0; // Cycle Count
        self.flash_block_data[7] = 0;
        
        self.flash_block_data[8] = (capacity >> 8) as u8; // CC Threshold
        self.flash_block_data[9] = (capacity & 0xff) as u8;
        
        self.flash_block_data[11] = (capacity >> 8) as u8; // Design Capacity
        self.flash_block_data[12] = (capacity & 0xff) as u8;

        println!("Block 11 {} block 12 {}", self.flash_block_data[11], self.flash_block_data[12]);

        for i in 6..=9  {
            self.write_reg(0x40 + i, self.flash_block_data[i as usize]);
        }
        
        for  i in 11..=12  {
            self.write_reg(0x40 + i, self.flash_block_data[i as usize]);
        }
        
        let checksum = self.flash_block_checksum();
        self.write_reg(0x60, checksum);
        
        
        println!("Checksum {}", checksum);

        self.delay.delay_ms(150);
        self.reset();
        self.delay.delay_ms(150);

        self.unsealed();        
        
        self.read_flash_block(48, 0);
        let mut updated_cc_threshold : i16 = (self.flash_block_data[8] as i16) << 8_i16;
        updated_cc_threshold |= self.flash_block_data[9] as i16;
        
        let mut updated_capacity: i16 = (self.flash_block_data[11] as i16) << 8;
        updated_capacity |= self.flash_block_data[12] as i16;
        
        if (self.flash_block_data[6] != 0 || self.flash_block_data[7] != 0) {
            println!("Block 6 or 7 wrong");
            return false;
        }
        println!("Expected capacity {} updated threshold {}" ,  capacity, updated_capacity);
        if (capacity as i32 != updated_cc_threshold as i32) {
            println!("cc threshold wrong");
            return false;
        }
        if (capacity as i32 != updated_capacity as i32) {
            println!("capacity wrong");
            return false;
        }
        return true;
        // unsealed();
        // read_flash_block(48, 0);

        // flash_block_data[6] = 0; // Cycle Count
        // flash_block_data[7] = 0;

        // flash_block_data[8] = capacity >> 8; // CC Threshold
        // flash_block_data[9] = capacity & 0xff;

        // flash_block_data[11] = capacity >> 8; // Design Capacity
        // flash_block_data[12] = capacity & 0xff;

        // for (uint8_t i = 6; i <= 9; i++) {
        //     write_reg(0x40 + i, flash_block_data[i]);
        // }

        // for (uint8_t i = 11; i <= 12; i++) {
        //     write_reg(0x40 + i, flash_block_data[i]);
        // }

        // write_reg(0x60, flash_block_checksum());

        // delay(150);
        // reset();
        // delay(150);

        // unsealed();
        // read_flash_block(48, 0);
        // int16_t updated_cc_threshold = flash_block_data[8] << 8;
        // updated_cc_threshold |= flash_block_data[9];

        // int16_t updated_capacity = flash_block_data[11] << 8;
        // updated_capacity |= flash_block_data[12];

        // if (flash_block_data[6] != 0 || flash_block_data[7] != 0) {
        //     return false;
        // }
        // if (capacity != updated_cc_threshold) {
        //     return false;
        // }
        // if (capacity != updated_capacity) {
        //     return false;
        // }
        // return true;
    }

    fn update_q_max(&mut self, capacity:i16) -> bool  {
        self.unsealed();
        self.read_flash_block(82, 0);
        self.flash_block_data[0] = (capacity >> 8) as u8; // Q Max
        self.flash_block_data[1] = (capacity & 0xff) as u8;
        
        self.flash_block_data[2] = 0; // Cycle Count
        self.flash_block_data[3] = 0;
        
        for i in 0_u8 .. 3_u8 {
            self.write_reg(0x40 + i, self.flash_block_data[i as usize]);
        }
        
        let checksum = self.flash_block_checksum();
        self.write_reg(0x60, checksum);
        
        self.delay.delay_ms(150);
        self.reset();
        self.delay.delay_ms(150);
        
        self.unsealed();
        self.read_flash_block(82, 0);
        let mut updated_q_max: i16 = (self.flash_block_data[0] as i16) << 8;
        updated_q_max |= self.flash_block_data[1] as i16;
        
        if (capacity != updated_q_max) {
            return false;
        }
        return true;

        // unsealed();
        // read_flash_block(82, 0);
        // flash_block_data[0] = capacity >> 8; // Q Max
        // flash_block_data[1] = capacity & 0xff;
        
        // flash_block_data[2] = 0; // Cycle Count
        // flash_block_data[3] = 0;
        
        // for (uint8_t i = 0; i <= 3; i++) {
        //     write_reg(0x40 + i, flash_block_data[i]);
        // }
        
        // write_reg(0x60, flash_block_checksum());
        
        // delay(150);
        // reset();
        // delay(150);
        
        // unsealed();
        // read_flash_block(82, 0);
        // int16_t updated_q_max = flash_block_data[0] << 8;
        // updated_q_max |= flash_block_data[1];
        
        // if (capacity != updated_q_max) {
        //     return false;
        // }
        // return true;
    }

    fn update_design_energy(&mut self, energy:i16, energy_scale:u8) -> bool  {
        self.unsealed();
        self.read_flash_block(48, 0);
        self.flash_block_data[13] = (energy >> 8) as u8; // Design Energy
        self.flash_block_data[14] = (energy & 0xff) as u8;
        self.flash_block_data[30] = energy_scale;
        
        for i in 13..=14 {
            self.write_reg(0x40 + i, self.flash_block_data[i as usize]);
        }
        self.write_reg(0x40 + 30, self.flash_block_data[30]);
        
        let checksum = self.flash_block_checksum();
        self.write_reg(0x60, checksum);
        
        self.delay.delay_ms(150);
        self.reset();
        self.delay.delay_ms(150);
        
        self.unsealed();
        self.read_flash_block(48, 0);
        let mut updated_energy :i16 = (self.flash_block_data[13] as i16) << 8;
        updated_energy |= self.flash_block_data[14] as i16;
        
        if (energy != updated_energy) {
            return false;
        }


        let updated_energy_scale: u8 = self.flash_block_data[30];
        if updated_energy_scale != energy_scale {
            return false;
        }

        return true;

        // unsealed();
        // read_flash_block(48, 0);
        // flash_block_data[13] = energy >> 8; // Design Energy
        // flash_block_data[14] = energy & 0xff;
        
        // for (uint8_t i = 13; i <= 14; i++) {
        //     write_reg(0x40 + i, flash_block_data[i]);
        // }
        
        // write_reg(0x60, flash_block_checksum());
        
        // delay(150);
        // reset();
        // delay(150);
        
        // unsealed();
        // read_flash_block(48, 0);
        // int16_t updated_energy = flash_block_data[13] << 8;
        // updated_energy |= flash_block_data[14];
        
        // if (energy != updated_energy) {
        //     return false;
        // }
        // return true;
    }

    fn update_cell_charge_voltage_range(&mut self, t1_t2:u16, t2_t3:u16, t3_t4:u16)-> bool  {
        self.unsealed();
        self.read_flash_block(48, 0);
    
        self.flash_block_data[17] = (t1_t2 >> 8) as u8; // Cell Charge Voltage T1-T2
        self.flash_block_data[18] = (t1_t2 & 0xff) as u8;
    
        self.flash_block_data[19] = (t2_t3 >> 8) as u8; // Cell Charge Voltage T2-T3
        self.flash_block_data[20] = (t2_t3 & 0xff) as u8;
    
        self.flash_block_data[21] = (t3_t4 >> 8) as u8; // Cell Charge Voltage T3-T4
        self.flash_block_data[22] = (t3_t4 & 0xff) as u8;
    
    for i in 17..=22 {
        self.write_reg(0x40 + i, self.flash_block_data[i as usize]);
    }
    
    let checksum = self.flash_block_checksum();
    self.write_reg(0x60, checksum);
    
    self.delay.delay_ms(150);
    self.reset();
    self.delay.delay_ms(150);
    
    self.unsealed();
    self.read_flash_block(48, 0);
    let mut updated_t1_t2 : u16 = (self.flash_block_data[17] as u16) << 8;
    updated_t1_t2 |= self.flash_block_data[18] as u16;
    
    let mut updated_t2_t3 : u16 = (self.flash_block_data[19] as u16 )<< 8;
    updated_t2_t3 |= self.flash_block_data[20] as u16;
    
    let mut updated_t3_t4 : u16 = (self.flash_block_data[21] as u16) << 8;
    updated_t3_t4 |= self.flash_block_data[22] as u16;
    
    if (t1_t2 as u16 != updated_t1_t2 || t2_t3 as u16 != updated_t2_t3 || t3_t4 as u16 != updated_t3_t4) {
        return false;
    }
    return true;
    //     unsealed();
    // read_flash_block(48, 0);
    
    // flash_block_data[17] = t1_t2 >> 8; // Cell Charge Voltage T1-T2
    // flash_block_data[18] = t1_t2 & 0xff;
    
    // flash_block_data[19] = t2_t3 >> 8; // Cell Charge Voltage T2-T3
    // flash_block_data[20] = t2_t3 & 0xff;
    
    // flash_block_data[21] = t3_t4 >> 8; // Cell Charge Voltage T3-T4
    // flash_block_data[22] = t3_t4 & 0xff;
    
    // for (uint8_t i = 17; i <= 22; i++) {
    //     write_reg(0x40 + i, flash_block_data[i]);
    // }
    
    // write_reg(0x60, flash_block_checksum());
    
    // delay(150);
    // reset();
    // delay(150);
    
    // unsealed();
    // read_flash_block(48, 0);
    // uint16_t updated_t1_t2 = flash_block_data[17] << 8;
    // updated_t1_t2 |= flash_block_data[18];
    
    // uint16_t updated_t2_t3 = flash_block_data[19] << 8;
    // updated_t2_t3 |= flash_block_data[20];
    
    // uint16_t updated_t3_t4 = flash_block_data[21] << 8;
    // updated_t3_t4 |= flash_block_data[22];
    
    // if (t1_t2 != updated_t1_t2 || t2_t3 != updated_t2_t3 || t3_t4 != updated_t3_t4) {
    //     return false;
    // }
    // return true;
    }

    fn update_number_of_series_cells(&mut self, cells:u8)-> bool  {
        self.unsealed();
        self.read_flash_block(64, 0);
    
        self.flash_block_data[7] = cells; // Number of Series Cell  
        self.write_reg(0x40 + 7, self.flash_block_data[7]);
    
        let checksum = self.flash_block_checksum();
        self.write_reg(0x60, checksum);
    
        self.delay.delay_ms(150);
        self.reset();
        self.delay.delay_ms(150);
    
        self.unsealed();
        self.read_flash_block(64, 0);
    
    if (cells != self.flash_block_data[7]) {
        return false;
    }
    return true;
    //     unsealed();
    // read_flash_block(64, 0);
    
    // flash_block_data[7] = cells; // Number of Series Cell
    
    // write_reg(0x40 + 7, flash_block_data[7]);
    
    // write_reg(0x60, flash_block_checksum());
    
    // delay(150);
    // reset();
    // delay(150);
    
    // unsealed();
    // read_flash_block(64, 0);
    
    // if (cells != flash_block_data[7]) {
    //     return false;
    // }
    // return true;
    }

    fn update_pack_configuration(&mut self, config:u16) -> bool  {
        self.unsealed();
        self.read_flash_block(64, 0);
        
        self.flash_block_data[0] = (config >> 8) as u8; // Pack Configuration
        self.flash_block_data[1] = (config & 0xff) as u8;
        
        for i in 0..=1 {
            self.write_reg(0x40 + i, self.flash_block_data[i as usize]);
        }
        
        let checksum = self.flash_block_checksum();
        self.write_reg(0x60, checksum);
        
        self.delay.delay_ms(150);
        self.reset();
        self.delay.delay_ms(1000);
        
        self.unsealed();
        self.read_flash_block(64, 0);
        let mut updated_config = (self.flash_block_data[0] as u16) << 8;
        updated_config |= self.flash_block_data[1] as u16;
        if (config != updated_config) {
            return false;
        }
        return true;
        // unsealed();
        // read_flash_block(64, 0);
        
        // flash_block_data[0] = config >> 8; // Pack Configuration
        // flash_block_data[1] = config & 0xff;
        
        // for (uint8_t i = 0; i <= 1; i++) {
        //     write_reg(0x40 + i, flash_block_data[i]);
        // }
        
        // write_reg(0x60, flash_block_checksum());
        
        // delay(150);
        // reset();
        // delay(150);
        
        // unsealed();
        // read_flash_block(64, 0);
        // uint16_t updated_config = flash_block_data[0] << 8;
        // updated_config |= flash_block_data[1];
        // if (config != updated_config) {
        //     return false;
        // }
        // return true;
    }

    fn update_charge_termination_parameters(&mut self, taper_current:i16, min_taper_capacity:i16, cell_taper_voltage:i16, 
        taper_window:u8, tca_set:i8, tca_clear:i8, fc_set:i8, fc_clear:i8) -> bool  {
            self.unsealed();
            self.read_flash_block(36, 0);
            
            self.flash_block_data[0] = (taper_current >> 8) as u8; // Taper Current
            self.flash_block_data[1] = (taper_current & 0xff) as u8;
            
            self.flash_block_data[2] = (min_taper_capacity >> 8) as u8; // Min Taper Capacity
            self.flash_block_data[3] = (min_taper_capacity & 0xff) as u8;
            
            self.flash_block_data[4] = (cell_taper_voltage >> 8) as u8; // Cell Taper Voltage
            self.flash_block_data[5] = (cell_taper_voltage & 0xff) as u8;
            
            self.flash_block_data[6] = taper_window; // Current Taper Window
            
            self.flash_block_data[7] = (tca_set as u8) & 0xff; // TCA Set %
            
            self.flash_block_data[8] = (tca_clear as u8) & 0xff; // TCA Clear %
            
            self.flash_block_data[9] = (fc_set as u8) & 0xff; // FC Set %
            
            self.flash_block_data[10] = (fc_clear as u8) & 0xff; // FC Clear %
            
            for i in 0..=10 {
                self.write_reg(0x40 + i, self.flash_block_data[i as usize]);
            }
            
            let checksum = self.flash_block_checksum();
            self.write_reg(0x60, checksum);
            
            self.delay.delay_ms(150);
            self.reset();
            self.delay.delay_ms(150);
            
            self.unsealed();
            self.read_flash_block(36, 0);
            let mut updated_taper_current :i16;
            let mut updated_min_taper_capacity: i16;
            let mut updated_cell_taper_voltage: i16;
            let updated_taper_window: u8;
            let updated_tca_set: i8;
            let updated_tca_clear: i8;
            let updated_fc_set: i8;
            let updated_fc_clear: i8;
            
            updated_taper_current = (self.flash_block_data[0] as i16) << 8;
            updated_taper_current |= self.flash_block_data[1] as i16;
            
            updated_min_taper_capacity = (self.flash_block_data[2] as i16) << 8;
            updated_min_taper_capacity |= self.flash_block_data[3] as i16;
            
            updated_cell_taper_voltage = (self.flash_block_data[4] as i16) << 8;
            updated_cell_taper_voltage |= self.flash_block_data[5] as i16;
            
            updated_taper_window = self.flash_block_data[6];
            
            updated_tca_set = (self.flash_block_data[7] & 0xff) as i8;
            
            updated_tca_clear = (self.flash_block_data[8] & 0xff) as i8;
            
            updated_fc_set = (self.flash_block_data[9] & 0xff) as i8;
            
            updated_fc_clear = (self.flash_block_data[10] & 0xff) as i8;
            
            if (taper_current != updated_taper_current) {
                println!("Could not update taper current expected {} actual {}", taper_current, updated_taper_current);
                return false;
            }
            if (min_taper_capacity != updated_min_taper_capacity) {
                println!("Could not update min_taper_capacity expected {} actual {}", min_taper_capacity, updated_min_taper_capacity);
                return false;
            }
            if (cell_taper_voltage != updated_cell_taper_voltage) {
                println!("Could not update cell_taper_voltage expected {} actual {}", cell_taper_voltage, updated_cell_taper_voltage);
                return false;
            }
            if (taper_window != updated_taper_window) {
                println!("Could not update taper_window expected {} actual {}", taper_window, updated_taper_window);
                return false;
            }
            if (tca_set != updated_tca_set) {
                println!("Could not update tca_set expected {} actual {}", tca_set, updated_tca_set);
                return false;
            }
            if (tca_clear != updated_tca_clear) {
                println!("Could not update tca_clear expected {} actual {}", tca_clear, updated_tca_clear);
                return false;
            }
            if (fc_set != updated_fc_set) {
                println!("Could not update fc_set expected {} actual {}", fc_set, updated_fc_set);
                return false;
            }
            if (fc_clear != updated_fc_clear) {
                println!("Could not update fc_clear expected {} actual {}", fc_clear, updated_fc_clear);
                return false;
            }
            return true;

    //         unsealed();
    // read_flash_block(36, 0);
    
    // flash_block_data[0] = taper_current >> 8; // Taper Current
    // flash_block_data[1] = taper_current & 0xff;
    
    // flash_block_data[2] = min_taper_capacity >> 8; // Min Taper Capacity
    // flash_block_data[3] = min_taper_capacity & 0xff;
    
    // flash_block_data[4] = cell_taper_voltage >> 8; // Cell Taper Voltage
    // flash_block_data[5] = cell_taper_voltage & 0xff;
    
    // flash_block_data[6] = taper_window; // Current Taper Window
    
    // flash_block_data[7] = tca_set & 0xff; // TCA Set %
    
    // flash_block_data[8] = tca_clear & 0xff; // TCA Clear %
    
    // flash_block_data[9] = fc_set & 0xff; // FC Set %
    
    // flash_block_data[10] = fc_clear & 0xff; // FC Clear %
    
    // for (uint8_t i = 0; i <= 10; i++) {
    //     write_reg(0x40 + i, flash_block_data[i]);
    // }
    
    // write_reg(0x60, flash_block_checksum());
    
    // delay(150);
    // reset();
    // delay(150);
    
    // unsealed();
    // read_flash_block(36, 0);
    // int16_t updated_taper_current, updated_min_taper_capacity, updated_cell_taper_voltage;
    // uint8_t updated_taper_window;
    // int8_t updated_tca_set, updated_tca_clear, updated_fc_set, updated_fc_clear;
    
    // updated_taper_current = flash_block_data[0] << 8;
    // updated_taper_current |= flash_block_data[1];
    
    // updated_min_taper_capacity = flash_block_data[2] << 8;
    // updated_min_taper_capacity |= flash_block_data[3];
    
    // updated_cell_taper_voltage = flash_block_data[4] << 8;
    // updated_cell_taper_voltage |= flash_block_data[5];
    
    // updated_taper_window = flash_block_data[6];
    
    // updated_tca_set = flash_block_data[7] & 0xff;
    
    // updated_tca_clear = flash_block_data[8] & 0xff;
    
    // updated_fc_set = flash_block_data[9] & 0xff;
    
    // updated_fc_clear = flash_block_data[10] & 0xff;
    
    // if (taper_current != updated_taper_current) {
    //     return false;
    // }
    // if (min_taper_capacity != updated_min_taper_capacity) {
    //     return false;
    // }
    // if (cell_taper_voltage != updated_cell_taper_voltage) {
    //     return false;
    // }
    // if (taper_window != updated_taper_window) {
    //     return false;
    // }
    // if (tca_set != updated_tca_set) {
    //     return false;
    // }
    // if (tca_clear != updated_tca_clear) {
    //     return false;
    // }
    // if (fc_set != updated_fc_set) {
    //     return false;
    // }
    // if (fc_clear != updated_fc_clear) {
    //     return false;
    // }
    // return true;
    }

    fn calibrate_cc_offset(&mut self) {
        self.enter_calibration();
        
        loop {
            println!("Loop cc offset");
            self.cc_offset();
            self.delay.delay_ms(1000);
            if(self.control_status() & 0x0800 > 0){
                break;
            }
        } // CCA
        
        loop {
            self.delay.delay_ms(1000);
            if(self.control_status() & 0x0800 == 0){
                break;
            } 
        } // CCA
        
        self.cc_offset_save();
        self.exit_calibration();
        // enter_calibration();
        // do {
        //     cc_offset();
        //     delay(1000);
        // } while (!(control_status() & 0x0800)); // CCA
        
        // do {
        //     delay(1000);
        // } while (!(control_status() &~ 0x0800)); // CCA
        
        // cc_offset_save();
        // exit_calibration();
    }

    fn calibrate_board_offset(&mut self) {
        self.enter_calibration();
        loop {
            self.board_offset();
            self.delay.delay_ms(1000);
            if self.control_status() & 0x0c00  > 0{
                break;
            }
        }// CCA + BCA
        
        loop {
            self.delay.delay_ms(1000);
            if self.control_status() & 0x0c00  == 0{
                break;
            } 
        } // CCA + BCA
        
        self.cc_offset_save();
        self.exit_calibration();

        // enter_calibration();
        // do {
        //     board_offset();
        //     delay(1000);
        // } while (!(control_status() & 0x0c00)); // CCA + BCA
        
        // do {
        //     delay(1000);
        // } while (!(control_status() &~ 0x0c00)); // CCA + BCA
        
        // cc_offset_save();
        // exit_calibration();
    }

    fn calibrate_voltage_divider(&mut self, applied_voltage:f32,  cells_count:u8) {
        let mut volt_array : [f32;50] = [0.0; 50];
        for i in 0 .. 50 {
            
            volt_array[i] = self.voltage() as f32;
            self.delay.delay_ms(150);
            println!("Reading voltage {} as {}", i, volt_array[i]);
        }
        let mut volt_mean : f32 = 0.0;
        for i in  0..50 {
            volt_mean += volt_array[i];
        }
        volt_mean /= 50.0;
        
        let mut volt_sd: f32 = 0.0;
        for i in 0..50 {
            volt_sd +=  (volt_array[i] - volt_mean).powf(2.0);
        }
        volt_sd /= 50.0;
        volt_sd = volt_sd.sqrt();
        
        if (volt_sd > 100.0) {
            return;
        }
    
        self.unsealed();
        
        self.read_flash_block(104, 0);
        
        
        let mut current_voltage_divider : u16 = (self.flash_block_data[14] as u16) << 8;
        current_voltage_divider |= self.flash_block_data[15] as u16;
        
        let new_voltage_divider: u16 = ((applied_voltage as f32/ volt_mean as f32) * current_voltage_divider as f32) as u16;
        
        println!("Setting new voltage divider to {}", new_voltage_divider);

        self.flash_block_data[14] = (new_voltage_divider >> 8) as u8;
        self.flash_block_data[15] = (new_voltage_divider & 0xff) as u8;
        
        for i in 14..=15 {
            self.write_reg(0x40 + i, self.flash_block_data[i as usize]);
        }
        
        let checksum = self.flash_block_checksum();
        self.write_reg(0x60, checksum);
        self.delay.delay_ms(150);
        
       // self.unsealed();
       // self.read_flash_block(68, 0);
        
       // let flash_update_of_cell_voltage:i16 = ((2800.0 * cells_count as f32 * 5000.0) / new_voltage_divider as f32) as i16;
        
       // self.flash_block_data[0] = (flash_update_of_cell_voltage << 8) as u8;
       // self.flash_block_data[1] = (flash_update_of_cell_voltage & 0xff) as u8;
        
       // for i in 0..=1 {
       //     self.write_reg(0x40 + i, self.flash_block_data[i as usize]);
       // }
       // println!("Wrote cell voltage {}", flash_update_of_cell_voltage);
        
       // let checksum = self.flash_block_checksum();
       // self.write_reg(0x60, checksum);
        
       // self.delay.delay_ms(150);
       // self.reset();
       // self.delay.delay_ms(150);

    //     double volt_array[50];
    // for (uint8_t i = 0; i < 50; i++) {
    //     volt_array[i] = voltage();
    //     delay(150);
    // }
    // double volt_mean = 0;
    // for (uint8_t i = 0; i < 50; i++) {
    //     volt_mean += volt_array[i];
    // }
    // volt_mean /= 50.0;
    
    // double volt_sd = 0;
    // for (uint8_t i = 0; i < 50; i++) {
    //     volt_sd += pow(volt_array[i] - volt_mean, 2);
    // }
    // volt_sd /= 50.0;
    // volt_sd = sqrt(volt_sd);
    
    // if (volt_sd > 100) {
    //     return;
    // }

    // unsealed();
    // read_flash_block(104, 0);
    
    // uint16_t current_voltage_divider = flash_block_data[14] << 8;
    // current_voltage_divider |= flash_block_data[15];
    
    // uint16_t new_voltage_divider = ((double)applied_voltage / volt_mean) * (double)current_voltage_divider;
    
    // flash_block_data[14] = new_voltage_divider >> 8;
    // flash_block_data[15] = new_voltage_divider & 0xff;
    
    // for (uint8_t i = 14; i <= 15; i++) {
    //     write_reg(0x40 + i, flash_block_data[i]);
    // }
    
    // write_reg(0x60, flash_block_checksum());
    // delay(150);
    
    // unsealed();
    // read_flash_block(68, 0);
    
    // int16_t flash_update_of_cell_voltage = (double)(2800 * cells_count * 5000) / (double)new_voltage_divider;
    
    // flash_block_data[0] = flash_update_of_cell_voltage << 8;
    // flash_block_data[1] = flash_update_of_cell_voltage & 0xff;
    
    // for (uint8_t i = 0; i <= 1; i++) {
    //     write_reg(0x40 + i, flash_block_data[i]);
    // }
    
    // write_reg(0x60, flash_block_checksum());
    
    // delay(150);
    // reset();
    // delay(150);
    }

    fn calibrate_sense_resistor(&mut self, applied_current:i16) {
        // // test data from https://e2e.ti.com/support/power-management/f/196/p/551252/2020286?tisearch=e2e-quicksearch&keymatch=xemics#2020286
        // let value_float: f32 = 0.8335;
        // let value_xemics: u32 = 0x80555E9E;
        // // try converting float to xemics
        // let converted_value: u32 = double_to_xemics(value_float);
        // println!("Converted value: {}", converted_value);
         
        // // try converting xemics to float
        // let converted_float :f32 = xemics_to_double(value_xemics);
        // println!("Converted float: {}", converted_float);
        
        // println!("Expected default CC Gain: {}", double_to_xemics(0.4768));
        // println!("Expected default CC Delta: {}", double_to_xemics(567744.56));
        for i in 1 .. 500000 {
            let xemics = double_to_xemics(i as f32);
            let restored = xemics_to_double(xemics);
            if((i as f32 - restored).abs() > 0.1){
                println!("Large diff for {}, restored as {}", i , restored);
            }
        }

        unsafe { vTaskDelay(1001) };
        let mut current_array: [f32;50] = [0.0;50];
        for i in 0 .. 50 {
            current_array[i] = self.current() as f32;
            println!("Reading current {} @ {}", current_array[i], i);
            self.delay.delay_ms(150);
        }
        let mut current_mean: f32 = 0.0;
        for i in 0 .. 50 {
            current_mean += current_array[i];
        }
        current_mean /= 50.0;

        let mut current_sd: f32 = 0.0;
        for i in 0 .. 50 {
            current_sd += (current_array[i] - current_mean).powf(2.0);
        }
        current_sd /= 50.0;
        current_sd = current_sd.sqrt();

        if (current_sd > 100.0) {
            return;
        }

        self.unsealed();
        self.read_flash_block(104, 0);

        let mut cc_gain: u32 = (self.flash_block_data[0] as u32) << 24;
        cc_gain |= (self.flash_block_data[1] as u32) << 16;
        cc_gain |= (self.flash_block_data[2] as u32) << 8;
        cc_gain |= self.flash_block_data[3] as u32;
        
        let float_cc_gain = xemics_to_double(cc_gain);
        let xemics_cc_gain = double_to_xemics(float_cc_gain);
        let float_cc_gain2 = xemics_to_double(xemics_cc_gain);
        if (float_cc_gain-float_cc_gain2).abs() > 0.01 {
            println!("Error converting old gain!!");
        }

        let mut gain_resistence: f32 = 4.768 / float_cc_gain;
        println!("Current gain R is {}  xemics is {}", gain_resistence, cc_gain);
        if(gain_resistence == 0.0){
            gain_resistence = 10.0;
        }
        
        let mut temp: f32 = (current_mean * gain_resistence) / applied_current as f32;
        println!("Current is {} , applied current ist {}, new gain is {}", current_mean, applied_current, temp);
        if(temp == 0.0){
            println!("Failure calculating new gain, fallback gain used");
            temp = 10.0;
        }

        let mut new_cc_gain : u32 = double_to_xemics(4.768 / temp);
        self.flash_block_data[0] = (new_cc_gain >> 24) as u8;
        self.flash_block_data[1] = (new_cc_gain >> 16) as u8;
        self.flash_block_data[2] = (new_cc_gain >> 8) as u8;
        self.flash_block_data[3] = (new_cc_gain & 0xff) as u8;

        new_cc_gain = double_to_xemics(5677445.6 / temp);
        self.flash_block_data[4] = (new_cc_gain >> 24) as u8;
        self.flash_block_data[5] = (new_cc_gain >> 16) as u8;
        self.flash_block_data[6] = (new_cc_gain >> 8) as u8;
        self.flash_block_data[7] = (new_cc_gain & 0xff) as u8;
        

        for i in  0..=3 {
            self.write_reg(0x40 + i, self.flash_block_data[i as usize]);
        }

        for i in 4..=7 {
            self.write_reg(0x40 + i, self.flash_block_data[i as usize]);
        }

        let checksum = self.flash_block_checksum();
        self.write_reg(0x60, checksum);
        self.delay.delay_ms(150);
        self.reset();
        self.delay.delay_ms(150);
        
    //     double current_array[50];
    // for (uint8_t i = 0; i < 50; i++) {
    //     current_array[i] = current();
    //     delay(150);
    // }
    // double current_mean = 0;
    // for (uint8_t i = 0; i < 50; i++) {
    //     current_mean += current_array[i];
    // }
    // current_mean /= 50.0;

    // double current_sd = 0;
    // for (uint8_t i = 0; i < 50; i++) {
    //     current_sd += pow(current_array[i] - current_mean, 2);
    // }
    // current_sd /= 50.0;
    // current_sd = sqrt(current_sd);

    // if (current_sd > 100) {
    //     return;
    // }

    // unsealed();
    // read_flash_block(104, 0);

    // uint32_t cc_gain = flash_block_data[0] << 24;
    // cc_gain |= flash_block_data[1] << 16;
    // cc_gain |= flash_block_data[2] << 8;
    // cc_gain |= flash_block_data[3];
    
    // double gain_resistence = 4.768 / xemics_to_double(cc_gain);

    // double temp = (current_mean * gain_resistence) / (double)applied_current;

    // uint32_t new_cc_gain = double_to_xemics(4.768 / temp);
    // flash_block_data[0] = new_cc_gain >> 24;
    // flash_block_data[1] = new_cc_gain >> 16;
    // flash_block_data[2] = new_cc_gain >> 8;
    // flash_block_data[3] = new_cc_gain & 0xff;

    // new_cc_gain = double_to_xemics(5677445.6 / temp);
    // flash_block_data[4] = new_cc_gain >> 24;
    // flash_block_data[5] = new_cc_gain >> 16;
    // flash_block_data[6] = new_cc_gain >> 8;
    // flash_block_data[7] = new_cc_gain & 0xff;
    

    // for (uint8_t i = 0; i <= 3; i++) {
    //     write_reg(0x40 + i, flash_block_data[i]);
    // }

    // for (uint8_t i = 4; i <= 7; i++) {
    //     write_reg(0x40 + i, flash_block_data[i]);
    // }

    // write_reg(0x60, flash_block_checksum());
    // delay(150);
    // reset();
    // delay(150);
    }

    fn set_current_deadband(&mut self,  deadband:u8) {
        todo!()
    }

    fn ready(&mut self) {
        self.unsealed();
        self.it_enable();
    }

    fn control_status(&mut self) -> u16 {
        return self.read_control(0x00, 0x00);
    }

    fn device_type(&mut self) ->u16 {
        return self.read_control(0x01, 0x00);
    }

    fn fw_version(&mut self)->u16 {
        return self.read_control(0x02, 0x00);
    }

    fn hw_version(&mut self)->u16 {
        return self.read_control(0x03, 0x00);
    }

    fn reset_data(&mut self) -> u16  {
        return self.read_control(0x05, 0x00);
    }

    fn prev_macwrite(&mut self) -> u16  {
        return self.read_control(0x07, 0x00);
    }

    fn chem_id(&mut self) -> u16  {
        return self.read_control(0x08, 0x00);
    }

    fn board_offset(&mut self) -> u16  {
        return self.read_control(0x09, 0x00);
    }

    fn cc_offset(&mut self) -> u16  {
        return self.read_control(0x0a, 0x00);
    }

    fn cc_offset_save(&mut self) -> u16  {
        return self.read_control(0x0b, 0x00);
    }

    fn df_version(&mut self) -> u16  {
        return self.read_control(0x0c, 0x00);
    }

    fn set_fullsleep(&mut self) -> u16  {
        return self.read_control(0x10, 0x00);
    }

    fn static_chem_chksum(&mut self) -> u16  {
        return self.read_control(0x17, 0x00);
    }

    fn sealed(&mut self) -> u16  {
        todo!()
    }

    fn it_enable(&mut self) -> u16  {
        return self.read_control(0x21, 0x00);
    }

    fn cal_enable(&mut self) -> u16  {
        return self.read_control(0x2d, 0x00);
    }

    fn reset(&mut self) -> u16  {
        return self.read_control(0x41, 0x00);
    }

    fn exit_cal(&mut self) -> u16  {
        return self.read_control(0x80, 0x00);
    }

    fn enter_cal(&mut self) -> u16  {
        return self.read_control(0x81, 0x00);
    }

    fn offset_cal(&mut self) -> u16  {
        return self.read_control(0x82, 0x00);
    }

    fn state_of_charge(&mut self) -> u8  {
        return self.read_register(0x02, 1) as u8;
    }

    fn state_of_charge_max_error(&mut self) -> u8  {
        return self.read_register(0x03, 1) as u8;
    }

    fn remaining_capacity(&mut self) -> u16  {
        return self.read_register(0x04, 2);
    }

    fn full_charge_capacity(&mut self) -> u16  {
        return self.read_register(0x06, 2);
    }

    fn voltage(&mut self) -> u16  {
        return self.read_register(0x08, 2);
    }

    fn average_current(&mut self) -> i16 {
        return self.read_register(0x0a, 2) as i16;
    }

    fn temperature(&mut self) -> u16  {
        return self.read_register(0x0c, 2);
    }

    fn flags(&mut self) -> u16  {
        return self.read_register(0x0e, 2);
    }

    fn flags_b(&mut self) -> u16  {
        return self.read_register(0x12, 2);
    }

    fn current(&mut self) -> i16 {
        return self.read_register(0x10, 2) as i16;
    }

    fn average_time_to_empty(&mut self) -> u16 {
        return self.read_register(0x18, 2);
    }

    fn average_time_to_full(&mut self) -> u16 {
        return self.read_register(0x1a, 2);
    }

    fn passed_charge(&mut self) -> u16 {
        return self.read_register(0x1c, 2);
    }

    fn do_d0_time(&mut self) -> u16  {
        return self.read_register(0x1e, 2);
    }

    fn available_energy(&mut self) -> u16  {
        return self.read_register(0x24, 2);
    }

    fn average_power(&mut self) -> u16  {
        return self.read_register(0x26, 2);
    }

    fn serial_number(&mut self) -> u16  {
        return self.read_register(0x28, 2);
    }

    fn cycle_count(&mut self) -> u16  {
        return self.read_register(0x2c, 2);
    }

    fn state_of_health(&mut self) -> u16  {
        return self.read_register(0x2e, 2);
    }

    fn charge_voltage(&mut self) -> u16  {
        return self.read_register(0x30, 2);
    }

    fn charge_current(&mut self) -> u16 {
        return self.read_register(0x32, 2);
    }

    fn pack_configuration(&mut self) -> u16  {
        return self.read_register(0x3a, 2);
    }

    fn design_capacity(&mut self) -> u16  {
        return self.read_register(0x3c, 2);
    }

    fn grid_number(&mut self) -> u8  {
        return self.read_register(0x62, 1) as u8;
    }

    fn learned_status(&mut self) -> u8  {
        return self.read_register(0x63, 1) as u8;
    }

    fn dod_at_eoc(&mut self) -> u16  {
        return self.read_register(0x64, 2);
    }

    fn  q_start(&mut self) -> u16 {
        return self.read_register(0x66, 2);
    }

    fn  true_fcc(&mut self) -> u16 {
        return self.read_register(0x6a, 2);
    }

    fn state_time(&mut self) -> u16  {
        return self.read_register(0x6c, 2);
    }

    fn  q_max_passed_q(&mut self) -> u16 {
        return self.read_register(0x6e, 2);
    }

    fn  dod_0(&mut self) -> u16 {
        return self.read_register(0x70, 2);
    }

    fn  q_max_dod_0(&mut self) -> u16 {
        return self.read_register(0x72, 2);
    }

    fn  q_max_time(&mut self) -> u16 {
        return self.read_register(0x74, 2);
    }

}

pub struct Bq34z100g1Driver<I2C, Delay>{
    pub i2c: I2C,
    pub delay: Delay,
    pub flash_block_data: [u8;32],
}
pub trait Bq34z100g1 {
    fn read_register(&mut self, address:u8 , length:u8) -> u16;
    fn read_control(&mut self, address_lsb:u8, address_msb: u8) -> u16;
    fn read_flash_block(&mut self, sub_class:u8, offset:u8);
    fn write_reg(&mut self, address:u8, value:u8 );
    fn write_flash_block(&mut self, sub_class:u8 ,  offset:u8);
    
    fn flash_block_checksum(&mut self) -> u8;
    
    fn unsealed(&mut self);
    fn enter_calibration(&mut self);
    fn exit_calibration(&mut self);

    fn update_design_capacity(&mut self, capacity:u16) -> bool;
    fn update_q_max(&mut self, capacity:i16) -> bool ;
    fn update_design_energy(&mut self, energy:i16, scale:u8) -> bool ;
    fn update_cell_charge_voltage_range(&mut self, t1_t2:u16, t2_t3:u16, t3_t4:u16)-> bool ;
    fn update_number_of_series_cells(&mut self, cells:u8)-> bool ;
    fn update_pack_configuration(&mut self, config:u16) -> bool ;
    fn update_charge_termination_parameters(&mut self, taper_current:i16, min_taper_capacity:i16, cell_taper_voltage:i16, 
        taper_window:u8, tca_set:i8, tca_clear:i8, fc_set:i8, fc_clear:i8) -> bool ;
    fn calibrate_cc_offset(&mut self);
    fn calibrate_board_offset(&mut self);
    fn calibrate_voltage_divider(&mut self, applied_voltage:f32,  cells_count:u8);
    fn calibrate_sense_resistor(&mut self, applied_current:i16);
    fn set_current_deadband(&mut self,  deadband:u8);
    fn ready(&mut self);
    
    fn control_status(&mut self) -> u16;
    fn device_type(&mut self) ->u16;
    fn fw_version(&mut self)->u16;
    fn hw_version(&mut self)->u16;
    fn reset_data(&mut self) -> u16 ;
    fn prev_macwrite(&mut self) -> u16 ;
    fn chem_id(&mut self) -> u16 ;
    fn board_offset(&mut self) -> u16 ;
    fn cc_offset(&mut self) -> u16 ;
    fn cc_offset_save(&mut self) -> u16 ;
    fn df_version(&mut self) -> u16 ;
    fn set_fullsleep(&mut self) -> u16 ;
    fn static_chem_chksum(&mut self) -> u16 ;
    fn sealed(&mut self) -> u16 ;
    fn it_enable(&mut self) -> u16 ;
    fn cal_enable(&mut self) -> u16 ;
    fn reset(&mut self) -> u16 ;
    fn exit_cal(&mut self) -> u16 ;
    fn enter_cal(&mut self) -> u16 ;
    fn offset_cal(&mut self) -> u16 ;
    
    fn state_of_charge(&mut self) -> u8 ; // 0 to 100%
    fn state_of_charge_max_error(&mut self) -> u8 ; // 1 to 100%
    fn remaining_capacity(&mut self) -> u16 ; // mAh
    fn full_charge_capacity(&mut self) -> u16 ; // mAh
    fn voltage(&mut self) -> u16 ; // mV
    fn average_current(&mut self) -> i16; // mA
    fn temperature(&mut self) -> u16 ; // Unit of x10 K
    fn flags(&mut self) -> u16 ;
    fn flags_b(&mut self) -> u16 ;
    fn current(&mut self) -> i16; // mA
    
    fn average_time_to_empty(&mut self) -> u16; // Minutes
    fn average_time_to_full(&mut self) -> u16; // Minutes
    fn passed_charge(&mut self) -> u16; // mAh
    fn do_d0_time(&mut self) -> u16 ; // Minutes
    fn available_energy(&mut self) -> u16 ; // 10 mWh
    fn average_power(&mut self) -> u16 ; // 10 mW
    fn serial_number(&mut self) -> u16 ;
    fn internal_temperature(&mut self) -> u16 ; // Unit of x10 K
    fn cycle_count(&mut self) -> u16 ; // Counts
    fn state_of_health(&mut self) -> u16 ; // 0 to 100%
    fn charge_voltage(&mut self) -> u16 ; // mV
    fn charge_current(&mut self) -> u16; // mA
    fn pack_configuration(&mut self) -> u16 ;
    fn design_capacity(&mut self) -> u16 ; // mAh
    fn grid_number(&mut self) -> u8 ;
    fn learned_status(&mut self) -> u8 ;
    fn dod_at_eoc(&mut self) -> u16 ;
    fn  q_start(&mut self) -> u16; // mAh
    fn  true_fcc(&mut self) -> u16; // mAh
    fn state_time(&mut self) -> u16 ; // s
    fn  q_max_passed_q(&mut self) -> u16; // mAh
    fn  dod_0(&mut self) -> u16;
    fn  q_max_dod_0(&mut self) -> u16;
    fn  q_max_time(&mut self) -> u16;
}