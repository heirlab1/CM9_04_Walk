#include "IMU.h"

/*
  The following is an exaction from MPU6050.h based on I2Cdev library
  Dated 10/3/2011 by Jeff Rowberg
  
  edited by woeimun 30/4/2014
  edited by Adam Stroud, Marquette University HEIR Lab 3/22/2015
*/

uint8 buffer[20];

// I2C address 0x69 could be 0x68 depends on your wiring. 
int MPU9150_I2C_ADDRESS = 0x68;

void IMU::MPU9150_initialize(){

  MPU9150_writeBit(MPU9150_addr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 0);
  //select reference clock
  MPU9150_writeBits(MPU9150_addr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
  //select gyro range
  MPU9150_writeBits(MPU9150_addr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_250);
  //select accelrometer range
 MPU9150_writeBits(MPU9150_addr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS_2);
  //enable I2C bypass for magnetometer
  //MPU9150_writeBit(MPU9150_addr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, 1);
  //set rate
// buffer[0]=0xf0;
  MPU9150_write(MPU9150_addr, MPU6050_RA_SMPLRT_DIV, 1, buffer);
  
  MPU9150_setupCompass();

}


int IMU::getGYRO_X(){	return MPU9150_readSensor(MPU9150_GYRO_XOUT_L,MPU9150_GYRO_XOUT_H);}
int IMU::getGYRO_Y(){	return MPU9150_readSensor(MPU9150_GYRO_YOUT_L,MPU9150_GYRO_YOUT_H);}
int IMU::getGYRO_Z(){	return MPU9150_readSensor(MPU9150_GYRO_ZOUT_L,MPU9150_GYRO_ZOUT_H);}

int IMU::getACCL_X(){	return MPU9150_readSensor(MPU9150_ACCL_XOUT_L,MPU9150_ACCL_XOUT_H);}
int IMU::getACCL_Y(){	return MPU9150_readSensor(MPU9150_ACCL_YOUT_L,MPU9150_ACCL_YOUT_H);}
int IMU::getACCL_Z(){	return MPU9150_readSensor(MPU9150_ACCL_ZOUT_L,MPU9150_ACCL_ZOUT_H);}

int IMU::getMAG_X(){	return MPU9150_readSensor(MPU9150_RA_MAG_XOUT_L,MPU9150_RA_MAG_XOUT_H);}
int IMU::getMAG_Y(){	return MPU9150_readSensor(MPU9150_RA_MAG_YOUT_L,MPU9150_RA_MAG_YOUT_H);}
int IMU::getMAG_Z(){	return MPU9150_readSensor(MPU9150_RA_MAG_ZOUT_L,MPU9150_RA_MAG_ZOUT_H);}

int IMU::getCMPS_X(){	return MPU9150_readSensor(MPU9150_CMPS_XOUT_L,MPU9150_CMPS_XOUT_H);}
int IMU::getCMPS_Y(){	return MPU9150_readSensor(MPU9150_CMPS_YOUT_L,MPU9150_CMPS_YOUT_H);}
int IMU::getCMPS_Z(){	return MPU9150_readSensor(MPU9150_CMPS_ZOUT_L,MPU9150_CMPS_ZOUT_H);}


void IMU::MPU9150_setupCompass(){
  MPU9150_I2C_ADDRESS = 0x0C;      //change Adress to Compass

  MPU9150_writeSensor(0x0A, 0x00); //PowerDownMode
  MPU9150_writeSensor(0x0A, 0x0F); //SelfTest
  MPU9150_writeSensor(0x0A, 0x00); //PowerDownMode

  MPU9150_I2C_ADDRESS = 0x68;      //change Adress to MPU

  MPU9150_writeSensor(0x24, 0x40); //Wait for Data at Slave0
  MPU9150_writeSensor(0x25, 0x8C); //Set i2c address at slave0 at 0x0C
  MPU9150_writeSensor(0x26, 0x02); //Set where reading at slave 0 starts
  MPU9150_writeSensor(0x27, 0x88); //set offset at start reading and enable
  MPU9150_writeSensor(0x28, 0x0C); //set i2c address at slv1 at 0x0C
  MPU9150_writeSensor(0x29, 0x0A); //Set where reading at slave 1 starts
  MPU9150_writeSensor(0x2A, 0x81); //Enable at set length to 1
  MPU9150_writeSensor(0x64, 0x01); //overvride register
  MPU9150_writeSensor(0x67, 0x03); //set delay rate
  MPU9150_writeSensor(0x01, 0x80);

  MPU9150_writeSensor(0x34, 0x04); //set i2c slv4 delay
  MPU9150_writeSensor(0x64, 0x00); //override register
  MPU9150_writeSensor(0x6A, 0x00); //clear usr setting
  MPU9150_writeSensor(0x64, 0x01); //override register
  MPU9150_writeSensor(0x6A, 0x20); //enable master i2c mode
  MPU9150_writeSensor(0x34, 0x13); //disable slv4
}

int IMU::MPU9150_readSensor(int addrL, int addrH){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrL);
  Wire.endTransmission();

  Wire.requestFrom(MPU9150_I2C_ADDRESS, BUFFER_LENGTH);
  byte L = Wire.read();

  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrH);
  Wire.endTransmission();

  Wire.requestFrom(MPU9150_I2C_ADDRESS, BUFFER_LENGTH);
  byte H = Wire.read();

  return (int16)((H<<8)+L);
}

int IMU::MPU9150_readSensor(int addr){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom(MPU9150_I2C_ADDRESS,BUFFER_LENGTH);
  return Wire.read();
}

int IMU::MPU9150_writeSensor(int addr,int data){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();

  return 1;
}


uint8 IMU::MPU9150_read(uint8 devAddr,uint8 regAddr, uint8 length, uint8* buf, uint16 timeout)
{
    uint8 count = 0;
    unsigned long t1 = millis();

    for (unsigned char k = 0; k < length; k += min(length, BUFFER_LENGTH))
    {
        Wire.beginTransmission(devAddr);
        Wire.write(regAddr);
        Wire.endTransmission();
        Wire.beginTransmission(devAddr);
        Wire.requestFrom(devAddr, (uint8)min(length - k, BUFFER_LENGTH));
        
        while (Wire.available() && (timeout == 0 || millis() - t1 < timeout))
        {
            buf[count++] = Wire.read();           
        }
    }

    // check for timeout
    if (timeout > 0 && millis() - t1 >= timeout && count < length)
    {
        count = -1; // timeout
    }
    return count;
}

bool IMU::MPU9150_write(uint8 devAddr,uint8 regAddr,uint8 length, uint8* data)
{
     uint8 status = 0;

    Wire.beginTransmission(devAddr);
    Wire.write((uint8) regAddr); // send address

    for (uint8 i = 0; i < length; i++)
    {
        Wire.write((uint8) data[i]);
    }

    status = Wire.endTransmission();

}

bool IMU::MPU9150_writeBit(uint8 devAddr,uint8 regAddr,uint8 bitNum, uint8 data)
{
    uint8 b;
    MPU9150_read(devAddr, regAddr, 1, &b,DEFAULT_READ_TIMEOUT);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return MPU9150_write(devAddr, regAddr, 1, &b);  
}

bool IMU::MPU9150_writeBits(uint8 devAddr, uint8 regAddr, uint8 bitStart, uint8 length, uint8 data)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8 b;

    if (MPU9150_read(devAddr, regAddr, 1, &b,DEFAULT_READ_TIMEOUT) != 0)
    {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte

        return MPU9150_write(devAddr, regAddr, 1, &b);
    }

    return false;
}

int8 IMU::MPU9150_readBits(uint8 devAddr, uint8 regAddr, uint8 bitStart, uint8 length, uint8 *data)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8 count, b;
    if ((count = MPU9150_read(devAddr, regAddr, 1, &b,DEFAULT_READ_TIMEOUT)) != 0)
    {
        uint8 mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

bool IMU::testConnection(uint8 devAddr) 
{
  uint8 b[1];
  MPU9150_readBits(devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, b);
  return b[0]==0x34 ;
}
