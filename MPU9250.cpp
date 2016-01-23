// MPU-9250 I2C device class
// Based on InvenSense MPU-9250 Register Map and Descriptions Rev. 1.4, 9/9/2013 (RM-MPU-9250A-00)
// Jordan Day

/*
=========================================
This code is placed under the MIT license
Copyright (c) 2016 Jordan Day

Please reference the LICENSE file
=========================================
*/

#include "MPU9250.h"

MPU9250::MPU9250()
{
    Wire.begin();
}

void MPU9250::init()
{
    // Wake up
    writeByte(PWR_MGMT_1, 0x00);
    delay(100);

    // Get time source
    writeByte(PWR_MGMT_1, 0x01);
    delay(200);

    // Set Gyro and Temp bandwidth to 41 Hz and 42 Hz
    // Limit sample rate to 1 kHz each
    writeByte(CONFIG, 0x03);

    // Sample rate = Gyro output / (1 + SMPLRT_DIV)
    // 200 hz
    writeByte(SMPLRT_DIV, 0x04);

    // Set Gyro full scale range
    uint8_t conf = readByte(GYRO_CONFIG);
    writeByte(GYRO_CONFIG, conf & ~0x02);
    writeByte(GYRO_CONFIG, conf & ~0x18);
    writeByte(GYRO_CONFIG, conf | GyroScale << 3);

    // Set Accel full scale range
    conf = readByte(ACCEL_CONFIG);
    writeByte(ACCEL_CONFIG_2, conf & ~0x18);
    writeByte(ACCEL_CONFIG_2, conf | AccelScale << 3);

    // Set Accel sample rate
    conf = readByte(ACCEL_CONFIG_2);
    writeByte(ACCEL_CONFIG_2, conf & ~0x0F);
    writeByte(ACCEL_CONFIG_2, conf | 0x03);

    // Configure interrupts
    writeByte(INT_PIN_CFG, 0x22);
    writeByte(INT_ENABLE, 0x01);
    delay(100);

    setGyroRes();
    setAccelRes();
    setMagRes();
}

void MPU9250::calibrate()
{
    uint8_t data[12];
    uint16_t index, cycleCount, fifoCount;
    int32_t accelFactory[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    uint32_t temperatureMask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t maskBit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
    int32_t gyroOffset[3] = {0, 0, 0}, accelOffset[3] = {0, 0, 0};

    //Reset
    writeByte(PWR_MGMT_1, 0x80);
    delay(100);

    // Set time source
    writeByte(PWR_MGMT_1, 0x01);
    writeByte(PWR_MGMT_2, 0x00);
    delay(200);

    // Prepare device for offset calculation
    writeByte(INT_ENABLE, 0x00);
    writeByte(FIFO_EN, 0x00);
    writeByte(PWR_MGMT_1, 0x00);
    writeByte(I2C_MST_CTRL, 0x00);
    writeByte(USER_CTRL, 0x00);
    writeByte(USER_CTRL, 0x0C);
    delay(15);

    // Prepare gyro and accel for offset calculation
    writeByte(CONFIG, 0x01);
    writeByte(SMPLRT_DIV, 0x00);
    writeByte(GYRO_CONFIG, 0x00);
    writeByte(ACCEL_CONFIG, 0x00);

    // Configure FIFO for offset calculation
    writeByte(USER_CTRL, 0x40);
    writeByte(FIFO_EN, 0x78);
    delay(40);

    // Stop FIFO and read number of cycles
    writeByte(FIFO_EN, 0x00);
    readBytes(FIFO_COUNTH, 2, &data[0]);
    fifoCount = ((uint16_t)data[0] << 8) | data[1];
    cycleCount = fifoCount / 12;

    for (index = 0; index < cycleCount; index++)
    {
        int16_t gyroTemp[3] = {0, 0, 0}, accelTemp[3] = {0, 0, 0};
        readBytes(FIFO_R_W, 12, &data[0]);
        accelTemp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]);
        accelTemp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]);
        accelTemp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]);
        gyroTemp[0] = (int16_t) (((int16_t)data[6] << 8) | data[7]);
        gyroTemp[1] = (int16_t) (((int16_t)data[8] << 8) | data[9]);
        gyroTemp[2] = (int16_t) (((int16_t)data[10] << 8) | data[11]);

        accelOffset[0] += (int32_t) accelTemp[0];
        accelOffset[1] += (int32_t) accelTemp[1];
        accelOffset[2] += (int32_t) accelTemp[2];
        gyroOffset[0]  += (int32_t) gyroTemp[0];
        gyroOffset[1]  += (int32_t) gyroTemp[1];
        gyroOffset[2]  += (int32_t) gyroTemp[2];
    }

    accelOffset[0] /= (int32_t) cycleCount;
    accelOffset[1] /= (int32_t) cycleCount;
    accelOffset[2] /= (int32_t) cycleCount;
    gyroOffset[0] /= (int32_t) cycleCount;
    gyroOffset[1] /= (int32_t) cycleCount;
    gyroOffset[2] /= (int32_t) cycleCount;

    // Make sure we don't remove gravity.
    if(accelOffset[2] > 0L) {accelOffset[2] -= (int32_t) accelSensitivity;}
    else {accelOffset[2] += (int32_t) accelSensitivity;}

    data[0] = (-gyroOffset[0]/4 >> 8) & 0xFF;
    data[1] = (-gyroOffset[0]/4) & 0xFF;
    data[2] = (-gyroOffset[1]/4 >> 8) & 0xFF;
    data[3] = (-gyroOffset[1]/4) & 0xFF;
    data[4] = (-gyroOffset[2]/4 >> 8) & 0xFF;
    data[5] = (-gyroOffset[2]/4) & 0xFF;

    // Write gyro offsets.
    writeByte(XG_OFFSET_H, data[0]);
    writeByte(XG_OFFSET_L, data[1]);
    writeByte(YG_OFFSET_H, data[2]);
    writeByte(YG_OFFSET_L, data[3]);
    writeByte(ZG_OFFSET_H, data[4]);
    writeByte(ZG_OFFSET_L, data[5]);

    // Read factory accelerometer trim values. These are non-zero at startup.
    readBytes(XA_OFFSET_H, 2, &data[0]);
    accelFactory[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    readBytes(YA_OFFSET_H, 2, &data[0]);
    accelFactory[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    readBytes(ZA_OFFSET_H, 2, &data[0]);
    accelFactory[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

    // Account for temperature offset.
    for(index = 0; index < 3; index++)
    {
        if((accelFactory[index] & temperatureMask)) maskBit[index] = 0x01; // If temperature compensation bit is set, record that fact in maskBit
    }

    // Construct total accelerometer offset.
    accelFactory[0] -= (accelOffset[0]/8);
    accelFactory[1] -= (accelOffset[1]/8);
    accelFactory[2] -= (accelOffset[2]/8);

    data[0] = (accelFactory[0] >> 8) & 0xFF;
    data[1] = (accelFactory[0])      & 0xFF;
    data[1] = data[1] | maskBit[0]; // Preserve temperature compensation.
    data[2] = (accelFactory[1] >> 8) & 0xFF;
    data[3] = (accelFactory[1])      & 0xFF;
    data[3] = data[3] | maskBit[1]; // Preserve temperature compensation.
    data[4] = (accelFactory[2] >> 8) & 0xFF;
    data[5] = (accelFactory[2])      & 0xFF;
    data[5] = data[5] | maskBit[2]; // Preserve temperature compensation.

    // Write accel offsets.
    writeByte(XA_OFFSET_H, data[0]);
    writeByte(XA_OFFSET_L, data[1]);
    writeByte(YA_OFFSET_H, data[2]);
    writeByte(YA_OFFSET_L, data[3]);
    writeByte(ZA_OFFSET_H, data[4]);
    writeByte(ZA_OFFSET_L, data[5]);
}

uint8_t MPU9250::whoAmI()
{
    return readByte(WHO_AM_I);
}

void MPU9250::readAccelData(int16_t* dest)
{
  uint8_t rawData[6];
  readBytes(ACCEL_XOUT_H, 6, &rawData[0]);
  dest[0] = ((int16_t)rawData[0] << 8) | rawData[1];
  dest[1] = ((int16_t)rawData[2] << 8) | rawData[3];
  dest[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}

void MPU9250::readGyroData(int16_t* dest)
{
  uint8_t rawData[6];
  readBytes(GYRO_XOUT_H, 6, &rawData[0]);
  dest[0] = (((int16_t)rawData[0] << 8) | rawData[1]);
  dest[1] = (((int16_t)rawData[2] << 8) | rawData[3]);
  dest[2] = (((int16_t)rawData[4] << 8) | rawData[5]);
}

// ----- Private -----

// Res = Range / pow(2, Resolution-1)
// 16 bit resolution
// pow(2, 16-1) = 32768.0
void MPU9250::setGyroRes()
{
    switch (GyroScale)
    {
        case GFS_250DPS:
            gyroRes = 250.0 / 32768.0;
            break;
        case GFS_500DPS:
            gyroRes = 500.0 / 32768.0;
            break;
        case GFS_1000DPS:
            gyroRes = 1000.0 / 32768.0;
            break;
        case GFS_2000DPS:
            gyroRes = 2000.0 / 32768.0;
            break;
    }
}

// Res = Range / pow(2, Resolution-1)
// 16 bit resolution
// pow(2, 16-1) = 32768.0
void MPU9250::setAccelRes()
{
    switch (AccelScale)
    {
        case AFS_2G:
            accelRes = 2.0 / 32768.0;
            break;
        case AFS_4G:
            accelRes = 4.0 / 32768.0;
            break;
        case AFS_8G:
            accelRes = 8.0 / 32768.0;
            break;
        case AFS_16G:
            accelRes = 16.0 / 32768.0;
            break;
    }
}

// Res = Range / pow(2, Resolution-1)
// 16 bit resolution
// pow(2, 16-1) = 32768.0
void MPU9250::setMagRes()
{

}

void MPU9250::writeByte(uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void MPU9250::writeBytes(uint8_t reg, uint8_t* data, uint8_t length)
{
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(reg);

    for (int i = 0; i < length; i++)
    {
        Wire.write((uint8_t)data[i]);
    }

    Wire.endTransmission();
}

uint8_t MPU9250::readByte(uint8_t reg)
{
    uint8_t data;

    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)I2C_ADDRESS, (uint8_t) 1);
    data = Wire.read();

    return data;
}

void MPU9250::readBytes(uint8_t reg, uint8_t count, uint8_t* dest)
{
    uint8_t i = 0;

    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)I2C_ADDRESS, (uint8_t)count);

    while (Wire.available())
    {
        dest[i++] = Wire.read();
    }
}
