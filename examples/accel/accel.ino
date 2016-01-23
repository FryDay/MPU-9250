// MPU-9250 Accelerator Example
// Jordan Day

/*
=========================================
This code is placed under the MIT license
Copyright (c) 2016 Jordan Day

Please reference the LICENSE file
=========================================
*/

#include "MPU9250.h"
#define alpha 0.5f

MPU9250 imu;

int16_t accel[3];
float fXg = 0;
float fYg = 0;
float fZg = 0;

void setup()
{
    Serial.begin(9600);
    imu.calibrate();
    imu.init();
}

void loop()
{
    float pitch, roll, fX, fY, fZ;

    imu.readAccelData(&accel[0]);

    fX = (float)accel[0] * imu.accelRes;
    fY = (float)accel[1] * imu.accelRes;
    fZ = (float)accel[2] * imu.accelRes;

    fXg = fX * alpha + (fXg * (1.0 - alpha));
    fYg = fY * alpha + (fYg * (1.0 - alpha));
    fZg = fZ * alpha + (fZg * (1.0 - alpha));

    // Print g force
    /*Serial.print("X: ");
    Serial.print(fX);
    Serial.print(" Y: ");
    Serial.print(fY);
    Serial.print(" Z: ");
    Serial.println(fZ);*/

    roll = atan2(fXg, fZg) * 180.0 / M_PI;
    pitch = atan2(fYg, fZg) * 180.0 / M_PI;

    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print("\tPitch: ");
    Serial.println(pitch);

    delay(50);
}
