// MPU-9250 Gyro Example
// Jordan Day

/*
=========================================
This code is placed under the MIT license
Copyright (c) 2016 Jordan Day

Please reference the LICENSE file
=========================================
*/

#include "MPU9250.h"

MPU9250 imu;

int16_t gyro[3];

void setup()
{
    Serial.begin(9600);
    imu.calibrate();
    imu.init();
}

void loop()
{
    imu.readGyroData(&gyro[0]);

    // Display pitch in degrees per second
    Serial.print("Pitch:");
    if (gyro[0] >= 0)
        Serial.print("+");
    Serial.print(gyro[0] / 57.14286, 0);

    if (gyro[0]/57.14286 - 2 > 0)
        Serial.print(" NoU\t");
    else if (gyro[0] / 57.14286 + 2 < 0)
        Serial.print(" NoD\t");
    else
        Serial.print(" ---\t");

    // Display roll in degrees per second
    Serial.print("  Roll:");
    if (gyro[1] >= 0)
        Serial.print("+");

    Serial.print(gyro[1]/57.14286, 0);

    if (gyro[1]/57.14286 - 2 > 0)
        Serial.print(" RwD\t");
    else if (gyro[1]/57.14286 + 2 < 0)
        Serial.print(" RwU\t");
    else
        Serial.print(" ---\t");

    // Display yaw in degrees per second
    Serial.print("  Yaw:");
    if (gyro[2] >= 0)
        Serial.print("+");

    Serial.print(gyro[2] / 57.14286, 0);

    if (gyro[2] / 57.14286 - 2 > 0)
        Serial.println(" NoR\t");
    else if (gyro[2] / 57.14286 + 2 < 0)
        Serial.println(" NoL\t");
    else
        Serial.println(" ---\t");

    delay(50);
}
