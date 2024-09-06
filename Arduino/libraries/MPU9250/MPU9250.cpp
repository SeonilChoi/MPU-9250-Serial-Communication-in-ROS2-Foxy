#include "MPU9250.h"

#include "Arduino.h"
#include <Wire.h>
#include <stdlib.h>

MPU9250::MPU9250()
{}

MPU9250::~MPU9250()
{}

void MPU9250::setupSensor()
{
    setupMPU6500();
    setupAK8963();
}

void MPU9250::readSensor(const uint8_t * id, const uint8_t & length, uint8_t * data)
{
    uint8_t read_length = 0;
    
    for (uint8_t s = 0; s < length; s++)
    {
        switch (id[s]){
            case ID_ACCELEROMETER:
                readMPU6500(id[s], 6, data + read_length);
                read_length += 6;
                break;
            case ID_GYROSCOPES:
                readMPU6500(id[s], 6, data + read_length);
                read_length += 6;
                break;
            case ID_MAGNETOMETER:
                readAK8963(id[s], 6, data + read_length);
                read_length += 6;
                break;
            default:
                continue;
        }
    }
}

void MPU9250::setupMPU6500()
{
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    delay(10);
}

void MPU9250::setupAK8963()
{
    Wire.beginTransmission(AK_ADDR);
    Wire.write(0x0A);
    Wire.write(0x01);
    Wire.endTransmission(true);
    delay(10);
}

void MPU9250::readMPU6500(const uint8_t & id, const uint8_t & length, uint8_t * data)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(id);
    Wire.endTransmission(false);
    delay(1);

    Wire.requestFrom((uint8_t)MPU_ADDR, length);
    for (uint8_t s = 0; s < length; s++){
        data[s] = Wire.read();
    }    
    Wire.endTransmission(true); 
}

void MPU9250::readAK8963(const uint8_t & id, const uint8_t & length, uint8_t * data)
{
    Wire.beginTransmission(AK_ADDR);
    Wire.write(0x0A);
    Wire.write(0x01);
    Wire.endTransmission(true);
    delay(10);

    Wire.beginTransmission(AK_ADDR);
    Wire.write(id);
    Wire.endTransmission(false);
    delay(10);
  
    Wire.requestFrom((uint8_t)AK_ADDR, length);
    for (uint8_t s = 0; s < length / 2; s++)
    {
      data[s * 2 + 1] = Wire.read();
      data[s * 2] = Wire.read();
    }
    Wire.endTransmission(true);
}
