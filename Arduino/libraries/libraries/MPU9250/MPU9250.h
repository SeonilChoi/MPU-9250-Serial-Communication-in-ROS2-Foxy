#ifndef MPU_9250_H_
#define MPU_9250_H_

#define ID_ACCELELOMETER 0x

#define MPU_ADDR 0x68
#define AK_ADDR  0x0C

#define ID_ACCELEROMETER 59
#define ID_GYROSCOPES    67
#define ID_MAGNETOMETER  3

#include <stdint.h>

class MPU9250
{
public:
    MPU9250();
    virtual ~MPU9250();

    void setupSensor();
    void readSensor(uint8_t * id, uint8_t length, uint8_t * data);
    
private:
    void setupMPU6500();
    void setupAK8963();

    void readMPU6500(uint8_t id, uint8_t length, uint8_t * data);
    void readAK8963(uint8_t id, uint8_t length, uint8_t * data);
};

#endif
