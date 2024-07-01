#include "SerialHandler.h"
#include "MPU9250.h"

long BAUDRATE = 38400;

SerialHandler serial_handler;
MPU9250       mpu_9250;

void setup() {
  serial_handler.openPort(BAUDRATE);
  mpu_9250.setupSensor();
}

void loop() {
  uint8_t read_data[4];
  uint8_t read_length = 4;
  
  int result = serial_handler.readPort(read_data, read_length);
  if (result != COMM_SUCCESS)
    return;
  
  uint8_t imu_data[18];
  uint8_t id_length = read_data[0];
  mpu_9250.readSensor(read_data + 1, id_length, imu_data);

  result = serial_handler.writePort(imu_data, 18);
}