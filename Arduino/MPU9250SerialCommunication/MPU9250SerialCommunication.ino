#include "SerialHandler.h"
#include "MPU9250.h"

const long BAUDRATE = 38400;
const uint8_t max_read_length = 4; // ID_LENGTH ACCELEROMETER GYROSCOPE MAGENTOMETER

SerialHandler serial_handler;
MPU9250       mpu_9250;

void setup() {
  serial_handler.openPort(BAUDRATE);
  delay(3000);
  mpu_9250.setupSensor();
}

void loop() {
  uint8_t read_data[max_read_length] = {0, 0, 0, 0};

  int result = serial_handler.readPort(read_data, max_read_length);
  if (result != COMM_SUCCESS)
    return;
  
  const uint8_t id_length = read_data[0];
  uint8_t * imu_data = new uint8_t[id_length * 6];
  mpu_9250.readSensor(read_data + 1, id_length, imu_data);

  result = serial_handler.writePort(imu_data, id_length * 6);
  
  delete[] imu_data;
}
