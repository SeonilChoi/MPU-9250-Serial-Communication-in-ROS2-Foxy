# MPU9250 Serial Communication

This work presents the serial communication between an Arduino UNO and a local PC for sending the raw data of MPU9250.

> [!NOTE]
> In this repository, this work only includes the source code for Arduino. If you want to see how the data is recieved on the local PC, click this [link](https://github.com/SeonilChoi/Quaternion-based-9-DOF-IMU-Kalman-Filter-in-ROS2-foxy.git).

# System Information

- Ubuntu 20.04
- Arduino IDE 2.3.2 linux

- Arduino UNO
- MPU9250

# Circuit Diagram

<img width="517" alt="스크린샷 2024-07-02 111250" src="https://github.com/SeonilChoi/MPU-9250-Serial-Communication/assets/172185042/b1f2e55c-3198-4ead-9105-7ead2ea03db2"> 

|제목|내용|설명|
|------|---|---|
|테스트1|테스트2|테스트3|
|테스트1|테스트2|테스트3|
|테스트1|테스트2|테스트3|

# Description

The **SerialHandler** library is used to read and write the data and set up the port.

The **MPU9250** library is used to read the IMU raw data.

The main code is **MPU9250SerialCommunication**, which transmits and recives the packet to and from the local PC.

# Usage

1. Set the permission of the port.

```
sudo chmod 777 /dev/(Arduino port name)

```

2. Compile and upload to the Arduino board.

3. Open the serial monitor and set the baudrate and close the serial monitor.
