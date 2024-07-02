# MPU9250 Serial Communication

This work presents the serial communication between an **Arduino UNO** and a **local PC** for sending the raw data of MPU9250.

<p align = "center">
  <img src= "https://github.com/SeonilChoi/MPU-9250-Serial-Communication/assets/172185042/9b19c336-c4d5-4ed5-a7b5-d4651b853bc4"/>
</p>

# System Information

- Ubuntu 20.04
- Arduino IDE 2.3.2 linux
- ROS2 Foxy

- Arduino UNO
- MPU9250

# Circuit Diagram

<table>
    <tr>
        <td align = "center">
          <img src="https://github.com/SeonilChoi/MPU-9250-Serial-Communication/assets/172185042/ac212186-184a-4530-98da-9cd03c5de7ff" width = 1500>
        </td>
        <td>
            <table>
                <tr>
                    <th><strong>UNO</strong></th>
                    <th><strong>MPU9250</strong></th>
                </tr>
                <tr>
                    <td>VCC</td>
                    <td>VCC</td>
                </tr>
                <tr>
                    <td>GND</td>
                    <td>GND</td>
                </tr>
                <tr>
                    <td>A4</td>
                    <td>SDA, EDA</td>
                </tr>
                <tr>
                    <td>A5</td>
                    <td>SCL, ECL</td>
                </tr>
                 <tr>
                    <td>INT</td>
                    <td>2</td>
                </tr>
            </table>
        </td>
    </tr>
</table>

# Description

## Arduino

The **SerialHandler** library is used to read and write the data and set up the port.

The **MPU9250** library is used to read the IMU raw data.

The main code is **MPU9250SerialCommunication**, which transmits and recives the packet to and from the local PC.

## Local PC

The data from the IMU, which consists of accelerations, angular velocities, and magnetic fields, is published by the node named **IMUPublisher**.

This package reads raw data from the IMU using a library named **PortHandler**.

# Usage

## Arduino

1. Set the permission of the port.

```
sudo chmod 777 /dev/(Arduino port name)

```

2. Compile and upload to the Arduino board.

3. Open the serial monitor and set the baudrate and close the serial monitor.

## ROS2

### Build

```
colcon build --packages-select imu_serial_communication

```

### Run

```
ros2 launch imu_serial_communication imu_serial_communication.launch.py
```
