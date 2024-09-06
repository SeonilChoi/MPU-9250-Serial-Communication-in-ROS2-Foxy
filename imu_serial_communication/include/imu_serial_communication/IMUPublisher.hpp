#ifndef IMU_PUBLISHER_HPP_
#define IMU_PUBLISHER_HPP_

#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

#include "imu_serial_communication/PortHandler.hpp"

class IMUPublisher : public rclcpp::Node
{
public:
    using Imu = sensor_msgs::msg::Imu;
    using MagneticField = sensor_msgs::msg::MagneticField;

    IMUPublisher();
    virtual ~IMUPublisher();

private:
    rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<MagneticField>::SharedPtr mag_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback();

    void convertRaw2Imu(const std::vector<uint8_t> & raw, std::vector<int16_t> & data);
    void convertData2ImuMsg(const int16_t * data);
    void convertData2MagMsg(const int16_t * data);

    const bool MPU6050_;
    const bool AK8963_;

    std::vector<uint8_t> write_data_;
    uint8_t write_length_;

    std::unique_ptr<PortHandler> imu_port_;
};

#endif
