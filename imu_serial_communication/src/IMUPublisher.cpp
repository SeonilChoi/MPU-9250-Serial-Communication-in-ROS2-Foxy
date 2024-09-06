#include <chrono>
#include <cmath>
#include <termios.h>

#include "imu_serial_communication/IMUPublisher.hpp"

#define IMU_DEVICE "/dev/ttyACM0"

using namespace std::chrono_literals;

IMUPublisher::IMUPublisher() : Node("imu_publisher"),
    MPU6050_([this](){
        this->declare_parameter("MPU6050", true);
        return this->get_parameter("MPU6050").get_value<bool>();
    }()),
    AK8963_([this](){
        this->declare_parameter("AK8963", true);
        return this->get_parameter("AK8963").get_value<bool>();
    }())
{
    RCLCPP_INFO(this->get_logger(), "[IMUPublisher::IMUPublisher] Run node.");

    if (MPU6050_ && AK8963_){
        write_length_ = 4;
        write_data_ = {3, 59, 67, 3};
    }
    else if (MPU6050_){
        write_length_ = 3;
        write_data_ = {2, 59, 67};
    }
    else if (AK8963_){
        write_length_ = 2;
        write_data_ = {1, 3};
    }
    else{
        RCLCPP_INFO(this->get_logger(), "[IMUPublisher::IMUPublisher] No read data.");
        return;
    }

    const auto QOS_BEKL5V = rclcpp::QoS(rclcpp::KeepLast(5))
        .best_effort().durability_volatile();

    imu_pub_ = this->create_publisher<Imu>("imu/data_raw", QOS_BEKL5V);
    mag_pub_ = this->create_publisher<MagneticField>("mag/data_raw", QOS_BEKL5V);
    timer_ = this->create_wall_timer(1ms, std::bind(&IMUPublisher::timer_callback, this));

    imu_port_ = std::make_unique<PortHandler>(IMU_DEVICE);
    imu_port_->openPort(B38400);
}

IMUPublisher::~IMUPublisher()
{
    imu_port_->closePort();
}

void IMUPublisher::timer_callback()
{   
    int result = imu_port_->writePort(write_data_, write_length_);
    
    if (result != COMM_SUCCESS)
        return;

    const uint8_t read_length = (write_length_ - 1) * 6;
    std::vector<uint8_t> read_data(read_length);
    result = imu_port_->readPort(read_data, read_length);
    
    if (result != COMM_SUCCESS){
        return;    
    }

    std::vector<int16_t> imu_data((write_length_ - 1) * 3);
    convertRaw2Imu(read_data, imu_data);
    
    if (MPU6050_ && AK8963_){
        convertData2ImuMsg(imu_data.data());
        convertData2MagMsg(imu_data.data()+6);
    } else if(MPU6050_){
        convertData2ImuMsg(imu_data.data());
    } else if(AK8963_){
        convertData2MagMsg(imu_data.data());
    }
}

void IMUPublisher::convertRaw2Imu(const std::vector<uint8_t> & raw, std::vector<int16_t> & data)
{
    for (uint8_t s = 0; s < (write_length_ - 1) * 3; s++)
    {
        data[s] = (static_cast<int16_t>(raw[s*2]) << 8) | raw[s*2+1];
    }
}

void IMUPublisher::convertData2ImuMsg(const int16_t * data)
{
    Imu imu_msg;
    
    imu_msg.linear_acceleration.x = (*(data + 0) / 16384.0 + 0.016748) * 9.80665;
    imu_msg.linear_acceleration.y = (*(data + 1) / 16384.0 - 0.008024) * 9.80665;
    imu_msg.linear_acceleration.z = (*(data + 2) / 16384.0 - 0.012744) * 9.80665;

    imu_msg.angular_velocity.x = (*(data + 3) / 131.0 - 0.4208451271) / 180.0 * M_PI;
    imu_msg.angular_velocity.y = (*(data + 4) / 131.0 + 0.6887240409) / 180.0 * M_PI;
    imu_msg.angular_velocity.z = (*(data + 5) / 131.0 + 0.7604086399) / 180.0 * M_PI;
    
    imu_msg.header.stamp = this->get_clock()->now();
    
    imu_pub_->publish(imu_msg);
}

void IMUPublisher::convertData2MagMsg(const int16_t * data)
{
    MagneticField mag_msg;
    
    mag_msg.magnetic_field.x = *(data + 0) * 1200.0 / 4096.0 - 6.0;
    mag_msg.magnetic_field.y = *(data + 1) * 1200.0 / 4096.0 - 24.0;
    mag_msg.magnetic_field.z = *(data + 2) * 1200.0 / 4096.0 + 3.0;
    
    mag_pub_->publish(mag_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto imu_node = std::make_shared<IMUPublisher>();
    rclcpp::spin(imu_node);
    rclcpp::shutdown();

    return 0;
}
