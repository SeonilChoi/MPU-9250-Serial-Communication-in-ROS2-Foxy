from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    imu_publisher = Node(
        package = "imu_serial_communication",
        executable = "imu_publisher",
        name = "imu_publisher",
        parameters = [{
            "MPU6050":True,
            "AK8963":True
        }]
    )
    
    return LaunchDescription([
        imu_publisher
    ])
    
