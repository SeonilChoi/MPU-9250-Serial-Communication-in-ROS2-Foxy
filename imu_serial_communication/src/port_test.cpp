#include "imu_serial_communication/PortHandler.hpp"

#include <memory>
#include <vector>
#include <iostream>
#include <cstdio>
#include <termios.h>

int main()
{
    std::unique_ptr<PortHandler> imu_port;
    imu_port = std::make_unique<PortHandler>("/dev/ttyACM0");

    imu_port->openPort(B38400);

    std::vector<uint8_t> data = {2, 59, 67};

    int result = imu_port->writePort(data, 3);    
    std::cout << result << std::endl;    
    
    std::vector<uint8_t> read_data(12);
    result = imu_port->readPort(read_data, 12);
    
    std::cout << result << std::endl;

    for (const auto & val : read_data)
        printf("%d\n", val);

    return 0; 
}
