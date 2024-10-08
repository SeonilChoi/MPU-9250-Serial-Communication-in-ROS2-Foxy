#include "imu_serial_communication/PortHandler.hpp"

#include <iostream>

#include <stdlib.h>
#include <string.h>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

PortHandler::PortHandler(const char * port_name)
    : socket_fd_(-1)
{
    setPortName(port_name);
}

PortHandler::~PortHandler()
{
    closePort();
}

bool PortHandler::openPort(const int & baudrate)
{
    return setBaudrate(baudrate);   
}

void PortHandler::closePort()
{
    if (socket_fd_ != -1){
        std::cout << "[PortHandler::cloesPort] close the port." << std::endl;
        close(socket_fd_);
    }
    socket_fd_ = -1;
}

void PortHandler::clearPort()
{
    tcflush(socket_fd_, TCIFLUSH);
}

void PortHandler::setPortName(const char * port_name)
{
    strcpy(port_name_, port_name);
}

bool PortHandler::setBaudrate(const int & baudrate)
{
    closePort();

    baudrate_ = baudrate;
    return setupPort(baudrate);
}

bool PortHandler::setupPort(const int & baudrate)
{
    socket_fd_ = open(port_name_, O_RDWR | O_NOCTTY);
    
    if (socket_fd_ < 0)
        return false;
    
    struct termios tty;

    memset(&tty, 0, sizeof tty);

    if (tcgetattr(socket_fd_, &tty) != 0){
        close(socket_fd_);
        return false;
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);
    
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(socket_fd_, TCSANOW, &tty) != 0) {
        close(socket_fd_);
        return -1;
    }
    
    tcflush(socket_fd_, TCIFLUSH);

    usleep(5000000);

    return true;
}

int PortHandler::writePort(const std::vector<uint8_t> & data, const uint8_t & length)
{
    int result = COMM_WRITE_FAIL;

    std::vector<uint8_t> packet(length + 5);
    
    packet[PKT_HEADER_0]    = 0xFF;
    packet[PKT_HEADER_1]    = 0xFF;
    packet[PKT_HEADER_2]    = 0xFD;
    packet[PKT_INSTRUCTION] = FROM_PC;
    packet[PKT_LENGTH]      = length;

    for (uint8_t s = 0; s < length; s++)
        packet[PKT_PARAMETER + s] = data[s];

    result = writePacket(packet);
    
    return result;
}

int PortHandler::writePacket(const std::vector<uint8_t> & packet)
{
    ssize_t written_packet_length = 0;
    ssize_t total_packet_length = static_cast<ssize_t>(packet[PKT_LENGTH]) + 5;

    clearPort();
    
    written_packet_length = write(socket_fd_, packet.data(), total_packet_length);
    
    if (written_packet_length != total_packet_length){
        return COMM_WRITE_FAIL;
    }
    
    return COMM_SUCCESS;
}

int PortHandler::readPort(std::vector<uint8_t> & data, const uint8_t & length)
{
    int result = COMM_READ_FAIL;

    std::vector<uint8_t> packet(length + 5);
    
    do {
        result = readPacket(packet, length);
        if (result == COMM_READ_TIMEOUT)
            break;
    }while(result != COMM_SUCCESS);

    if (result == COMM_SUCCESS){
        for (uint8_t s = 0; s < length; s++)
            data[s] = packet[PKT_PARAMETER + s];
    }
    
    return result;
}

int PortHandler::readPacket(std::vector<uint8_t> & packet, const uint8_t & length)
{
    int result = COMM_READ_FAIL;
    
    ssize_t read_length = 0;
    ssize_t wait_length = length + 5;
    
    const int timeout_sec = 3;
    fd_set read_fds;
    struct timeval timeout;
    
    while (true)
    {        
        
        FD_ZERO(&read_fds);
        FD_SET(socket_fd_, &read_fds);
        
        timeout.tv_sec = timeout_sec;
        timeout.tv_usec = 0;
        
        int select_result = select(socket_fd_ + 1, &read_fds, NULL, NULL, &timeout);
        if (select_result == -1){
            return COMM_READ_FAIL;
        } else if (select_result == 0){
            return COMM_READ_TIMEOUT;
        }
        
        ssize_t read_bytes = read(socket_fd_, &packet[read_length], wait_length - read_length);
        if (read_bytes < 0){
            if (errno == EAGAIN || errno == EWOULDBLOCK){
                usleep(1000);
                continue;
            } else {
                return COMM_READ_FAIL;
            }
        }

        read_length += read_bytes;
        
        if (read_length == wait_length){
            ssize_t idx = 0;
            for (idx = 0; idx < read_length - 3; idx++)
            {
                if ((packet[idx] == 0xFF) && (packet[idx + 1] == 0xFF) && (packet[idx + 2] == 0xFD) && (packet[idx + 3] == FROM_ARDUINO))
                    break;
            }

            if (idx == 0){
                result = COMM_SUCCESS;
                break;
            }
            else{
                for (ssize_t s = 0; s < read_length - idx; s++)
                    packet[s] = packet[s + idx];
                read_length -= idx;
            }
        }
    }

    return result;
}
