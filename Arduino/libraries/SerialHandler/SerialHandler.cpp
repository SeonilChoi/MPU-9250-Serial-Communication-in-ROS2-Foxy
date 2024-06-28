#include "SerialHandler.h"

#include "Arduino.h"
#include <stdlib.h>

SerialHandler::SerialHandler()
{}

SerialHandler::~SerialHandler()
{
    closePort();
}

void SerialHandler::openPort(const int baudrate)
{
    setBaudrate(baudrate);
}

void SerialHandler::closePort()
{
    Serial.end();
}

void SerialHandler::clearPort()
{
    while (Serial.available() > 0)
    {
        Serial.read();
    }
}

void SerialHandler::setBaudrate(const int baudrate)
{
    closePort();
    
    baudrate_ = baudrate;
    setupPort();
}

void SerialHandler::setupPort()
{
    Serial.begin(baudrate_);
    delay(2000);
}

int SerialHandler::writePort(uint8_t * data, uint8_t length)
{
    int result = COMM_WRITE_FAIL;

    uint8_t * packet = new uint8_t[length + 5];

    packet[PKT_HEADER_0]     = 0xFF;
    packet[PKT_HEADER_1]     = 0xFF;
    packet[PKT_HEADER_2]     = 0xFD;
    packet[PKT_INSTRUCTION]  = FROM_ARDUINO;
    packet[PKT_LENGTH]       = length;

    for (uint8_t s = 0; s < length; s++)
        packet[PKT_PARAMETER + s] = data[s];
    
    result = writePacket(packet);

    delete[] packet;

    return result;
}

int SerialHandler::writePacket(uint8_t * packet)
{
    size_t written_packet_length = 0;
    size_t total_packet_length   = static_cast<size_t>(packet[PKT_LENGTH]) + 5;

    clearPort();
    
    if(Serial.availableForWrite() < total_packet_length)
        return COMM_WRITE_FAIL;
    
    written_packet_length = Serial.write(packet, total_packet_length);

    if (written_packet_length != total_packet_length)
        return COMM_WRITE_FAIL;
        
    return COMM_SUCCESS;
}

int SerialHandler::readPort(uint8_t * data, uint8_t length)
{
    int result = COMM_READ_FAIL;
    
    uint8_t * packet = new uint8_t[length + 5];  
    
    do{
        result = readPacket(packet, length);
    }while(result != COMM_SUCCESS);
    
    if (result == COMM_SUCCESS)
    {
        for (uint8_t s = 0; s < length; s++)
            data[s] = packet[PKT_PARAMETER + s];
    }
    
    delete[] packet;

    return result;
}

int SerialHandler::readPacket(uint8_t * packet, uint8_t length)
{
    int result = COMM_READ_FAIL;
    
    if (Serial.availableForWrite() <= 0)
        return result;
    
    size_t read_length = 0;
    size_t wait_length = length + 5;
    
    while (true)
    {
        size_t read_bytes = Serial.readBytes(packet + read_length, wait_length - read_length);
        if (read_bytes < 0){
            delay(1);
            continue;
        }

        read_length += read_bytes;
        
        if (read_length == wait_length){
            size_t idx = 0;
            for (idx = 0; idx < read_length - 3; idx++)
            {
                if ((packet[idx] == static_cast<uint8_t>(0xFF)) && (packet[idx + 1] == static_cast<uint8_t>(0xFF)) && (packet[idx + 2] == static_cast<uint8_t>(0xFD)) && (packet[idx + 3] == static_cast<uint8_t>(FROM_PC)))
                    break;
            }
            
            if (idx == 0){
                result = COMM_SUCCESS;
                break;
            }
            else{
                for (size_t s = 0; s < read_length - idx; s++)
                    packet[s] = packet[s + idx];
                read_length -= idx;
            }
        }
    }
    return result;
}
