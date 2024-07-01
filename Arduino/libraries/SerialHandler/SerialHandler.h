#ifndef SERIALHANDLER_H_
#define SERIALHANDLER_H_

#include <stdint.h>

#define COMM_SUCCESS     0
#define COMM_WRITE_FAIL -1
#define COMM_READ_FAIL  -2

#define PKT_HEADER_0     0
#define PKT_HEADER_1     1
#define PKT_HEADER_2     2
#define PKT_INSTRUCTION  3
#define PKT_LENGTH       4
#define PKT_PARAMETER    5

#define FROM_PC      0x40
#define FROM_ARDUINO 0x41

class SerialHandler
{
public:
    SerialHandler();
    virtual ~SerialHandler();
    
    void openPort(long baudrate);
    void closePort();
    void clearPort();
    
    int writePort(uint8_t * data, uint8_t length);
    int readPort(uint8_t * data, uint8_t length);
    
private:
    long baudrate_;
    
    void setBaudrate(long baudrate);
    void setupPort(long baudrate);
    
    int writePacket(uint8_t * packet);
    int readPacket(uint8_t * packet, uint8_t length);
};

#endif
