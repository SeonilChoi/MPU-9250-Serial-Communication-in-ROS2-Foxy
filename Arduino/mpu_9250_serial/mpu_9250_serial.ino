#include <stdint.h>
#include <stdlib.h>
#include <Wire.h>

#define MPU_ADDR 0x68
#define AK_ADDR 0x0C

#define BROADCAST_ID 0xFE

#define INST_READ 2
#define INST_STATUS 85

#define COMM_SUCCESS        0
#define COMM_PORT_BUSY     -1000
#define COMM_TX_FAIL       -1001
#define COMM_RX_FAIL       -1002
#define COMM_TX_ERROR      -2000
#define COMM_RX_WAITING    -3000
#define COMM_RX_TIMEOUT    -3001
#define COMM_RX_CORRUPT    -3002
#define COMM_NOT_AVAILABLE -9000

#define PKT_HEADER0 0
#define PKT_HEADER1 1
#define PKT_HEADER2 2
#define PKT_RESERVED 3
#define PKT_ID 4
#define PKT_LENGTH_L 5
#define PKT_LENGTH_H 6
#define PKT_INSTRUCTION 7
#define PKT_PARAMETER0 8
#define PKT_ERROR 8

#define TTR_MAKEWORD(a, b)      ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define TTR_MAKEWORDL(a, b)     ((uint32_t)(((uint16_t)(((uint64_t)(a)) 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))
#define TTR_LOWORD(l)           ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define TTR_HIWORD(l)           ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define TTR_LOBYTE(w)           ((uint8_t)(((uint64_t)(w)) & 0xff))
#define TTR_HIBYTE(w)           ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

#define TXPACKET_MAX_LEN     (1*1024)
#define RXPACKET_MAX_LEN     (1*1024)

uint8_t mpu_id_array[6] = {59, 61, 63, 67, 69, 71}; 
uint8_t ak_id_array[3] = {3, 5, 7};

byte serial_length = 0;

unsigned short updateCRC(uint16_t crc_accum, uint8_t * data_blk_ptr, uint16_t data_blk_size)
{
  uint16_t i;
  static const uint16_t crc_table[256] = {0x0000,
  0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
  0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
  0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
  0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
  0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
  0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
  0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
  0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
  0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
  0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
  0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
  0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
  0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
  0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
  0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
  0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
  0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
  0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
  0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
  0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
  0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
  0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
  0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
  0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
  0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
  0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
  0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
  0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
  0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
  0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
  0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
  0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
  0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
  0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
  0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
  0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
  0x820D, 0x8207, 0x0202 };

  for (uint16_t j = 0; j < data_blk_size; j++)
  {
      i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
      crc_accum = (crc_accum << 8) ^ crc_table[i];
  }
  return crc_accum;
}

void addStuffing(uint8_t * packet)
{
  int packet_length_in  = TTR_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
  int packet_length_out = packet_length_in;

  if (packet_length_in < 8)                                                        // 데이터가 짧아서 할 필요가 없음
    return;

  uint8_t * packet_ptr;
  uint16_t  packet_length_before_crc = packet_length_in - 2;

  for (uint16_t i = 3; i < packet_length_before_crc; i++)                          // 자신 보다 뒤에 있는 데이터가 3개 이상인 경우만 보겠다
  {
    packet_ptr = &packet[i + PKT_INSTRUCTION - 2];
    if (packet_ptr[0] == 0xFF && packet_ptr[1] == 0xFF && packet_ptr[2] == 0xFD)   // param 부분에 헤더 flag가 있으면 stuffing 을 하기 위해 데이터 길이를 증가시켜줌
        packet_length_out++; 
  }

  if (packet_length_in == packet_length_out)                                       // param 부분에 헤더 flag가 없다
    return;

  uint16_t out_index = packet_length_out + 6 - 2;                                  // crc 전까지의 인덱스
  uint16_t in_index  = packet_length_in + 6 - 2;

  while (out_index != in_index)
  {
    if (packet[in_index] == 0xFD && packet[in_index - 1] == 0xFF && packet[in_index - 2] == 0xFF)
    {
      packet[out_index--] = 0xFD;                                              // 0xFD 로 Stuffing !!!
      if (out_index != in_index)                                               // Stuffing 할게 남아 있으면
      {
        packet[out_index--] = packet[in_index--];                            // 0xFD
        packet[out_index--] = packet[in_index--];                            // 0xFF
        packet[out_index--] = packet[in_index--];                            // 0xFF
      }
    }
    else
    {
      packet[out_index--] = packet[in_index--];                                
    }
  }

  packet[PKT_LENGTH_L] = TTR_LOBYTE(packet_length_out);                            // Stuffing 으로 인해 늘어난 length로 변경
  packet[PKT_LENGTH_H] = TTR_HIBYTE(packet_length_out);

  return;
}

void removeStuffing(uint8_t * packet)
{
  int i     = 0;
  int index = 0;

  int packet_length_in  = TTR_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
  int packet_length_out = packet_length_in;

  index = PKT_INSTRUCTION;

  for (i = 0; i < packet_length_in - 2; i++)  // crc 제외
  {
    if (packet[i + PKT_INSTRUCTION] == 0xFD && packet[i + PKT_INSTRUCTION + 1] == 0xFD && packet[i + PKT_INSTRUCTION - 1] == 0xFF && packet[i + PKT_INSTRUCTION - 2] == 0xFF)
    {
      // 0xFF 0xFF 0xFD 0xFD
      packet_length_out--;
      i++;                                // 건너뛰기
    }
    packet[index++] = packet[i + PKT_INSTRUCTION];
  }
  packet[index++] = packet[PKT_INSTRUCTION + packet_length_in - 2];
  packet[index++] = packet[PKT_INSTRUCTION + packet_length_in - 1];

  packet[PKT_LENGTH_L] = TTR_LOBYTE(packet_length_out);
  packet[PKT_LENGTH_H] = TTR_HIBYTE(packet_length_out);
}

int txPacket(uint8_t * txpacket)
{
  uint16_t total_packet_length = 0;
  uint16_t written_packet_length = 0;

  addStuffing(txpacket);

  total_packet_length = TTR_MAKEWORD(txpacket[PKT_LENGTH_L], txpacket[PKT_LENGTH_H]) + 7;

  txpacket[PKT_HEADER0] = 0xFF;
  txpacket[PKT_HEADER1] = 0xFF;
  txpacket[PKT_HEADER2] = 0xFD;
  txpacket[PKT_RESERVED] = 0x00;

  uint16_t crc = updateCRC(0, txpacket, total_packet_length - 2);
  txpacket[total_packet_length - 2] = TTR_LOBYTE(crc);
  txpacket[total_packet_length - 1] = TTR_HIBYTE(crc);
  
  Serial.flush();

  written_packet_length = Serial.write(txpacket, total_packet_length);

  if (written_packet_length != total_packet_length)
  {
      return COMM_TX_FAIL;
  }
  return COMM_SUCCESS;
}

int rxPacket(uint8_t * rxpacket)
{
  int result = COMM_RX_FAIL;

  byte rx_length = 0;
  byte wait_length = 20;

  while (true)
  {
    rx_length += Serial.readBytes(&rxpacket[rx_length], wait_length - rx_length);

    if (rx_length >= wait_length)
    {
      byte idx = 0;
      for (idx = 0; idx < rx_length - 3; idx++)
      {
        if ((rxpacket[idx] == 0xFF) && (rxpacket[idx + 1] == 0xFF) && (rxpacket[idx + 2] == 0xFD) && (rxpacket[idx + 3] != 0xFD))
          break;
      }

      if (idx == 0)
      {
        if ((rxpacket[PKT_RESERVED] != 0x00) || (TTR_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) > RXPACKET_MAX_LEN))
        {
          for (uint16_t s = 0; s < rx_length - 1; s++)
            rxpacket[s] = rxpacket[s + 1];
          rx_length -= 1;
          continue;
        }

        if (wait_length != TTR_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) + 7)
        {
          wait_length = TTR_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) + 7;
          continue;
        }

        if (rx_length < wait_length)
        {
          continue;
        }

        uint16_t crc = TTR_MAKEWORD(rxpacket[wait_length - 2], rxpacket[wait_length - 1]);
        if (updateCRC(0, rxpacket, wait_length - 2) == crc)
          result = COMM_SUCCESS;
        else 
          result = COMM_RX_CORRUPT;
        break;
      }
      else
      {
        for (uint16_t s = 0; s < rx_length - idx; s++)
          rxpacket[s] = rxpacket[s + idx];
        rx_length -= idx;
      }
    }
    delay(0);
  }

  if (result == COMM_SUCCESS)
    removeStuffing(rxpacket);

  Serial.end();
  Serial.begin(1000000);

  return result;
}

int readRx(uint8_t id, uint16_t length, uint8_t * data, uint8_t * error)
{
    int result = COMM_RX_FAIL;
    uint8_t * rxpacket = (uint8_t *)malloc(20);

    if (rxpacket == NULL)
    {  
      return result;
    }
    
    do{
        result = rxPacket(rxpacket);
    }while(result != COMM_SUCCESS && rxpacket[PKT_ID] != id);
  
    if (result == COMM_SUCCESS && rxpacket[PKT_ID] == id)
    {
        if (error != 0)
            *error = (uint8_t)rxpacket[PKT_ERROR];
        
        for (uint16_t s = 0; s < length; s++)
            data[s] = rxpacket[PKT_PARAMETER0 + 1 + s];
    }

    free(rxpacket);

    return result;
}

int writeTxOnly(uint8_t id, uint16_t length, uint8_t * data)
{
    int result = COMM_TX_FAIL;

    uint8_t * txpacket = (uint8_t *)malloc(length + 12 + (length / 3));

    if (txpacket == NULL)
      return result;

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH_L]      = TTR_LOBYTE(length + 4);
    txpacket[PKT_LENGTH_H]      = TTR_HIBYTE(length + 4);
    txpacket[PKT_INSTRUCTION]   = INST_STATUS;
    txpacket[PKT_ERROR]         = 0;

    for (uint16_t s = 0; s < length; s++)
        txpacket[PKT_PARAMETER0 + s + 1] = data[s];

    result = txPacket(txpacket);

    free(txpacket);
    return result;
}

int write2ByteTxOnly(uint8_t id, uint16_t data)
{
    uint8_t data_write[2] = {TTR_LOBYTE(data), TTR_HIBYTE(data)};
    return writeTxOnly(id, 2, data_write);
}

bool MPUReadWrite(int8_t address, uint8_t id)
{
  int16_t data;
  uint16_t u_data;

  Wire.beginTransmission(address);
  Wire.write(id);
  Wire.endTransmission(false);
  delay(1);

  Wire.requestFrom(address, 2, true);
  data = ((int16_t)Wire.read() << 8) | Wire.read();
  Wire.endTransmission(true);

  u_data = data;
  bool result = write2ByteTxOnly(id, u_data);
  
  return result;
}

bool AKReadWrite(int8_t address, uint8_t id)
{
  int16_t data;
  uint16_t u_data;

  Wire.beginTransmission(AK_ADDR);
  Wire.write(0x0A);
  Wire.write(0x01);
  Wire.endTransmission(true);
  delay(10);

  Wire.beginTransmission(address);
  Wire.write(id);
  Wire.endTransmission(false);
  delay(10);

  Wire.requestFrom(address, 2, true);
  data = Wire.read() | ((int16_t) Wire.read() << 8);
  Wire.endTransmission(true);

  u_data = data;
  bool result = write2ByteTxOnly(id, u_data);
  
  return result;
}

bool SensorReadTxWrite()
{
  Wire.beginTransmission(AK_ADDR);
  Wire.write(0x0A);
  Wire.write(0x01);
  Wire.endTransmission(true);
  delay(10);

  Wire.beginTransmission(AK_ADDR);
  Wire.write(ak_id_array[0]);
  Wire.endTransmission(false);
  delay(10);

  Wire.requestFrom(AK_ADDR, 6, true);
  int16_t mag_data[3];
  mag_data[0] = Wire.read() | ((int16_t) Wire.read() << 8);
  mag_data[1] = Wire.read() | ((int16_t) Wire.read() << 8);
  mag_data[2] = Wire.read() | ((int16_t) Wire.read() << 8);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(mpu_id_array[0]);
  Wire.endTransmission(false);
  delay(1);

  Wire.requestFrom(MPU_ADDR, 6, true);
  int16_t acc_data[3];
  acc_data[0] = ((int16_t)Wire.read() << 8) | Wire.read();
  acc_data[1] = ((int16_t)Wire.read() << 8) | Wire.read();
  acc_data[2] = ((int16_t)Wire.read() << 8) | Wire.read();
  Wire.endTransmission(true); 

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(mpu_id_array[3]);
  Wire.endTransmission(false);
  delay(1);

  Wire.requestFrom(MPU_ADDR, 6, true);
  int16_t gyro_data[3];
  gyro_data[0] = ((int16_t)Wire.read() << 8) | Wire.read();
  gyro_data[1] = ((int16_t)Wire.read() << 8) | Wire.read();
  gyro_data[2] = ((int16_t)Wire.read() << 8) | Wire.read();
  Wire.endTransmission(true);

  uint8_t data_write[18] = {TTR_LOBYTE((uint16_t)acc_data[0]), TTR_HIBYTE((uint16_t)acc_data[0]), TTR_LOBYTE((uint16_t)acc_data[1]), TTR_HIBYTE((uint16_t)acc_data[1]), TTR_LOBYTE((uint16_t)acc_data[2]), TTR_HIBYTE((uint16_t)acc_data[2]),
                            TTR_LOBYTE((uint16_t)gyro_data[0]), TTR_HIBYTE((uint16_t)gyro_data[0]), TTR_LOBYTE((uint16_t)gyro_data[1]), TTR_HIBYTE((uint16_t)gyro_data[1]), TTR_LOBYTE((uint16_t)gyro_data[2]), TTR_HIBYTE((uint16_t)gyro_data[2]),
                            TTR_LOBYTE((uint16_t)mag_data[0]), TTR_HIBYTE((uint16_t)mag_data[0]), TTR_LOBYTE((uint16_t)mag_data[1]), TTR_HIBYTE((uint16_t)mag_data[1]), TTR_LOBYTE((uint16_t)mag_data[2]), TTR_HIBYTE((uint16_t)mag_data[2])};
  bool result = writeTxOnly(BROADCAST_ID, 18, data_write);
  return result;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  
  // MPU Setup
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // AK Setup
  Wire.beginTransmission(AK_ADDR);
  Wire.write(0x0A);
  Wire.write(0x01);
  Wire.endTransmission(true);
  delay(10);
}


void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() == 0)
    return;

  int result = COMM_RX_FAIL;
  uint8_t data[9];

  result = readRx(BROADCAST_ID, 9, data, 0);
  
  if (result != COMM_SUCCESS)
    return;

  result = COMM_TX_FAIL;
  result = SensorReadTxWrite();
  //result = MPUReadWrite(MPU_ADDR, mpu_id_array[0]);
  
  if (result != COMM_SUCCESS)
    return;
}

/*
void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() == 0)
  {
    return;
  }

  int result = COMM_RX_FAIL;
  uint8_t data[9];

  result = readRx(BROADCAST_ID, 9, data, 0);
  
  if (result != COMM_SUCCESS)
    return;

  result = COMM_TX_FAIL;

  for (uint8_t s = 0; s < 6; s++)
  {
    result = MPUReadWrite(MPU_ADDR, mpu_id_array[s]);
    if (result != COMM_SUCCESS)
      return;
  }
  for (uint8_t s = 0; s < 3; s++)
  {
    result = AKReadWrite(AK_ADDR, ak_id_array[s]);
    if (result != COMM_SUCCESS)
      return;
  }
}
*/