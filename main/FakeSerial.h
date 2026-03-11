/*** 
 * @Author: konodoki 1326898804@qq.com
 * @Date: 2026-02-02 15:09:20
 * @LastEditors: konodoki 1326898804@qq.com
 * @LastEditTime: 2026-02-09 13:16:43
 * @FilePath: /bxi_hand/main/FakeSerial.h
 * @Description: 
 * @
 * @Copyright (c) 2026 by konodoki , All Rights Reserved. 
 */
#ifndef USB_SERIAL_H
#define USB_SERIAL_H
#include "freertos/idf_additions.h"
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include "CRC.h"

typedef struct{
    uint8_t header[4];
    uint16_t payload[28];
    uint8_t CRC;
}__attribute__((packed)) JointStateSerialPack;

typedef union{
    JointStateSerialPack pack;
    uint8_t bytes[sizeof(JointStateSerialPack)];
}JointStateSerialPack_u;


typedef struct{
    uint8_t header[4];
    uint8_t oper;
    uint8_t reserve;
    uint8_t payload[14];
    uint8_t CRC;
}__attribute__((packed)) RxSerialPack;

typedef union{
    RxSerialPack pack;
    uint8_t bytes[sizeof(RxSerialPack)];
}RxSerialPack_u;

typedef struct{
    uint8_t header[4];
    uint8_t payload[16];
    uint8_t CRC;
}__attribute__((packed)) TxSerialPack;

typedef union{
    TxSerialPack pack;
    uint8_t bytes[sizeof(TxSerialPack)];
}TxSerialPack_u;

extern CRC::Table<crcpp_uint8, 8> CRC8_Table;

void FakeSerialpushDataToBuf(uint8_t *data,uint32_t len);

class FakeSerial {
public:
    FakeSerial();
    void begin(unsigned long baud);
    void write(uint8_t *data,uint32_t len);
private:
    bool _initialized;
};

extern FakeSerial Serial;
extern QueueHandle_t pack_queue;

#endif