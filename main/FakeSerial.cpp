/***
 * @Author: konodoki 1326898804@qq.com
 * @Date: 2026-02-02 15:09:25
 * @LastEditors: konodoki 1326898804@qq.com
 * @LastEditTime: 2026-03-11 19:44:07
 * @FilePath: /bxi_hand/main/FakeSerial.cpp
 * @Description:
 * @
 * @Copyright (c) 2026 by konodoki , All Rights Reserved.
 */
#include "FakeSerial.h"
#include "ble_hand_app.h"
#include "driver/usb_serial_jtag_select.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdio.h>
#include <vector>
extern "C" {
#include "driver/usb_serial_jtag.h"
#include "freertos/ringbuf.h"
}

CRC::Table<crcpp_uint8, 8> CRC8_Table(CRC::CRC_8());

QueueHandle_t pack_queue;
#define RINGBUF_SIZE 1024
static std::vector<uint8_t> rx_buffer;
const size_t PACK_SIZE = sizeof(RxSerialPack);

void FakeSerialpushDataToBuf(uint8_t *data, uint32_t len)
{
    if (len <= 0)
        return;
    // 如果发太快，缓冲区满了就放弃旧包
    if (rx_buffer.size() < RINGBUF_SIZE) {
        rx_buffer.insert(rx_buffer.end(), data, data + len);
        ESP_LOGI("ble", "receive %lu", len);
    }
}

void readUsbTask(void *)
{
    uint8_t temp_buf[256];

    while (1) {
        if (usb_serial_jtag_read_ready()) {
            ssize_t available = usb_serial_jtag_get_read_bytes_available();
            if (available > 0) {
                ssize_t read_cnt = std::min(available, (ssize_t)256);
                ssize_t real_read =
                    usb_serial_jtag_read_bytes(temp_buf, read_cnt, 0);
                if (real_read > 0) {
                    rx_buffer.insert(rx_buffer.end(), temp_buf,
                                     temp_buf + real_read);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
void process_pack(void *)
{
    while (true) {
        while (rx_buffer.size() >= PACK_SIZE) {
            // 检查帧头 (0xA1, 0xA2, 0xA3, 0xA4)
            if (rx_buffer[0] == 0xA1 && rx_buffer[1] == 0xA2 &&
                rx_buffer[2] == 0xA3 && rx_buffer[3] == 0xA4) {
                RxSerialPack_u u;
                // 将 vector 前 PACK_SIZE 字节拷贝到联合体
                std::copy(rx_buffer.begin(), rx_buffer.begin() + PACK_SIZE,
                          u.bytes);

                // CRC 校验
                uint8_t crc =
                    CRC::Calculate(u.bytes, PACK_SIZE - 1, CRC8_Table);
                if (u.pack.CRC == crc) {
                    // 校验通过，发送队列
                    xQueueSendToBack(pack_queue, &u.pack, 0);
                    // 从缓冲区移除这完整的一帧
                    rx_buffer.erase(rx_buffer.begin(),
                                    rx_buffer.begin() + PACK_SIZE);
                    continue;
                } else {
                    // 校验失败，说明可能是伪造帧头，移除首字节继续寻找下一个帧头
                    rx_buffer.erase(rx_buffer.begin());
                }
            } else {
                // 非帧头字节，直接弹出
                rx_buffer.erase(rx_buffer.begin());
            }
        }
        vTaskDelay(1);
    }
}
FakeSerial::FakeSerial()
    : _initialized(false)
{
}

void FakeSerial::begin(unsigned long baud)
{
    if (_initialized)
        return;

    // 1. 安装 USB-Serial-JTAG 驱动

    usb_serial_jtag_driver_config_t cfg =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();

    // 调整缓冲区大小以防高速传输丢失数据 (可选)

    cfg.rx_buffer_size = 1024;

    cfg.tx_buffer_size = 1024;

    usb_serial_jtag_driver_install(&cfg);

    pack_queue =
        xQueueGenericCreate(32, sizeof(RxSerialPack), queueQUEUE_TYPE_BASE);

    xTaskCreatePinnedToCore(readUsbTask, "readUsbTask", 9600, NULL, 10, NULL,
                            1);

    xTaskCreate(process_pack, "process_pack", 10240, NULL, 10, NULL);
    _initialized = true;
}

void FakeSerial::write(uint8_t *data, uint32_t len)
{
    if (len != 16) {
        ESP_LOGE("FakeSerial", "Only Can write 16 bytes data");

        return;
    }

    TxSerialPack_u u;

    u.pack.header[0] = 0xA1;

    u.pack.header[1] = 0xA2;

    u.pack.header[2] = 0xA3;

    u.pack.header[3] = 0xA4;

    memcpy(u.pack.payload, data, len);

    u.pack.CRC = CRC::Calculate(u.bytes, sizeof(TxSerialPack) - 1, CRC8_Table);

    if (!sendHandState(u)) {
        usb_serial_jtag_write_bytes(u.bytes, sizeof(TxSerialPack), 0);
    }
}

// 实例化全局对象

FakeSerial Serial;