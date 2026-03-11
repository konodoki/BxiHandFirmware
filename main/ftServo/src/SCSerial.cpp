/*** 
 * @Author: konodoki 1326898804@qq.com
 * @Date: 2026-02-02 11:24:18
 * @LastEditors: konodoki 1326898804@qq.com
 * @LastEditTime: 2026-02-09 13:17:00
 * @FilePath: /bxi_hand/main/ftServo/src/SCSerial.cpp
 * @Description: 
 * @
 * @Copyright (c) 2026 by konodoki , All Rights Reserved. 
 */
/*
 * SCSerial.h
 * 飞特串行舵机硬件接口层程序
 * 日期: 2024.4.2
 * 作者:
 */

#include "SCSerial.h"
#include "esp_log.h"
#include "hal/uart_types.h"
// 注意，移植的时候我没有考虑这个类多次被实例化
extern "C" {
#include "driver/uart.h"
QueueHandle_t uart_queue;
void init_uart()
{
    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, uart_buffer_size,
                                        uart_buffer_size, 10, &uart_queue, 0));
    uart_config_t uart_config = {
        .baud_rate = 1000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 3, 2, -1, -1));
    ESP_LOGI("SCSerial", "Uart init!");
}
}
SCSerial::SCSerial()
{
    IOTimeOut = 10;
    init_uart();
}

SCSerial::SCSerial(u8 End)
    : SCS(End)
{
    IOTimeOut = 10;
    init_uart();
}

SCSerial::SCSerial(u8 End, u8 Level)
    : SCS(End, Level)
{
    IOTimeOut = 10;
    init_uart();
}

int SCSerial::readSCS(unsigned char *nDat, int nLen, unsigned long TimeOut)
{
    return uart_read_bytes(UART_NUM_1, nDat, nLen, 100);
}

int SCSerial::readSCS(unsigned char *nDat, int nLen)
{
    return uart_read_bytes(UART_NUM_1, nDat, nLen, 100);
}

int SCSerial::writeSCS(unsigned char *nDat, int nLen)
{
    uart_write_bytes(UART_NUM_1, nDat, nLen);
    return 0;
}

int SCSerial::writeSCS(unsigned char bDat)
{
    uart_write_bytes(UART_NUM_1, &bDat, 1);
    return 0;
}

void SCSerial::rFlushSCS()
{
    uart_flush_input(UART_NUM_1);
}

void SCSerial::wFlushSCS()
{
    uart_wait_tx_done(UART_NUM_1, 10);
}