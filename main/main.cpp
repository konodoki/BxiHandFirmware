/*** 
 * @                       _oo0oo_
 * @                      o8888888o
 * @                      88" . "88
 * @                      (| -_- |)
 * @                      0\  =  /0
 * @                    ___/`---'\___
 * @                  .' \\|     |// '.
 * @                 / \\|||  :  |||// \
 * @                / _||||| -:- |||||- \
 * @               |   | \\\  - /// |   |
 * @               | \_|  ''\---/''  |_/ |
 * @               \  .-\__  '-'  ___/-. /
 * @             ___'. .'  /--.--\  `. .'___
 * @          ."" '<  `.___\_<|>_/___.' >' "".
 * @         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 * @         \  \ `_.   \_ __\ /__ _/   .-` /  /
 * @     =====`-.____`.___ \_____/___.-`___.-'=====
 * @                       `=---='
 * @
 * @
 * @     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * @
 * @           佛祖保佑     永不宕机     永无BUG
 */

/*** 
 * @Author: konodoki 1326898804@qq.com
 * @Date: 2026-02-01 18:07:02
 * @LastEditors: konodoki 1326898804@qq.com
 * @LastEditTime: 2026-02-09 13:16:23
 * @FilePath: /bxi_hand/main/main.cpp
 * @Description: 
 * @
 * @Copyright (c) 2026 by konodoki , All Rights Reserved. 
 */
#include "HLSCL.h"
#include <cstddef>
#include <cstdio>
#include "firmware.h"
extern "C" {
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "bxi_ble.h"
void app_main(void)
{
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ", CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ?
               ", 802.15.4 (Zigbee/Thread)" :
               "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }
    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" :
                                                           "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n",
           esp_get_minimum_free_heap_size());
    bxi_ble_init();
    // 注意手的初始化必须在蓝牙后面
    HandSetUp();
}
}
