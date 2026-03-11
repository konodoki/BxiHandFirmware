/*** 
 * @Author: konodoki 1326898804@qq.com
 * @Date: 2026-02-02 12:32:33
 * @LastEditors: konodoki 1326898804@qq.com
 * @LastEditTime: 2026-02-09 13:16:51
 * @FilePath: /bxi_hand/main/hand_driver/Perferences.cpp
 * @Description: 
 * @
 * @Copyright (c) 2026 by konodoki , All Rights Reserved. 
 */
#include"Perferences.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include <cstdint>
#include <cstring>
Preferences::Preferences(){

}
void Preferences::begin(const char *ns,bool rw){
    if(!closed){
        nvs_close(handle);
        ESP_LOGW("NVS","NVS Open again when last was not end");
    }
    esp_err_t err = nvs_open(ns, rw?NVS_READONLY:NVS_READWRITE, &handle);
    if(err != ESP_OK){
        ESP_LOGE("NVS","NVS Open ERR");
    }
    ESP_LOGI("NVS","NVS begin");
    closed = false;
}
void Preferences::putInt(const char *key,int valuse){
    esp_err_t err = nvs_set_i32(handle,key,valuse);
    if(err != ESP_OK){
        ESP_LOGE("NVS","NVS putInt ERR");
    }
}
int Preferences::getInt(const char *key,int valuse){
    int32_t out;
    esp_err_t err = nvs_get_i32(handle,key,&out);
    if(err != ESP_OK){
        ESP_LOGE("NVS","NVS getInt ERR");
    }
    return out;
}
void Preferences::end(){
    esp_err_t err = nvs_commit(handle);
    if(err != ESP_OK){
        ESP_LOGE("NVS","NVS nvs_commit ERR");
    }
    if(!closed){
        nvs_close(handle);
        ESP_LOGI("NVS","NVS end");
        closed=true;
    }
}