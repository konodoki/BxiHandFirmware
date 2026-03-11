/*** 
 * @Author: konodoki 1326898804@qq.com
 * @Date: 2026-02-02 12:32:41
 * @LastEditors: konodoki 1326898804@qq.com
 * @LastEditTime: 2026-02-09 13:16:48
 * @FilePath: /bxi_hand/main/hand_driver/Perferences.h
 * @Description: 
 * @
 * @Copyright (c) 2026 by konodoki , All Rights Reserved. 
 */
extern "C"{
    #include "nvs_flash.h"
}
class Preferences{
private:
    nvs_handle_t handle;
    bool closed=true;
public:
    Preferences();
    void begin(const char *ns,bool rw);//true readonly
    void putInt(const char *key,int valuse);
    int getInt(const char *key,int valuse);
    void end();
};