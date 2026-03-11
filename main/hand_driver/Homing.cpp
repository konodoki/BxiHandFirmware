/***
 * @Author: konodoki 1326898804@qq.com
 * @Date: 2026-02-02 12:20:53
 * @LastEditors: konodoki 1326898804@qq.com
 * @LastEditTime: 2026-02-26 13:14:26
 * @FilePath: /bxi_hand/main/hand_driver/Homing.cpp
 * @Description:
 * @
 * @Copyright (c) 2026 by konodoki , All Rights Reserved.
 */
#include "Homing.h"
#include "HandConfig.h"

// 将 FreeRTOS 的 tick 计数映射到 millis() 接口
#define millis() xTaskGetTickCount()
// 将 FreeRTOS 的延迟函数映射到 delay() 接口
#define delay(_) vTaskDelay(_)

// ========================= 舵机基准参数（只读） =========================
// 根据硬件版本宏定义，分别提供左手和右手的基准舵机数据。
// ServoData 结构体字段含义：{ extend_count, grasp_count, servo_direction }
//   extend_count   : 伸展姿态对应的编码器计数值
//   grasp_count    : 抓握姿态对应的编码器计数值
//   servo_direction: 正转方向（+1 或 -1），取决于安装方向

const ServoData sd_base_left[7] = {
    { 3186, 2048, 1 }, // 0: 拇指外展
    { 2048, 865, -1 }, // 1: 拇指屈曲
    { 0, 2980, 1 }, // 2: 拇指肌腱
    { 4095, 817, -1 }, // 3: 食指
    { 4095, 817, -1 }, // 4: 中指
    { 4095, 817, -1 }, // 5: 无名指
    { 4095, 817, -1 }, // 6: 小指
};
const ServoData sd_base_right[7] = {
    { 910, 2048, -1 }, // 0: 拇指外展
    { 2048, 3231, 1 }, // 1: 拇指屈曲
    { 4095, 1115, -1 }, // 2: 拇指肌腱
    { 0, 3278, 1 }, // 3: 食指
    { 0, 3278, 1 }, // 4: 中指
    { 0, 3278, 1 }, // 5: 无名指
    { 0, 3278, 1 }, // 6: 小指
};

// ========================= 工具函数 =========================

/**
 * @brief 将运行时舵机参数数组 sd[] 重置为出厂基准值。
 *        根据编译时宏 RIGHT_HAND / LEFT_HAND 选择对应的基准表。
 */
void resetSdToBaseline()
{
#if defined(RIGHT_HAND)
    const ServoData *src = sd_base_right;
#elif defined(LEFT_HAND)
    const ServoData *src = sd_base_left;
#else
// 未定义手型宏时默认使用右手基准，并在编译期给出警告
#warning "No hand macro defined; defaulting to RIGHT_HAND baseline"
    const ServoData *src = sd_base_right;
#endif
    // 将基准数据逐一复制到全局舵机参数数组
    for (int i = 0; i < 7; ++i)
        sd[i] = src[i];
}

// ========================= 归零忙标志 =========================

// 归零流程是否正在进行（true = 忙碌，外部禁止下发运动指令）
static volatile bool s_busy = false;

/** @brief 查询归零流程是否仍在运行。 */
bool HOMING_isBusy()
{
    return s_busy;
}

// ========================= 缓冲位置常量 =========================

// 归零时为防止舵机卡死，在软件限位边界内留出 10° 的缓冲余量
static const float BUF_DEG = 10.0f;
// 将缓冲角度转换为 12 位编码器计数（编码器量程 0~4095 对应 0~360°）
static const uint16_t BUF_CNT = (uint16_t)((BUF_DEG / 360.0f) * 4095.0f);

// ========================= 辅助函数 =========================

/**
 * @brief 设置指定舵机的角度软限位（写入 EEPROM）。
 * @param servoID  目标舵机 ID
 * @param minLim   最小角度限位（12 位编码器计数，0~4095）
 * @param maxLim   最大角度限位（12 位编码器计数，0~4095）
 *
 * 流程：解锁 EEPROM → 写最小/最大限位 → 锁定 EEPROM → 重新使能扭矩
 * 注意：EEPROM 写操作会自动关闭扭矩，因此写完后必须重新使能。
 */
static void set_servo_limits(uint8_t servoID, uint16_t minLim, uint16_t maxLim)
{
    // 防止越界：将输入值钳位到 12 位范围
    if (minLim > 4095)
        minLim = 4095;
    if (maxLim > 4095)
        maxLim = 4095;

    // 解锁 EEPROM（同时会禁用扭矩）
    hlscl.unLockEprom(servoID);
    hlscl.writeWord(servoID, HLSCL_MIN_ANGLE_LIMIT_L,
                    minLim); // 寄存器地址 9-10：最小角度限位
    hlscl.writeWord(servoID, HLSCL_MAX_ANGLE_LIMIT_L,
                    maxLim); // 寄存器地址 11-12：最大角度限位
    hlscl.LockEprom(servoID);

    // 重新使能扭矩，确保后续运动指令有效
    hlscl.EnableTorque(servoID, 1);
}

/**
 * @brief 对单个舵机执行电流触碰归零，然后将其移动到初始姿态。
 *
 * 算法：
 *   1. 全范围开放限位（min=0, max=0 表示无限位）。
 *   2. 以恒定速度向归零方向运动，实时检测电流。
 *   3. 电流超过阈值（即触碰机械限位）时停止并校准零点偏移。
 *   4. 根据舵机编号执行不同的后处理动作（各关节逻辑不同）。
 *
 * @param index          舵机在 sd[] 及 SERVO_IDS[] 中的逻辑索引（0~6）
 * @param direction      归零运动方向（+1 或 -1）
 * @param current_limit  触碰判断阈值（mA）
 */
static void zero_with_current(uint8_t index, int direction, int current_limit)
{
    uint8_t servoID = SERVO_IDS[index];
    int current = 0;
    int position = 0;

    // 归零过程中完全开放行程窗口，防止限位提前截断运动
    set_servo_limits(servoID, 0, 0);
    hlscl.ServoMode(servoID); // 切换到位置伺服模式
    hlscl.FeedBack(servoID); // 使能反馈

    uint32_t t0 = millis();
    // 持续向限位方向推进，直到检测到足够大的堵转电流或超时（10 s）
    while (abs(current) < current_limit) {
        hlscl.WritePosEx(servoID, 50000 * direction, 2400, 0, current_limit);
        current = hlscl.ReadCurrent(servoID);
        position = hlscl.ReadPos(servoID);
        if (millis() - t0 > 10000)
            break; // 超时保护，防止永久堵转
        vTaskDelay(pdMS_TO_TICKS(1)); // 让出 CPU，避免看门狗复位
    }

    // ---- 在接触点执行一次零点校准 ----
    hlscl.WritePosEx(servoID, position, 2400, 0, 1000); // 保持当前位置
    delay(30);
    hlscl.CalibrationOfs(servoID); // 将当前位置写入零偏寄存器
    delay(30);
    position = hlscl.ReadPos(servoID); // 重新读取校准后的绝对位置

    // ---- 各关节的后处理动作 ----
    if (servoID == 0) {
        // 拇指外展轴：归零后移动到抓握姿态，为后续步骤做好准备
        hlscl.WritePosEx(servoID, sd[index].grasp_count, 2400, 0, 1000);
        delay(250);
    } else if (servoID == 1) {
        // 拇指屈曲轴：归零后移动到伸展姿态
        hlscl.WritePosEx(servoID, sd[index].extend_count, 2400, 0, 1000);
        delay(250);
    } else if (servoID == 2) {
// 拇指肌腱轴：需要二次校准以消除绳传动的初始松弛
#if defined(BXI_HAND_V1)
        //   步骤1：先向归零方向再推进 2048 计数（约 180°）
        hlscl.WritePosEx(servoID, position + (direction * 2048), 2400, 0, 1000);
        delay(250);
        //   步骤2：在新位置再次写入零偏
        hlscl.CalibrationOfs(servoID);
        delay(30);
        //   步骤3：移动到伸展姿态（退回 625 计数的松弛余量）
        hlscl.WritePosEx(servoID, sd[index].extend_count - (direction * 625),
                         2400, 0, 1000);
        delay(30);
#elif defined(BXI_HAND_V2)
        //   步骤1：先向归零方向再推进 2048 计数（约 180°）
        hlscl.WritePosEx(servoID, position + (direction * 1350), 2400, 0, 1000);
        delay(250);
        //   步骤2：在新位置再次写入零偏
        hlscl.CalibrationOfs(servoID);
        delay(30);
        //   步骤3：移动到伸展姿态（退回 625 计数的松弛余量）
        hlscl.WritePosEx(servoID, sd[index].extend_count - (direction * 625),
                         2400, 0, 1000);
        delay(30);
#endif
    } else {
        // 不应走到此分支（servoID 0/1/2 已处理完毕，3~6 由并行函数处理）
        // 若进入此处，说明传入了错误的 servoID，需排查调用方
    }
}

/**
 * @brief 对四根手指舵机（逻辑通道 3~6）并行执行电流触碰归零。
 *
 * 相较于逐一归零，并行方式可将四根手指的归零时间缩短约 75%。
 * 每个舵机独立检测堵转电流，触碰后立即锁定该舵机并记录接触位置。
 * 所有舵机均完成（或全局超时）后，依次执行二次校准并移动至伸展姿态。
 *
 * @param firstIdx      起始逻辑索引（通常为 3）
 * @param count         要归零的舵机数量（通常为 4）
 * @param current_limit 触碰判断阈值（mA）
 */
static void zero_fingers_parallel_with_current(uint8_t firstIdx, uint8_t count,
                                               int current_limit)
{
    // ---------- 入参校验 ----------
    if (count == 0)
        return;
    if (firstIdx > 6)
        return;
    if (firstIdx + count > 7)
        count = 7 - firstIdx; // 防止越界

    const uint32_t TIMEOUT_MS = 10000; // 最长等待 10 s

    // ---------- 状态变量初始化 ----------
    static bool done[7]; // 各舵机是否已触碰到限位
    static int contactPos[7]; // 各舵机触碰时的编码器位置
    static int cur[7]; // 各舵机最近一次读取的电流值
    static int pos[7]; // 各舵机最近一次读取的位置值

    for (uint8_t i = 0; i < 7; ++i) {
        done[i] = false;
        contactPos[i] = 0;
        cur[i] = 0;
        pos[i] = 0;
    }

    // ---------- 初始化所有手指舵机 ----------
    for (uint8_t i = 0; i < count; ++i) {
        uint8_t idx = firstIdx + i;
        uint8_t servoID = SERVO_IDS[idx];
        set_servo_limits(servoID, 0, 0); // 开放限位
        hlscl.ServoMode(servoID); // 伺服模式
        hlscl.FeedBack(servoID); // 使能反馈
    }

    uint32_t t0 = millis();

    // ---------- 并行归零主循环 ----------
    while (true) {
        // 检查是否所有手指均已完成触碰
        bool allDone = true;
        for (uint8_t i = 0; i < count; ++i) {
            uint8_t idx = firstIdx + i;
            if (!done[idx]) {
                allDone = false;
                break;
            }
        }
        if (allDone)
            break;

        // 全局超时保护
        if (millis() - t0 > TIMEOUT_MS)
            break;

        // 对尚未完成的舵机持续发送归零运动指令
        for (uint8_t i = 0; i < count; ++i) {
            uint8_t idx = firstIdx + i;
            if (done[idx])
                continue;
            uint8_t servoID = SERVO_IDS[idx];
            int dir = sd[idx].servo_direction;
            hlscl.WritePosEx(servoID, 50000 * dir, 2400, 0, current_limit);
        }

        // 读取电流和位置，检测是否触碰限位
        for (uint8_t i = 0; i < count; ++i) {
            uint8_t idx = firstIdx + i;
            if (done[idx])
                continue;
            uint8_t servoID = SERVO_IDS[idx];
            cur[idx] = hlscl.ReadCurrent(servoID);
            pos[idx] = hlscl.ReadPos(servoID);
            if (abs(cur[idx]) >= current_limit) {
                // 触碰到机械限位：记录位置并原地锁住（低速、低加速、保持扭矩）
                done[idx] = true;
                contactPos[idx] = pos[idx];
                hlscl.WritePosEx(servoID, contactPos[idx], 60, 50, 1000);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // 1 ms 轮询间隔，让出 CPU
    }

    // ---------- 后处理：逐一完成二次校准并移动至伸展姿态 ----------
    for (uint8_t i = 0; i < count; ++i) {
        uint8_t idx = firstIdx + i;
        uint8_t servoID = SERVO_IDS[idx];
        int dir = sd[idx].servo_direction;

        // 若超时前未触碰到限位，使用最后读取的位置作为回退参考
        int p = done[idx] ? contactPos[idx] : pos[idx];

        // 步骤1：在接触点执行一次零点校准
        hlscl.WritePosEx(servoID, p, 2400, 0, 1000);
        delay(30);
        hlscl.CalibrationOfs(servoID);
        delay(30);
        p = hlscl.ReadPos(servoID);

// 步骤2：向归零反方向推进 2048 计数（消除绳传动松弛），再次校准
#if defined(BXI_HAND_V1)
        hlscl.WritePosEx(servoID, p + (dir * 2048), 2400, 0, 1000);
#elif defined(BXI_HAND_V2)
        hlscl.WritePosEx(servoID, p + (dir * 900), 2400, 0, 1000);
#endif
        delay(300);
        hlscl.CalibrationOfs(servoID);
        delay(30);

        // 步骤3：移动到预设伸展姿态
        hlscl.WritePosEx(servoID, sd[idx].extend_count, 2400, 0, 1000);
    }
}

// ========================= 顶层归零函数 =========================

/**
 * @brief 对全部 7 个舵机执行完整的归零流程。
 *
 * 顺序：
 *   1. 重置运行时参数到基准值
 *   2. 获取总线互斥锁（防止与其他任务的通信冲突）
 *   3. 依次归零拇指三个轴（串行，因各轴有特殊后处理逻辑）
 *   4. 并行归零四根手指
 *   5. 将全部关节移动到最终伸展姿态
 *   6. 释放总线互斥锁
 */
void zero_all_motors()
{
    resetSdToBaseline(); // 恢复出厂基准参数

    // 占用总线互斥量，排他性访问 HLSCL 串行总线
    if (gBusMux)
        xSemaphoreTake(gBusMux, portMAX_DELAY);

    // ---- 拇指三轴：串行归零 ----
    zero_with_current(0, sd[0].servo_direction, 960); // 拇指外展
    zero_with_current(1, sd[1].servo_direction, 960); // 拇指屈曲
    zero_with_current(2, sd[2].servo_direction, 960); // 拇指肌腱

    // ---- 四指：并行归零（索引 3~6，共 4 个舵机）----
    zero_fingers_parallel_with_current(3, 4, 960);

    // ---- 归零完成后，驱动全部关节到伸展姿态 ----
    hlscl.WritePosEx(SERVO_IDS[0], sd[0].extend_count, 2400, 0,
                     1023); // 拇指外展
                            // → 伸展
    hlscl.WritePosEx(SERVO_IDS[1], sd[1].extend_count, 2400, 0,
                     1023); // 拇指屈曲
                            // → 伸展
    hlscl.WritePosEx(SERVO_IDS[2], sd[2].extend_count, 2400, 0,
                     1023); // 拇指肌腱
                            // → 伸展
    hlscl.WritePosEx(SERVO_IDS[3], sd[3].extend_count, 2400, 0, 1023); // 食指 →
                                                                       // 伸展
    hlscl.WritePosEx(SERVO_IDS[4], sd[4].extend_count, 2400, 0, 1023); // 中指 →
                                                                       // 伸展
    hlscl.WritePosEx(SERVO_IDS[5], sd[5].extend_count, 2400, 0, 1023); // 无名指
                                                                       // → 伸展
    hlscl.WritePosEx(SERVO_IDS[6], sd[6].extend_count, 2400, 0, 1023); // 小指 →
                                                                       // 伸展

    // 释放总线互斥量，允许其他任务访问总线
    if (gBusMux)
        xSemaphoreGive(gBusMux);
}

/**
 * @brief 归零任务入口，供外部调用（如 FreeRTOS 任务或按键触发）。
 *        执行期间将 s_busy 标志置为 true，完成后清除。
 */
void HOMING_start()
{
    s_busy = true;
    zero_all_motors();
    s_busy = false;
}