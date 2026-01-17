#ifndef ROBOT_GLOBAL_H
#define ROBOT_GLOBAL_H

#include "struct_typedef.h"
#include "stdint.h"
#include "../Components/remote/remote.h"
#include "../../Application/auto_aim.h"  //【新增】引入自瞄头文件，支持target_info_t结构体

/* --- 模式枚举定义 --- */

typedef enum {
    GIMBAL_RELAX = 0,    // 失能状态，电机不出力
    GIMBAL_REMOTE,       // 遥控器手动模式（基于IMU控制）
    GIMBAL_AUTO,         // 视觉自瞄模式
} gimbal_mode_e;

typedef enum {
    CHASSIS_RELAX = 0,   // 失能状态
    CHASSIS_FOLLOW,      // 跟随模式（以云台朝向为正前方）
} chassis_mode_e;

typedef enum {
    SHOOT_STOP = 0,      // 停止发射
    SHOOT_READY,         // 摩擦轮起旋
} shoot_mode_e;

/* --- 核心控制结构体 --- */

typedef struct {
    // 1. 系统当前运行模式
    gimbal_mode_e  gimbal_mode;
    chassis_mode_e chassis_mode;
    shoot_mode_e   shoot_mode;

    // 2. 云台姿态反馈数据 (由 Sensor Task 更新)
    struct {
        fp32 yaw;        // 当前航向角 (度)
        fp32 pitch;      // 当前俯仰角 (度)
        fp32 roll;       // 当前横滚角 (度)
        fp32 yaw_v;      // 航向角速度 (度/s)
        fp32 pitch_v;    // 俯仰角速度 (度/s)
    } gimbal;

    // 3. 底盘运动状态 (由 Chassis Task 更新)
    struct {
        fp32 yaw_speed;      // 云台相对于底盘的机械夹角 (由编码器转化)
    } chassis;

    // 4. 系统监控与异常处理
    struct {
        uint8_t  sensor_ready;   // 传感器校准完成标志
        uint8_t  remote_online;  // 遥控器在线标志
        uint8_t  vision_online;  // 视觉系统在线标志
    } monitor;

    // 5. 输入引用指针
    const RC_ctrl_t *rc;         // 遥控器原始数据引用


    target_info_t target_info;   // 上位机下发的自瞄数据(valid,shoot,yaw,pitch)

} robot_ctrl_info_t;

/* --- 全局变量声明 --- */
extern robot_ctrl_info_t robot_ctrl;

/* --- 核心工具函数 --- */
void Robot_Global_Init(void);

#endif