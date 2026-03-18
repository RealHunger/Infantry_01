#ifndef COMMUNICATION_TASK_H
#define COMMUNICATION_TASK_H

#include <stdint.h>
#include "../Bsp/usb_cdc/bsp_usb_cdc.h"

// 目标信息结构体
typedef struct {
    uint8_t valid;            // 上位机目标有效标志 1=有效 0=无效
    uint8_t shoot;            // 上位机发射允许标志 1=可发射 0=不可发射
    float aim_target_yaw;     // 云台Yaw目标角度
    float aim_target_pitch;   // 云台Pitch目标角度
} target_info_t;

// 小电脑底盘移动控制结构体
typedef struct {
    float auto_front_speed;   // 小电脑下发前后速度
    float auto_right_speed;   // 小电脑下发左右速度
    float auto_yaw_speed;     // 小电脑下发云台yaw轴速度
} auto_info_t;

// 任务函数声明
void communication_task_func(void const * argument);

// 角度限位控制（保留原有接口供 Gimbal Task 调用）
void auto_aim_control(target_info_t *target, float *yaw_output, float *pitch_output);

#endif // COMMUNICATION_TASK_H