#ifndef AUTOAIM_H
#define AUTOAIM_H

#include "../Bsp/uart/bsp_uart.h"
#include "../Bsp/usb_cdc/bsp_usb_cdc.h"
#include <stdint.h>

// 目标信息结构体【核心修改：新增 valid、shoot 两个成员，匹配上位机4字段】
typedef struct {
    uint8_t valid;            // 上位机目标有效标志 1=有效 0=无效
    uint8_t shoot;            // 上位机发射允许标志 1=可发射 0=不可发射
    float aim_target_yaw;     // 云台Yaw目标角度
    float aim_target_pitch;   // 云台Pitch目标角度

} target_info_t;

// 函数声明（无修改，完全保留）
int auto_aim_init(struct usb_device *uart_dev);
int parse_target_data(target_info_t *target);
int is_target_valid(target_info_t *target);
void auto_aim_control(target_info_t *target, float *yaw_output, float *pitch_output);

#endif //AUTOAIM_H