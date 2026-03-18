#include "auto_aim.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "../../Bsp/LED/bsp_led.h"

// 全局变量 (保留，无修改)
static struct uart_device *auto_aim_uart = NULL;
static struct usb_device *auto_aim_usb = NULL;

// 初始化自瞄系统 (无修改，完全保留)
int auto_aim_init(struct usb_device *usb_dev) {
    if (usb_dev == NULL) {
        return -1;
    }
    auto_aim_usb = usb_dev;

    return 0;
}

// 解析目标数据【核心重写：适配 valid,shoot,yaw,pitch\r\n 格式】
int parse_target_data(target_info_t *target) {
    char buffer[100];
    // 接收上位机USB数据，超时200us，长度不变
    int received_len = auto_aim_usb->Recv(auto_aim_usb, buffer, sizeof(buffer) - 1, 200);

    if (received_len > 0) {
        buffer[received_len] = '\0';  // 安全添加字符串结束符，修复原代码截断问题

        char *token;
        char *rest = buffer;
        const char *delim = ",\r\n";   // 分隔符：兼容逗号+回车+换行，自动过滤末尾\r\n

        // 第1段：解析 valid 目标有效标志 (整型 1=有效 0=无效)
        token = strtok_r(rest, delim, &rest);
        if (!token) return 0;
        target->valid = atoi(token);

        // 第2段：解析 shoot 发射允许标志 (整型 1=可发射 0=不可发射)
        token = strtok_r(NULL, delim, &rest);
        if (!token) return 0;
        target->shoot = atoi(token);

        // 第3段：解析 yaw 云台航向角目标值 (浮点型)
        token = strtok_r(NULL, delim, &rest);
        if (!token) return 0;
        target->aim_target_yaw = strtof(token, NULL);

        // 第4段：解析 pitch 云台俯仰角目标值 (浮点型)
        token = strtok_r(NULL, delim, &rest);
        if (!token) return 0;
        target->aim_target_pitch = strtof(token, NULL);


        // 调用校验函数，返回最终有效性 1=有效 0=无效
        return is_target_valid(target);
    }
    // 无接收数据，返回无效
    return 0;
}

// 判断目标是否有效【小幅优化：优先以上位机valid标志为准，NaN校验兜底】
int is_target_valid(target_info_t *target) {
    // 先判断上位机的有效标志 + 角度值不是非法值，双重保险
    if (target->valid != 1 || isnan(target->aim_target_pitch) || isnan(target->aim_target_yaw)) {
        return 0;
    }
    return 1;
}

// 自瞄角度限位控制 (保留你的原版逻辑，仅优化写法更简洁，功能不变)
void auto_aim_control(target_info_t *target, float *yaw_output, float *pitch_output) {
    *yaw_output = target->aim_target_yaw;
    // 你的原版俯仰角限位逻辑，完全保留
    *pitch_output = target->aim_target_pitch > 0.45f ? 0.45f : target->aim_target_pitch;
    *pitch_output = *pitch_output < -0.45f ? -0.45f : *pitch_output;
}