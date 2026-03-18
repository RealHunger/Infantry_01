#include "communication_task.h"
#include "../Application/robot_global.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "cmsis_os.h"

// 【保留你原本的函数】用于判断数据有效性
static int is_target_valid(target_info_t *target) {
    if (target->valid != 1 || isnan(target->aim_target_pitch) || isnan(target->aim_target_yaw)) {
        return 0;
    }
    return 1;
}

// 【保留你原本的解包逻辑】遇到错误直接 return 0，清爽无嵌套！
static int parse_target_data_internal(char *buffer, target_info_t *target, auto_info_t *move) {
    char *token;
    char *rest = buffer;
    const char *delim = ",\r\n";

    token = strtok_r(rest, delim, &rest);
    if (!token) return 0;
    target->valid = atoi(token);

    token = strtok_r(NULL, delim, &rest);
    if (!token) return 0;
    target->shoot = atoi(token);

    token = strtok_r(NULL, delim, &rest);
    if (!token) return 0;
    target->aim_target_yaw = strtof(token, NULL);

    token = strtok_r(NULL, delim, &rest);
    if (!token) return 0;
    target->aim_target_pitch = strtof(token, NULL);

    token = strtok_r(NULL, delim, &rest);
    if (!token) return 0;
    move->auto_front_speed = strtof(token, NULL);

    token = strtok_r(NULL, delim, &rest);
    if (!token) return 0;
    move->auto_right_speed = strtof(token, NULL);

    token = strtok_r(NULL, delim, &rest);
    if (!token) return 0;
    move->auto_yaw_speed = strtof(token, NULL);

    return is_target_valid(target);
}

// 通信任务主体
void communication_task_func(void const * argument) {
    struct usb_device *Usb = usb_get_device();
    char buffer[128];
    uint32_t last_recv_tick = osKernelSysTick();

    while (1) {
        if (Usb != NULL) {
            // 设置 10ms 超时，专心等数据，不影响别人
            int received_len = Usb->Recv(Usb, buffer, sizeof(buffer) - 1, 10);

            if (received_len > 0) {
                buffer[received_len] = '\0'; // 安全截断

                // 使用临时变量防撕裂
                target_info_t temp_target = {0};
                auto_info_t temp_move = {0};

                // 直接调用你原本清爽的解析逻辑！
                if (parse_target_data_internal(buffer, &temp_target, &temp_move) == 1) {
                    robot_ctrl.target_info = temp_target;
                    robot_ctrl.auto_info = temp_move;
                    robot_ctrl.monitor.vision_online = 1;
                    last_recv_tick = osKernelSysTick();
                }
            }
        }

        // 掉线监控
        if (osKernelSysTick() - last_recv_tick > 200) {
            robot_ctrl.monitor.vision_online = 0;
            robot_ctrl.target_info.valid = 0;
            robot_ctrl.auto_info.auto_front_speed = 0.0f;
            robot_ctrl.auto_info.auto_right_speed = 0.0f;
            robot_ctrl.auto_info.auto_yaw_speed = 0.0f;
        }

        vTaskDelay(2);
    }
}

// 角度限位控制保持不变
void auto_aim_control(target_info_t *target, float *yaw_output, float *pitch_output) {
    *yaw_output = target->aim_target_yaw;
    *pitch_output = target->aim_target_pitch > 0.45f ? 0.45f : target->aim_target_pitch;
    *pitch_output = *pitch_output < -0.45f ? -0.45f : *pitch_output;
}