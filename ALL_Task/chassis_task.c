#include "../ALL_Task/chassis_task.h"
#include "../Components/motor/motor.h"
#include "../Bsp/uart/bsp_uart.h"
#include "../Application/robot_global.h"
#include "math.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "../Bsp/LED/bsp_LED.h"

/***********************************************************************************************************************
* 逻辑常量与控制参数 - 集中管理
***********************************************************************************************************************/
#define GIMBAL_YAW_SENS         0.010f
#define GIMBAL_PIT_SENS         0.002f
#define MOUSE_YAW_SENS          0.0004f  // 鼠标横向灵敏度
#define MOUSE_PIT_SENS          0.0002f  // 鼠标纵向灵敏度
#define FOLLOW_P_GAIN           0.5f     // 底盘跟随比例系数
#define RC_DEADZONE             10       // 遥控器摇杆死区
#define YAW_CENTER_OFFSET       0.175f   // YAW轴中心偏移量

// 底盘几何参数
#define MOTOR_RPM_TO_VECTOR     (180.0f * 268.0f / 17.0f)
#define CHASSIS_MAX_RAD         6.28 / 50.0f

/***********************************************************************************************************************
* 静态控制变量
***********************************************************************************************************************/
static float world_yaw_target = 0.0f;
static float world_pit_target = 0.0f;
static float vx_ramp = 0.0f, vy_ramp = 0.0f;
static uint32_t last_rc_tick = 0;

// 自旋方向记忆+回正判定
static float last_rotate_speed = 0.0f; // 记录手动旋转角速度，正=右旋，负=左旋
static const float CORRECT_THRESH = 0.01f; // 角度回正到位阈值(rad)

/***********************************************************************************************************************
* 角度归一化：限制在[-π, π]
***********************************************************************************************************************/
static float Rad_Format(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

/***********************************************************************************************************************
* 底盘任务主函数
***********************************************************************************************************************/
void chassis_task_func(void const *argument) {
    /**************************************** 【初始化区】 ****************************************/
    struct uart_device *Uart = uart_get_device("uart1_dma");
    Uart->Init(Uart, 115200, 8, 'N', 1);

    // 获取底盘电机句柄
    struct motor_device *chassis[4];
    for (int i = 0; i < 4; i++) {
        char name[25];
        sprintf(name, "M3508_CHASSIS_%d", i + 1);
        chassis[i] = motor_get_device(name);
    }
    struct motor_device *yaw_m = motor_get_device("GM6020_YAW");

    // 获取遥控器句柄
    const RC_ctrl_t *rc = robot_ctrl.rc;

    // 模式切换边沿检测
    static uint8_t last_toggle_cmd = 0;
    float wheel_targets[4] = {0};

    /**************************************** 【系统启动保护】 ****************************************/
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);

    /**************************************** 【主循环】 ****************************************/
    while (1) {
        uint32_t current_tick = osKernelSysTick();

        /**************************************** 遥控器掉线保护 ****************************************/
        if (current_tick - rc->last_update_tick > 200) {
            robot_ctrl.monitor.remote_online = 0;
            robot_ctrl.chassis_mode = CHASSIS_RELAX;
        } else {
            robot_ctrl.monitor.remote_online = 1;

            // 底盘跟随/失能 模式切换（上升沿触发）
            uint8_t toggle_cmd = (rc->key.v & KEY_CTRL) || rc->rc.custom_r;
            uint8_t toggle_trigger = (toggle_cmd && !last_toggle_cmd);
            if (toggle_trigger) {
                robot_ctrl.chassis_mode = (robot_ctrl.chassis_mode != CHASSIS_RELAX) ? CHASSIS_RELAX : CHASSIS_FOLLOW;
            }
            last_toggle_cmd = toggle_cmd;
        }

        /**************************************** 底盘控制逻辑 ****************************************/
        if (robot_ctrl.monitor.remote_online) {
            if (robot_ctrl.chassis_mode != CHASSIS_RELAX) {
                /********************* 输入源融合：遥控器+键鼠 *********************/
                float vx_rc = (abs(rc->rc.ch[0]) > RC_DEADZONE) ? rc->rc.ch[0] / 660.0f : 0;
                float vy_rc = (abs(rc->rc.ch[1]) > RC_DEADZONE) ? rc->rc.ch[1] / 660.0f : 0;
                float vw_rc = (abs(rc->rc.wheel) > RC_DEADZONE) ? rc->rc.wheel / 660.0f : 0;

                float vx_kb = 0, vy_kb = 0, vw_kb = 0;
                float speed_ratio = (rc->key.v & KEY_SHIFT) ? 1.0f : 0.5f;

                if (rc->key.v & KEY_W) vy_kb -= speed_ratio;
                if (rc->key.v & KEY_S) vy_kb += speed_ratio;
                if (rc->key.v & KEY_A) vx_kb += speed_ratio;
                if (rc->key.v & KEY_D) vx_kb -= speed_ratio;
                if (rc->key.v & KEY_Q) vw_kb -= 0.5f;
                if (rc->key.v & KEY_E) vw_kb += 0.5f;

                float total_vx = -vx_rc + vx_kb;
                float total_vy = -vy_rc + vy_kb;

                /********************* 各向同性限速 *********************/
                float v_norm = sqrtf(total_vx * total_vx + total_vy * total_vy);
                if (v_norm > speed_ratio) {
                    total_vx = total_vx / v_norm * speed_ratio;
                    total_vy = total_vy / v_norm * speed_ratio;
                }

                /********************* 云台跟随+自旋逻辑 *********************/
                float yaw_m_pos;
                yaw_m->get_status(yaw_m, "POS", &yaw_m_pos);
                float angle_error = Rad_Format(yaw_m_pos + YAW_CENTER_OFFSET);
                Uart->Print(Uart, "%f,%f\r\n", robot_ctrl.gimbal.yaw, yaw_m_pos);

                float vw_final = 0;
                if (fabsf(vw_rc) > 0.05f || fabsf(vw_kb) > 0.01f) {
                    vw_final = vw_rc + vw_kb;
                    last_rotate_speed = vw_final;
                } else {
                    if (fabsf(angle_error) > CORRECT_THRESH && fabsf(last_rotate_speed) > 0.01f) {
                        vw_final = (last_rotate_speed > 0 ? 1 : -1) * fabsf(angle_error) * FOLLOW_P_GAIN;
                    } else {
                        vw_final = -angle_error * FOLLOW_P_GAIN;
                        last_rotate_speed = 0.0f;
                    }
                }
                robot_ctrl.chassis.yaw_speed = vw_final * CHASSIS_MAX_RAD;

                /********************* 随动坐标系变换 *********************/
                float final_vx = total_vx * cosf(angle_error) - total_vy * sinf(angle_error);
                float final_vy = total_vx * sinf(angle_error) + total_vy * cosf(angle_error);

                /********************* 逆运动学计算+电机限幅 *********************/
                wheel_targets[0] = (-final_vx - final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                wheel_targets[1] = (-final_vx + final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                wheel_targets[2] = (final_vx + final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                wheel_targets[3] = (final_vx - final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;

                for (int i = 0; i < 4; i++) {
                    if (chassis[i]) {
                        wheel_targets[i] = fabsf(wheel_targets[i]) > 10000 ? (wheel_targets[i]>0?10000:-10000) : wheel_targets[i];
                        chassis[i]->set_target(chassis[i], 1, wheel_targets[i]);
                    }
                }
            }
        } else {
            // 遥控器掉线 - 红灯闪（全局最高优先级），电机零速指令
            LED_GREEN_RESET(); LED_BLUE_RESET(); LED_RED_Toggle();
            for (int i = 0; i < 4; i++) { if (chassis[i]) chassis[i]->set_target(chassis[i], 1, 0); }
            osDelay(100);
        }
        osDelay(2);
    }
}