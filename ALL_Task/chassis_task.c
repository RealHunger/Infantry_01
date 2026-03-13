#include "../ALL_Task/chassis_task.h"
#include "../Components/motor/motor.h"
#include "../Bsp/uart/bsp_uart.h"
#include "../Application/robot_global.h"
#include "math.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "../Bsp/LED/bsp_LED.h"
#include "../Components/referee/referee.h"  // 【新增】引入裁判系统组件

/***********************************************************************************************************************
* 极致速度参数 + 云台跟随参数 双适配（十字全向轮专用，保留所有全速优化）
***********************************************************************************************************************/
#define RC_DEADZONE             8       // 缩小死区：提升操作灵敏度
#define LIMIT_SPEED             60000.0f// 拉满电机限幅：适配M3508最大输出（硬件安全上限）
#define MOTOR_RPM_TO_VECTOR     (180.0f * 268.0f / 17.0f) * 2.0f  // 转速系数×2：极致速度核心
#define SPEED_RATIO_FULL        1.2f    // SHIFT全速档：超100%输出
#define SPEED_RATIO_NORMAL      0.8f    // 普通档：比原0.5快60%，兼顾操控
#define FOLLOW_P_GAIN           0.6f    // 云台跟随比例系数【全速适配】响应快不抖动
#define YAW_CENTER_OFFSET       0.175f  // YAW轴中心偏移量 沿用你的原值
#define CORRECT_THRESH          0.01f   // 角度回正到位阈值(rad)

/***********************************************************************************************************************
* 静态控制变量：加回云台跟随核心变量 + 保留全速变量
***********************************************************************************************************************/
static float vx_ramp = 0.0f, vy_ramp = 0.0f;
static uint32_t last_rc_tick = 0;
static float last_rotate_speed = 0.0f; // 自旋方向记忆：正=右旋，负=左旋

/***********************************************************************************************************************
* 角度归一化 核心函数：云台跟随必须，限制角度在[-π, π]，防止溢出
***********************************************************************************************************************/
static float Rad_Format(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

/***********************************************************************************************************************
* 底盘任务主函数 - 终极版【极致速度+云台精准跟随】
***********************************************************************************************************************/
void chassis_task_func(void const *argument) {
    /**************************************** 【初始化区】 ****************************************/
    struct uart_device *Uart = uart_get_device("uart1_dma");
    Uart->Init(Uart, 115200, 8, 'N', 1);

    // 获取底盘4个电机句柄（十字全向轮：1=右 2=前 3=左 4=后，正方向对准2号轮）
    struct motor_device *chassis[4];
    for (int i = 0; i < 4; i++) {
        char name[25];
        sprintf(name, "M3508_CHASSIS_%d", i + 1);
        chassis[i] = motor_get_device(name);
    }

    // ★加回云台YAW电机句柄【云台跟随核心】
    struct motor_device *yaw_m = motor_get_device("GM6020_YAW");

    // 获取遥控器句柄
    const RC_ctrl_t *rc = robot_ctrl.rc;

    // 模式切换边沿检测
    static uint8_t last_toggle_cmd = 0;
    float wheel_targets[4] = {0};

    /**************************************** 【系统启动保护】 ****************************************/
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(500); // 缩短启动延时：更快进入工作状态

    /**************************************** 【主循环】 ****************************************/
    while (1) {
        uint32_t current_tick = osKernelSysTick();


        // ==========================================================
        // 【新增 1】裁判系统仪表盘打印 (每 500ms 打印一次)
        // 函数内部自带限频锁，直接调用即可，绝对不会阻塞 RTOS
        // ==========================================================
        Referee_Debug_Print();

        // ==========================================================
        // 【新增 2】裁判系统 CAN 数据转发 (限制为 50Hz，即每 20ms 发送一次)
        // ==========================================================
        static uint32_t last_can_send_tick = 0;
        if (current_tick - last_can_send_tick >= 20) {
            Referee_CAN_Forward();
            last_can_send_tick = current_tick;
        }


        /**************************************** 遥控器掉线保护 ****************************************/
        if (current_tick - rc->last_update_tick > 200) {
            robot_ctrl.monitor.remote_online = 0;
            robot_ctrl.chassis_mode = CHASSIS_RELAX;
        } else {
            robot_ctrl.monitor.remote_online = 1;

            // 底盘使能/失能 模式切换（上升沿触发：KEY_CTRL+自定义R）
            uint8_t toggle_cmd = (rc->key.v & KEY_CTRL) || rc->rc.custom_r;
            uint8_t toggle_trigger = (toggle_cmd && !last_toggle_cmd);
            if (toggle_trigger) {
                robot_ctrl.chassis_mode = (robot_ctrl.chassis_mode != CHASSIS_RELAX) ? CHASSIS_RELAX : CHASSIS_FOLLOW;
            }
            last_toggle_cmd = toggle_cmd;
        }

        /**************************************** 底盘全速+云台跟随控制逻辑 ****************************************/
        if (robot_ctrl.monitor.remote_online && robot_ctrl.chassis_mode != CHASSIS_RELAX) {
            /********************* 1. 遥控+键鼠输入融合（无任何衰减，全速映射） *********************/
            float vx_rc = (abs(rc->rc.ch[0]) > RC_DEADZONE) ? (float)rc->rc.ch[0] / 660.0f : 0.0f;
            float vy_rc = (abs(rc->rc.ch[1]) > RC_DEADZONE) ? (float)rc->rc.ch[1] / 660.0f : 0.0f;
            float vw_rc = (abs(rc->rc.wheel) > RC_DEADZONE) ? (float)rc->rc.wheel / 660.0f : 0.0f;

            // 键鼠速度档（SHIFT=全速1.2倍，普通=0.8倍，均比原版快）
            float speed_ratio = (rc->key.v & KEY_SHIFT) ? SPEED_RATIO_FULL : SPEED_RATIO_NORMAL;
            float vx_kb = 0.0f, vy_kb = 0.0f, vw_kb = 0.0f;

            // 键鼠方向：W前 S后 A左 D右 Q左旋 E右旋（全速无衰减）
            if (rc->key.v & KEY_W) vy_kb -= speed_ratio;
            if (rc->key.v & KEY_S) vy_kb += speed_ratio;
            if (rc->key.v & KEY_A) vx_kb += speed_ratio;
            if (rc->key.v & KEY_D) vx_kb -= speed_ratio;
            if (rc->key.v & KEY_Q) vw_kb -= speed_ratio * 0.8f; // 自旋略降一点，防止侧翻
            if (rc->key.v & KEY_E) vw_kb += speed_ratio * 0.8f;

            // 总速度：遥控+键鼠直接叠加（无负号衰减，完美匹配十字轮物理方向）
            float total_vx = vx_rc + vx_kb;
            float total_vy = vy_rc + vy_kb;
            float total_vw = vw_rc + vw_kb;

            /********************* 2. 各向同性限速（仅防止超全速档，无额外衰减） *********************/
            float v_norm = sqrtf(total_vx * total_vx + total_vy * total_vy);
            if (v_norm > speed_ratio) {
                total_vx = total_vx / v_norm * speed_ratio;
                total_vy = total_vy / v_norm * speed_ratio;
            }

            /********************* ★云台跟随核心逻辑【完美加回，无速度衰减】 *********************/
            float yaw_m_pos;
            yaw_m->get_status(yaw_m, "POS", &yaw_m_pos);          // 获取云台YAW轴实时角度
            float angle_error = Rad_Format(yaw_m_pos + YAW_CENTER_OFFSET); // 计算归一化角度误差

            // 自旋优先级：手动自旋>云台跟随回正，松手自动回正云台，保留记忆不漂移
            float vw_final = 0;
            if (fabsf(total_vw) > 0.01f) {
                vw_final = total_vw;
                last_rotate_speed = vw_final;
            } else {
                if (fabsf(angle_error) > CORRECT_THRESH && fabsf(last_rotate_speed) > 0.01f) {
                    vw_final = (last_rotate_speed > 0 ? 1 : -1) * fabsf(angle_error) * FOLLOW_P_GAIN;
                } else {
                    vw_final = -angle_error * FOLLOW_P_GAIN;
                    last_rotate_speed = 0.0f;
                }
            }
            robot_ctrl.chassis.yaw_speed = vw_final;

            /********************* ★坐标系旋转变换【云台跟随关键】速度无损映射 *********************/
            float final_vx = total_vx * cosf(angle_error) - total_vy * sinf(angle_error);
            float final_vy = total_vx * sinf(angle_error) + total_vy * cosf(angle_error);

            /********************* 3. 十字全向轮逆运动学【完全保留你修改的电机符号！】 *********************/
            // 核心：保留你手动修正的2/4号电机正负号，不用再调转向，正方向对准2号前轮，速度拉满
            wheel_targets[0] = (-final_vx - vw_final) * MOTOR_RPM_TO_VECTOR;  // 1号：右轮
            wheel_targets[1] = ( total_vy - vw_final) * MOTOR_RPM_TO_VECTOR;  // 2号：前轮【底盘正方向】你的修正符号
            wheel_targets[2] = ( final_vx - vw_final) * MOTOR_RPM_TO_VECTOR;  // 3号：左轮
            wheel_targets[3] = (-total_vy - vw_final) * MOTOR_RPM_TO_VECTOR;  // 4号：后轮 你的修正符号

            /********************* 4. 电机硬件限幅（仅防烧电机，拉满到60000） *********************/
            for (int i = 0; i < 4; i++) {
                if (chassis[i] != NULL) {
                    // 限幅：超过最大值直接拉满，无中间衰减
                    wheel_targets[i] = (wheel_targets[i] > LIMIT_SPEED)  ? LIMIT_SPEED  : wheel_targets[i];
                    wheel_targets[i] = (wheel_targets[i] < -LIMIT_SPEED) ? -LIMIT_SPEED : wheel_targets[i];
                    // 下发电机目标值（全速输出）
                    chassis[i]->set_target(chassis[i], 1, wheel_targets[i]);
                }
            }
        } else {
            /**************************************** 失能/掉线保护 ****************************************/
            // 电机零速指令（最高优先级），红灯闪烁提示
            LED_GREEN_RESET(); LED_BLUE_RESET(); LED_RED_Toggle();
            for (int i = 0; i < 4; i++) {
                if (chassis[i] != NULL) chassis[i]->set_target(chassis[i], 1, 0.0f);
            }
            osDelay(100);
        }
        osDelay(1); // 缩短任务延时：从2ms→1ms，提升控制频率，响应更快
    }
}