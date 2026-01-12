//
// Created by 14717 on 2026/1/11.
//

#include "gimbal_task.h"
#include "../Application/robot_global.h"
#include "../../Components/motor/motor.h"
#include "math.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "../../Bsp/uart/bsp_uart.h"
#include "../../Bsp/led/bsp_led.h"
#include "../../Components/remote/remote.h"
#include "../../Application/auto_aim.h"


#define RC_DEADZONE         10
#define MOUSE_YAW_SENS      0.00005f
#define MOUSE_PIT_SENS      0.00005f
#define RC_YAW_SENS     0.005f
#define RC_PIT_SENS     0.005f

/* --- 发射机构逻辑常量 --- */
#define STIR_REVERSE_CURRENT   8500   // 堵转电流阈值
#define STIR_BLOCK_TIME        150    // 判定堵转持续时间(ms)
#define STIR_REVERSE_TIME      200    // 自动反转逃逸时间(ms)
#define SHOOT_FW_SPEED         5500.0f   // 摩擦轮起旋转速
#define STIR_SHOOT_SPEED        5000.0f
#define STIR_REVERSE_SPEED     2500.0f

// 角度归一化
static float Rad_Format(float angle) {
    while (angle >  M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void gimbal_task_func(void const * argument) {
    /* --- 1. 初始化 --- */
    // 调试串口
    struct uart_device* Uart = uart_get_device("uart1_dma");
    Uart->Init(Uart, 115200, 8, 'N', 1);

    // 获取电机设备句柄
    const struct motor_device *yaw_m = motor_get_device("GM6020_YAW");
    const struct motor_device *pit_m = motor_get_device("J4310_PITCH");

    // 发射机构电机
    struct motor_device *shoot_l = motor_get_device("M3508_SHOOT_L");
    struct motor_device *shoot_r = motor_get_device("M3508_SHOOT_R");
    struct motor_device *stir_m  = motor_get_device("M2006_TRIGGER");

    // 与上位机通信USB
    struct usb_device* usb = usb_get_device();
    usb->Init(usb);
    auto_aim_init(usb);

    // 获取控制设别句柄
    const RC_ctrl_t *rc = robot_ctrl.rc;

    // 静态状态记录
    static uint8_t last_relax_toggle = 0;  // 用于 Pause/X 切换使能
    static uint8_t last_mode_toggle = 0;   // 用于 E/Custom_L 切换自瞄
    static uint8_t is_initialized = 0;

    static uint8_t last_f_key = 0;
    static uint8_t last_sw_state = RC_SW_N;
    static enum { STIR_NORMAL, STIR_BLOCKING, STIR_REVERSING } stir_state = STIR_NORMAL;
    static uint32_t block_start_tick = 0;
    static uint32_t reverse_end_tick = 0;

    float world_yaw_target = 0.0f;
    float world_pit_target = 0.0f;


    /******************************************************************************************************************/
    // 系统启动保护
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);

    /******************************************************************************************************************/
    // 主循环
    while (1) {
        uint32_t current_tick = osKernelSysTick();

        /**************************************************************************************************************/
        if (current_tick - rc->last_update_tick > 200) {
            // 遥控器掉线保护
            robot_ctrl.monitor.remote_online = 0;
            robot_ctrl.gimbal_mode = GIMBAL_RELAX;
            robot_ctrl.shoot_mode = SHOOT_STOP;
            is_initialized = 0;
        }
        else {
            robot_ctrl.monitor.remote_online = 1;


            // 键盘 F 键边沿检测
            if ((rc->key.v & KEY_F) && !last_f_key) {
                robot_ctrl.shoot_mode = (robot_ctrl.shoot_mode == SHOOT_STOP) ? SHOOT_READY : SHOOT_STOP;
            }
            last_f_key = (rc->key.v & KEY_F);

            // 遥控器 SW 挡位边沿检测
            if (rc->rc.sw != last_sw_state) {
                if (rc->rc.sw == RC_SW_S)      robot_ctrl.shoot_mode = SHOOT_READY;
                else if (rc->rc.sw == RC_SW_N) robot_ctrl.shoot_mode = SHOOT_STOP;
                last_sw_state = rc->rc.sw;
            }


            /* 输入抽象与边缘检测 */
            // 使能/失能切换 (Pause 或 X)
            uint8_t relax_cmd = (rc->rc.pause) || (rc->key.v & KEY_C);
            uint8_t relax_trigger = (relax_cmd && !last_relax_toggle);

            // 手动/自瞄切换 (Custom_L 或 E)
            uint8_t mode_cmd = (rc->rc.custom_l) || (rc->key.v & KEY_E);
            uint8_t mode_trigger = (mode_cmd && !last_mode_toggle);

            // 状态逻辑处理
            // 处理使能翻转
            if (relax_trigger) {
                if (robot_ctrl.gimbal_mode == GIMBAL_RELAX) {
                    robot_ctrl.gimbal_mode = GIMBAL_REMOTE; // 开启默认进入手动
                } else {
                    robot_ctrl.gimbal_mode = GIMBAL_RELAX;
                    is_initialized = 0; // 关闭时重置初始化标记
                }
            }

            // 在使能状态下处理模式切换
            if (mode_trigger && robot_ctrl.gimbal_mode != GIMBAL_RELAX) {
                robot_ctrl.gimbal_mode = (robot_ctrl.gimbal_mode == GIMBAL_REMOTE) ?
                                          GIMBAL_AUTO : GIMBAL_REMOTE;
            }

            last_relax_toggle = relax_cmd;
            last_mode_toggle = mode_cmd;
        }

        /* 运动控制执行 */
        if (robot_ctrl.monitor.remote_online && robot_ctrl.gimbal_mode != GIMBAL_RELAX) {
            // 首次进入使能状态时，同步当前机械角度，防止云台疯甩
            if (is_initialized == 0) {
                world_yaw_target = robot_ctrl.gimbal.yaw;
                world_pit_target = robot_ctrl.gimbal.pitch;
                is_initialized = 1;
            }

            // 灯光指示：工作模式
            LED_RED_RESET(); LED_BLUE_SET(); LED_GREEN_RESET();


            if (robot_ctrl.gimbal_mode == GIMBAL_REMOTE) {
                // --- A. 摇杆控制 ---
                float ry = (abs(rc->rc.ch[2]) > RC_DEADZONE) ? rc->rc.ch[2] / 660.0f : 0.0f;
                float rx = (abs(rc->rc.ch[3]) > RC_DEADZONE) ? rc->rc.ch[3] / 660.0f : 0.0f;

                // --- B. 鼠标控制 (新增) ---
                // 鼠标数据是增量，直接叠加到目标值上
                float mouse_x = (float)rc->mouse.x * MOUSE_YAW_SENS;
                float mouse_y = (float)rc->mouse.y * MOUSE_PIT_SENS;

                world_pit_target -= (ry * RC_PIT_SENS) + mouse_y;
                world_yaw_target -= (rx * RC_YAW_SENS) + mouse_x;

            }
            else if (robot_ctrl.gimbal_mode == GIMBAL_AUTO) {
                target_info_t target_info;

                if (parse_target_data(&target_info) == 1) {

                    LED_BLUE_RESET();
                    world_yaw_target = target_info.aim_target_yaw;
                    world_pit_target = target_info.aim_target_pitch;
                }
                else {
                    LED_BLUE_SET();
                }
            }

            float cur_yaw, cur_pit;
            yaw_m->get_status(yaw_m, "POS", &cur_yaw);
            pit_m->get_status(pit_m, "POS", &cur_pit);

            float yaw_out = cur_yaw + Rad_Format(world_yaw_target - robot_ctrl.gimbal.yaw + robot_ctrl.chassis.yaw_speed);
            float pit_out = cur_pit - (world_pit_target - robot_ctrl.gimbal.pitch);

            // 俯仰角限幅
            if (pit_out > 0.35f) pit_out = 0.35f;
            if (pit_out < -0.45f) pit_out = -0.45f;

            // 发送控制指令给电机
            yaw_m->set_target(yaw_m, 1, yaw_out);
            pit_m->set_target(pit_m, 1, pit_out);
        }
        else if (robot_ctrl.monitor.remote_online && robot_ctrl.gimbal_mode == GIMBAL_RELAX) {
            //已连接但失能：红灯常亮
            LED_GREEN_RESET(); LED_BLUE_RESET(); LED_RED_SET();
        }

        if (robot_ctrl.monitor.remote_online && robot_ctrl.shoot_mode == SHOOT_READY) {
            // --- A. 摩擦轮执行 ---
            if (robot_ctrl.shoot_mode == SHOOT_READY) {
                shoot_l->set_target(shoot_l, 1,  SHOOT_FW_SPEED);
                shoot_r->set_target(shoot_r, 1, -SHOOT_FW_SPEED);
            } else {
                shoot_l->set_target(shoot_l, 1, 0);
                shoot_r->set_target(shoot_r, 1, 0);
            }

            // --- B. 拨弹轮执行 (发射与堵转) ---
            float stir_torque = 0;
            stir_m->get_status(stir_m, "CURRENT", &stir_torque);

            // 指令判定叠加
            uint8_t shoot_cmd = (rc->mouse.press_l || rc->rc.trigger) && (robot_ctrl.shoot_mode == SHOOT_READY);
            uint8_t reverse_cmd = (rc->mouse.press_m || rc->rc.sw == RC_SW_C);

            if (reverse_cmd) {
                // 手动反转/清弹
                stir_m->set_target(stir_m, 1, STIR_REVERSE_SPEED);
                stir_state = STIR_NORMAL;
            }
            else if (shoot_cmd) {
                if (stir_state == STIR_REVERSING) {
                    stir_m->set_target(stir_m, 1, 2500); // 执行逃逸反转
                    if (current_tick > reverse_end_tick) stir_state = STIR_NORMAL;
                }
                else {
                    stir_m->set_target(stir_m, 1, -STIR_SHOOT_SPEED); // 正常发射
                    // 堵转检测逻辑
                    if (fabsf(stir_torque) > STIR_REVERSE_CURRENT) {
                        if (stir_state == STIR_NORMAL) {
                            stir_state = STIR_BLOCKING;
                            block_start_tick = current_tick;
                        } else if (current_tick - block_start_tick > STIR_BLOCK_TIME) {
                            stir_state = STIR_REVERSING;
                            reverse_end_tick = current_tick + STIR_REVERSE_TIME;
                        }
                    } else {
                        stir_state = STIR_NORMAL;
                    }
                }
            }
            else {
                stir_m->set_target(stir_m, 1, 0);
                stir_state = STIR_NORMAL;
            }
        }


        osDelay(2);
    }
}

