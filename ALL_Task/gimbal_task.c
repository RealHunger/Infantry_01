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

/***********************************************************************************************************************
* 宏定义-集中管理（分区归类，简明注释）
***********************************************************************************************************************/
// 云台控制参数
#define RC_DEADZONE         10
#define MOUSE_YAW_SENS      0.00005f
#define MOUSE_PIT_SENS      0.00005f
#define RC_YAW_SENS         0.005f
#define RC_PIT_SENS         0.005f

// 俯仰角物理限位【核心】
#define PITCH_UP_LIMIT      0.35f
#define PITCH_DOWN_LIMIT    -0.45f

// 发射机构参数
#define STIR_REVERSE_CURRENT   8500   // 堵转电流阈值
#define STIR_BLOCK_TIME        150    // 堵转判定时间(ms)
#define STIR_REVERSE_TIME      200    // 反转逃逸时间(ms)
#define SHOOT_FW_SPEED         5500.0f   // 摩擦轮转速
#define STIR_SHOOT_SPEED       5000.0f   // 拨弹转速
#define STIR_REVERSE_SPEED     2500.0f   // 反转转速

// 电机安全限幅
#define M3508_MAX_SPEED        10000.0f
#define M2006_MAX_SPEED        6000.0f

/***********************************************************************************************************************
* 角度归一化：[-π, π]
***********************************************************************************************************************/
static float Rad_Format(float angle) {
    while (angle >  M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

/***********************************************************************************************************************
* 云台任务主函数
***********************************************************************************************************************/
void gimbal_task_func(void const * argument) {
    /**************************************** 【初始化区】 ****************************************/
    struct uart_device* Uart = uart_get_device("uart1_dma");
    Uart->Init(Uart, 115200, 8, 'N', 1);

    // 获取电机句柄
    const struct motor_device *yaw_m = motor_get_device("GM6020_YAW");
    const struct motor_device *pit_m = motor_get_device("J4310_PITCH");
    struct motor_device *shoot_l = motor_get_device("M3508_SHOOT_L");
    struct motor_device *shoot_r = motor_get_device("M3508_SHOOT_R");
    struct motor_device *stir_m  = motor_get_device("M2006_TRIGGER");

    // 自瞄初始化
    struct usb_device* usb = usb_get_device();
    usb->Init(usb);
    auto_aim_init(usb);

    // 遥控器句柄
    const RC_ctrl_t *rc = robot_ctrl.rc;

    /**************************************** 静态状态变量 ****************************************/
    static uint8_t last_relax_toggle = 0;
    static uint8_t last_mode_toggle = 0;
    static uint8_t is_initialized = 0;
    static uint8_t last_f_key = 0;
    static uint8_t last_sw_state = RC_SW_N;
    static enum { STIR_NORMAL, STIR_BLOCKING, STIR_REVERSING } stir_state = STIR_NORMAL;
    static uint32_t block_start_tick = 0;
    static uint32_t reverse_end_tick = 0;

    float world_yaw_target = 0.0f;
    float world_pit_target = 0.0f;

    /**************************************** 【系统启动保护】 ****************************************/
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);

    /**************************************** 【主循环】 ****************************************/
    while (1) {
        uint32_t current_tick = osKernelSysTick();

        /**************************************** 遥控器掉线全局保护 ****************************************/
        if (current_tick - rc->last_update_tick > 200) {
            robot_ctrl.monitor.remote_online = 0;
            robot_ctrl.gimbal_mode = GIMBAL_RELAX;
            robot_ctrl.shoot_mode = SHOOT_STOP;
            is_initialized = 0;
            // 掉线急停：所有发射电机零速
            shoot_l->set_target(shoot_l, 1, 0);
            shoot_r->set_target(shoot_r, 1, 0);
            stir_m->set_target(stir_m, 1, 0);
            // 遥控器掉线 - 红灯闪（全局最高优先级）
            LED_GREEN_RESET(); LED_BLUE_RESET(); LED_RED_Toggle();
            osDelay(100);
        } else {
            robot_ctrl.monitor.remote_online = 1;

            // F键/档位 发射就绪/停止切换
            if ((rc->key.v & KEY_F) && !last_f_key) {
                robot_ctrl.shoot_mode = (robot_ctrl.shoot_mode == SHOOT_STOP) ? SHOOT_READY : SHOOT_STOP;
            }
            last_f_key = (rc->key.v & KEY_F);

            if (rc->rc.sw != last_sw_state) {
                robot_ctrl.shoot_mode = (rc->rc.sw == RC_SW_S) ? SHOOT_READY : SHOOT_STOP;
                last_sw_state = rc->rc.sw;
            }

            /********************* 云台模式切换：失能/手动/自瞄 *********************/
            uint8_t relax_cmd = (rc->rc.pause) || (rc->key.v & KEY_C);
            uint8_t relax_trigger = (relax_cmd && !last_relax_toggle);
            uint8_t mode_cmd = (rc->rc.custom_l) || (rc->key.v & KEY_E);
            uint8_t mode_trigger = (mode_cmd && !last_mode_toggle);

            if (relax_trigger) {
                robot_ctrl.gimbal_mode = (robot_ctrl.gimbal_mode == GIMBAL_RELAX) ? GIMBAL_REMOTE : GIMBAL_RELAX;
                is_initialized = 0;
            }
            if (mode_trigger && robot_ctrl.gimbal_mode != GIMBAL_RELAX) {
                robot_ctrl.gimbal_mode = (robot_ctrl.gimbal_mode == GIMBAL_REMOTE) ? GIMBAL_AUTO : GIMBAL_REMOTE;
            }

            last_relax_toggle = relax_cmd;
            last_mode_toggle = mode_cmd;

            /**************************************** 云台角度闭环控制 ****************************************/
            if (robot_ctrl.gimbal_mode != GIMBAL_RELAX) {
                // 首次使能：同步当前角度防甩动
                if (is_initialized == 0) {
                    world_yaw_target = robot_ctrl.gimbal.yaw;
                    world_pit_target = robot_ctrl.gimbal.pitch;
                    is_initialized = 1;
                }

                /********************* 手动模式：摇杆+鼠标控制 *********************/
                if (robot_ctrl.gimbal_mode == GIMBAL_REMOTE) {
                    // 云台使能-手动 → 绿灯常亮
                    LED_RED_RESET(); LED_BLUE_RESET(); LED_GREEN_SET();
                    float ry = (abs(rc->rc.ch[2]) > RC_DEADZONE) ? rc->rc.ch[2] / 660.0f : 0.0f;
                    float rx = (abs(rc->rc.ch[3]) > RC_DEADZONE) ? rc->rc.ch[3] / 660.0f : 0.0f;
                    float mouse_x = (float)rc->mouse.x * MOUSE_YAW_SENS;
                    float mouse_y = (float)rc->mouse.y * MOUSE_PIT_SENS;

                    world_pit_target -= (ry * RC_PIT_SENS) + mouse_y;
                    world_yaw_target -= (rx * RC_YAW_SENS) + mouse_x;

                    // 俯仰角目标值限位
                    if(world_pit_target > PITCH_UP_LIMIT)  world_pit_target = PITCH_UP_LIMIT;
                    if(world_pit_target < PITCH_DOWN_LIMIT)world_pit_target = PITCH_DOWN_LIMIT;
                }
                /********************* 自瞄模式：上位机数据解析 *********************/
                else if (robot_ctrl.gimbal_mode == GIMBAL_AUTO) {
                    target_info_t target_info;
                    if (parse_target_data(&target_info) == 1) {
                        // 云台使能-自瞄 → 蓝灯常亮
                        LED_RED_RESET(); LED_GREEN_RESET(); LED_BLUE_SET();
                        world_yaw_target = target_info.aim_target_yaw;
                        world_pit_target = target_info.aim_target_pitch;
                        // 自瞄俯仰角限位
                        if(world_pit_target > PITCH_UP_LIMIT)  world_pit_target = PITCH_UP_LIMIT;
                        if(world_pit_target < PITCH_DOWN_LIMIT)world_pit_target = PITCH_DOWN_LIMIT;
                    } else {
                        // 自瞄丢目标 -> 蓝灯闪烁
                        LED_RED_RESET(); LED_BLUE_Toggle(); LED_GREEN_RESET();
                    }
                }

                /********************* 角度闭环输出+双重限位 *********************/
                float cur_yaw, cur_pit;
                yaw_m->get_status(yaw_m, "POS", &cur_yaw);
                pit_m->get_status(pit_m, "POS", &cur_pit);

                float yaw_out = cur_yaw + Rad_Format(world_yaw_target - robot_ctrl.gimbal.yaw + robot_ctrl.chassis.yaw_speed);
                float pit_out = cur_pit - (world_pit_target - robot_ctrl.gimbal.pitch);

                if (pit_out > PITCH_UP_LIMIT) pit_out = PITCH_UP_LIMIT;
                if (pit_out < PITCH_DOWN_LIMIT) pit_out = PITCH_DOWN_LIMIT;

                yaw_m->set_target(yaw_m, 1, yaw_out);
                pit_m->set_target(pit_m, 1, pit_out);
            }
            /********************* 云台失能状态 *********************/
            else if (robot_ctrl.gimbal_mode == GIMBAL_RELAX) {
                // 云台失能 → 红灯常亮
                LED_GREEN_RESET(); LED_BLUE_RESET(); LED_RED_SET();
            }

            /**************************************** 发射机构控制 ****************************************/
            /********************* 摩擦轮控制 *********************/
            if (robot_ctrl.shoot_mode == SHOOT_READY) {
                float shoot_l_speed = (SHOOT_FW_SPEED > M3508_MAX_SPEED) ? M3508_MAX_SPEED : SHOOT_FW_SPEED;
                float shoot_r_speed = (-SHOOT_FW_SPEED < -M3508_MAX_SPEED) ? -M3508_MAX_SPEED : -SHOOT_FW_SPEED;
                shoot_l->set_target(shoot_l, 1,  shoot_l_speed);
                shoot_r->set_target(shoot_r, 1, shoot_r_speed);
            } else {
                shoot_l->set_target(shoot_l, 1, 0);
                shoot_r->set_target(shoot_r, 1, 0);
            }

            /********************* 拨弹轮+堵转逃逸逻辑 *********************/
            float stir_torque = 0;
            stir_m->get_status(stir_m, "CURRENT", &stir_torque);
            uint8_t shoot_cmd = (rc->mouse.press_l || rc->rc.trigger) && (robot_ctrl.shoot_mode == SHOOT_READY);
            uint8_t reverse_cmd = (rc->mouse.press_m || rc->rc.sw == RC_SW_C);

            if (reverse_cmd) {
                float rev_speed = STIR_REVERSE_SPEED > M2006_MAX_SPEED ? M2006_MAX_SPEED : STIR_REVERSE_SPEED;
                stir_m->set_target(stir_m, 1, rev_speed);
                stir_state = STIR_NORMAL;
            } else if (shoot_cmd) {
                if (stir_state == STIR_REVERSING) {
                    stir_m->set_target(stir_m, 1, 2500);
                    if (current_tick > reverse_end_tick) stir_state = STIR_NORMAL;
                } else {
                    float shoot_speed = (-STIR_SHOOT_SPEED < -M2006_MAX_SPEED) ? -M2006_MAX_SPEED : -STIR_SHOOT_SPEED;
                    stir_m->set_target(stir_m, 1, shoot_speed);
                    // 堵转检测
                    if (fabsf(stir_torque) > STIR_REVERSE_CURRENT && fabsf(stir_torque) < 30000) {
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
            } else {
                stir_m->set_target(stir_m, 1, 0);
                stir_state = STIR_NORMAL;
            }
        }
        osDelay(2);
    }
}