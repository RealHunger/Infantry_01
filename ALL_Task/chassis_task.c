#include "../ALL_Task/chassis_task.h"
#include "../Components/motor/motor.h"
#include "../Bsp/uart/bsp_uart.h"
#include "../Application/robot_global.h"
#include "math.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "stdio.h"

/***********************************************************************************************************************
* 文 件 说 明：底盘任务主函数 - 全向轮底盘核心控制任务
* 硬件适配：4颗M3508直流减速电机(底盘驱动) + GM6020云台航向电机(跟随基准)
* 核心功能：1.底盘双模式切换【放松模式/云台跟随模式】 2.遥控器摇杆+键鼠WASD双输入控制 3.云台随动坐标系变换
*          4.各向同性速度限幅 5.手动旋转优先于自动跟随 6.遥控器掉线全局急停保护 7.全向轮逆运动学解算
* 任务周期：2ms(osDelay(2))，保证控制实时性，兼顾CPU利用率
***********************************************************************************************************************/
/* --- 逻辑常量与控制参数【所有可调参数集中管理，改参只改此处】 --- */
#define FOLLOW_P_GAIN           0.5f     // 底盘跟随云台P环增益，值越大跟随越灵敏，0.3~0.8最佳
#define RC_DEADZONE             10       // 遥控器摇杆死区阈值，防止摇杆漂移误触发
#define YAW_CENTER_OFFSET       0.0f     // 云台航向零点偏移补偿，校准机械零点使用

// 底盘几何参数配置【全向轮核心换算参数，标定值不可随意修改】
#define MOTOR_RPM_TO_VECTOR     (180.0f * 268.0f / 17.0f)  // M3508转速→全向轮速度矢量 换算系数
#define CHASSIS_MAX_RAD         6.28 / 50.0f              // 底盘最大旋转角速度限幅，防止旋转过快失控

/***********************************************************************************************************************
* 弧度角度归一化处理，将任意弧度值限制在 [-π, π] 区间内，保证角度计算连续性，无累计误差
***********************************************************************************************************************/
static float Rad_Format(float angle) {
    while (angle >  M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

/***********************************************************************************************************************
* 底盘任务主函数 - FreeRTOS独立任务，优先级高，全向轮底盘所有控制逻辑入口
***********************************************************************************************************************/
void chassis_task_func(void const * argument) {
    /**************************************** 第一步：外设与设备句柄初始化 ****************************************/
    // 初始化调试串口1 DMA，波特率115200，用于调试数据打印
    struct uart_device* Uart = uart_get_device("uart1_dma");
    Uart->Init(Uart, 115200, 8, 'N', 1);

    // 获取4颗底盘M3508电机设备句柄，电机名称与注册名一一对应
    struct motor_device *chassis[4];
    for(int i=0; i<4; i++) {
        char name[25]; sprintf(name, "M3508_CHASSIS_%d", i+1);
        chassis[i] = motor_get_device(name);
    }
    // 获取云台航向GM6020电机句柄，用于读取云台实时角度，实现底盘跟随逻辑
    struct motor_device *yaw_m = motor_get_device("GM6020_YAW");

    // 获取遥控器全局控制句柄，读取摇杆/按键/键鼠所有指令数据
    const RC_ctrl_t *rc = robot_ctrl.rc;

    // 底盘模式切换按键 上升沿检测防抖变量，防止按键抖动导致模式频繁切换
    static uint8_t last_toggle_cmd = 0;

    // 全向轮组4个电机目标转速缓存数组，初始化清零，防止上电电机猛冲
    float wheel_targets[4] = {0};

    /**************************************** 第二步：系统启动安全保护 ****************************************/
    // 等待传感器初始化就绪，防止传感器未就绪导致数据异常
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);   // 延时1秒，等待电机/外设完全上电稳定，硬件必备保护

    /**************************************** 第三步：任务主循环【死循环，永不退出】 ****************************************/
    while (1) {
        uint32_t current_tick = osKernelSysTick();  // 获取当前系统滴答值，用于超时检测

        /********************* 【最高优先级】遥控器掉线全局急停保护 *********************/
        if (current_tick - rc->last_update_tick > 200) {
            robot_ctrl.monitor.remote_online = 0;       // 标记遥控器离线状态
            robot_ctrl.chassis_mode = CHASSIS_RELAX;    // 底盘强制进入放松模式，电机解锁无阻力
        }
        /********************* 遥控器在线：执行正常底盘控制逻辑 *********************/
        else {
            robot_ctrl.monitor.remote_online = 1;       // 标记遥控器在线状态

            /********************* 子逻辑1：底盘工作模式切换 【放松模式 ↔ 云台跟随模式】 *********************/
            // 模式切换指令：键盘CTRL键 或 遥控器右自定义键，两按键等效
            uint8_t toggle_cmd = (rc->key.v & KEY_CTRL) || rc->rc.custom_r;
            // 按键上升沿检测：仅按下瞬间触发切换，彻底防抖，长按/松开不响应
            uint8_t toggle_trigger = (toggle_cmd && !last_toggle_cmd);

            if (toggle_trigger) {
                if (robot_ctrl.chassis_mode != CHASSIS_RELAX) {
                    robot_ctrl.chassis_mode = CHASSIS_RELAX;  // 非放松→切换放松模式
                } else {
                    robot_ctrl.chassis_mode = CHASSIS_FOLLOW; // 放松→切换云台跟随模式
                }
            }
            // 更新按键状态，用于下一次防抖判断
            last_toggle_cmd = toggle_cmd;
        }

        /********************* 子逻辑2：遥控器在线时的底盘核心控制逻辑 *********************/
        if (robot_ctrl.monitor.remote_online) {
            // 底盘非放松模式，执行运动控制逻辑
            if (robot_ctrl.chassis_mode != CHASSIS_RELAX) {
                // 云台跟随模式【全向轮核心控制区】
                if (robot_ctrl.chassis_mode == CHASSIS_FOLLOW) {
                    // --- A. 输入源融合处理 (遥控器摇杆 + 键盘WASD) 双输入源，优先级等同 ---
                    float vx_rc = (abs(rc->rc.ch[0]) > RC_DEADZONE) ? rc->rc.ch[0] / 660.0f : 0;
                    float vy_rc = (abs(rc->rc.ch[1]) > RC_DEADZONE) ? rc->rc.ch[1] / 660.0f : 0;
                    float vw_rc = (abs(rc->rc.wheel) > RC_DEADZONE) ? rc->rc.wheel / 660.0f : 0;

                    float vx_kb = 0, vy_kb = 0, vw_kb = 0;
                    // Shift按键倍速逻辑：按下满速运行，松开半速，兼顾精准控制与高速移动
                    float speed_ratio = (rc->key.v & KEY_SHIFT) ? 1.0f : 0.5f;

                    // 键盘WASD映射全向轮运动方向，QE控制底盘手动旋转
                    if (rc->key.v & KEY_W) vy_kb += speed_ratio;
                    if (rc->key.v & KEY_S) vy_kb -= speed_ratio;
                    if (rc->key.v & KEY_A) vx_kb -= speed_ratio;
                    if (rc->key.v & KEY_D) vx_kb += speed_ratio;
                    if (rc->key.v & KEY_Q) vw_kb -= 0.5f; // 键盘Q - 底盘左旋
                    if (rc->key.v & KEY_E) vw_kb += 0.5f; // 键盘E - 底盘右旋

                    // 融合遥控器与键盘的最终横纵向速度指令
                    float total_vx = vx_rc + vx_kb;
                    float total_vy = vy_rc + vy_kb;

                    // --- B. 全向轮各向同性速度限幅 防止合成速度超限，底盘失控 ---
                    float v_norm = sqrtf(total_vx * total_vx + total_vy * total_vy);
                    if (v_norm > speed_ratio) {
                        total_vx = total_vx / v_norm * speed_ratio;
                        total_vy = total_vy / v_norm * speed_ratio;
                    }

                    // --- C. 云台跟随+手动旋转优先级逻辑 手动操作优先于自动跟随 ---
                    float yaw_m_pos;
                    yaw_m->get_status(yaw_m, "POS", &yaw_m_pos);
                    float angle_error = Rad_Format(yaw_m_pos - YAW_CENTER_OFFSET);

                    float vw_final = 0;
                    // 遥控器拨轮/键盘QE手动旋转 优先级更高，无手动操作时自动跟随云台
                    if (fabsf(vw_rc) > 0.05f || fabsf(vw_kb) > 0.01f) {
                        vw_final = vw_rc + vw_kb;
                    } else {
                        vw_final = -angle_error * FOLLOW_P_GAIN;
                    }
                    robot_ctrl.chassis.yaw_speed = vw_final * CHASSIS_MAX_RAD;

                    // --- D. 随动坐标系变换 WASD移动方向始终以云台朝向为基准 ---
                    float final_vx = total_vx * cosf(angle_error) - total_vy * sinf(angle_error);
                    float final_vy = total_vx * sinf(angle_error) + total_vy * cosf(angle_error);

                    // --- E. 全向轮逆运动学解算 根据底盘期望速度，计算4个电机目标转速 ---
                    wheel_targets[0] = (-final_vx - final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                    wheel_targets[1] = (-final_vx + final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                    wheel_targets[2] = (final_vx + final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                    wheel_targets[3] = (final_vx - final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;

                    // 下发目标转速至4个底盘M3508电机，执行闭环控制
                    for (int i = 0; i < 4; i++) {
                        if (chassis[i]) chassis[i]->set_target(chassis[i], 1, wheel_targets[i]);
                    }
                }
            }
        }

        osDelay(2); // 底盘任务周期2ms，保证控制实时性，不可随意增大
    }
}