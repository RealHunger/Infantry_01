#include "../ALL_Task/chassis_task.h"
#include "../Components/motor/motor.h"
#include "../Bsp/uart/bsp_uart.h"
#include "../Application/robot_global.h"
#include "math.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "../Bsp/LED/bsp_LED.h"
#include "../Application/auto_aim.h"
#include  "../ALL_Task/gimbal_task.h"

/* --- 逻辑常量与控制参数 --- */
#define GIMBAL_YAW_SENS         0.010f
#define GIMBAL_PIT_SENS         0.002f
#define MOUSE_YAW_SENS          0.0004f  // 鼠标横向灵敏度
#define MOUSE_PIT_SENS          0.0002f  // 鼠标纵向灵敏度
#define FOLLOW_P_GAIN           0.5f
#define RC_DEADZONE             10
#define YAW_CENTER_OFFSET       1.9f//-0.13f（步兵） //1.9f（哨兵）

// 底盘几何参数配置
#define MOTOR_RPM_TO_VECTOR     3000.0f
#define CHASSIS_MAX_RAD         60.0f

// 回正相关参数
#define YAW_ALIGN_THRESHOLD     0.05f    // 放宽到位阈值（适配机械误差，约2.86度）
#define WHEEL_ACTIVE_THRESHOLD  0.01f    // 拨轮有效输入阈值
#define QE_ACTIVE_THRESHOLD     0.01f     // Q/E有效输入阈值

/* --- 静态控制变量 --- */
static float world_yaw_target = 0.0f;
static float world_pit_target = 0.0f;
static float vx_ramp = 0.0f, vy_ramp = 0.0f;

static uint32_t last_rc_tick = 0;

// 扩展：新增Q/E相关状态变量，和拨轮统一管理
static uint8_t last_wheel_active = 0;    // 上一帧拨轮是否激活
static uint8_t last_qe_active = 0;       // 上一帧Q/E是否激活
static uint8_t yaw_align_enable = 0;     // 回正使能标志（1=需要回正，0=不需要）
static float last_manual_vw = 0.0f;      // 保存「拨轮/Q/E」松开前的最后有效旋转速度（统一变量，避免冲突）
// 新增：Q/E 按键切换状态与防抖记录（用于实现按键切换而非持续按住）
static uint8_t left_rotate_toggle = 0;   // Q键切换：左旋状态（1=左旋开启）
static uint8_t right_rotate_toggle = 0;  // E键切换：右旋状态（1=右旋开启）
static uint8_t last_q_pressed = 0;       // 上一帧 Q 键状态（防抖）
static uint8_t last_e_pressed = 0;       // 上一帧 E 键状态（防抖）


uint16_t cnt = 0;

static float Rad_Format(float angle) {
    while (angle >  (float)M_PI) angle -= 2.0f * (float)M_PI;
    while (angle < -(float)M_PI) angle += 2.0f * (float)M_PI;
    return angle;
}

void chassis_task_func(void const * argument) {
    /******************************************************************************************************************/
    /* 初始化 */
    struct uart_device* Uart = uart_get_device("uart1_dma");
    Uart->Init(Uart, 115200, 8, 'N', 1);

    struct motor_device *chassis[4];
    for(int i=0; i<4; i++) {
        char name[25]; sprintf(name, "M3508_CHASSIS_%d", i+1);
        chassis[i] = motor_get_device(name);
    }
    struct motor_device *yaw_m = motor_get_device("GM6020_YAW");

    const RC_ctrl_t *rc = robot_ctrl.rc;
    static uint8_t last_toggle_cmd = 0;
    float wheel_targets[4] = {0};

    /******************************************************************************************************************/
    // 系统启动保护
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);

    /******************************************************************************************************************/
    // 主循环
    while (1) {
        uint32_t current_tick = osKernelSysTick();

        //打印接受的数据
        static uint32_t last_gateway_print_tick = 0;
        if (current_tick - last_gateway_print_tick > 500) {
            struct uart_device *uart1 = uart_get_device("uart1_dma");
            if (uart1 != NULL) {
                uart1->Print(uart1,
                "====== MAIN BOARD CAN RX TEST ======\r\n"
                " [Test] CAN_Cnt: %d \r\n"
                " [RAW 101]: %02X %02X %02X %02X %02X %02X %02X %02X \r\n"
                " [RAW 102]: %02X %02X %02X %02X %02X %02X %02X %02X \r\n"
                "------------------------------------\r\n"
                "  > Game : Prog: %d | Time: %d s | Place: %d\r\n"
                "  > State: HP: %d | Heat: %d | Buf: %d J\r\n"
                "  > Shoot: Allow17: %d | ArmorID: %d | Hurt: %d\r\n"
                "====================================\r\n\r\n",
                cnt,
                can_raw_101[0], can_raw_101[1], can_raw_101[2], can_raw_101[3],
                can_raw_101[4], can_raw_101[5], can_raw_101[6], can_raw_101[7],
                can_raw_102[0], can_raw_102[1], can_raw_102[2], can_raw_102[3],
                can_raw_102[4], can_raw_102[5], can_raw_102[6], can_raw_102[7],
                // 解析后的黄金数据：
                robot_ctrl.gateway_referee_t.game_progress,
                robot_ctrl.gateway_referee_t.stage_remain_time,
                robot_ctrl.gateway_referee_t.place_status,
                robot_ctrl.gateway_referee_t.current_HP,
                robot_ctrl.gateway_referee_t.shooter_17mm_barrel_heat,
                robot_ctrl.gateway_referee_t.buffer_energy,
                robot_ctrl.gateway_referee_t.allow_bullet_17,
                robot_ctrl.gateway_referee_t.armor_id,
                robot_ctrl.gateway_referee_t.HP_deducation_reason
                );
            }
            last_gateway_print_tick = current_tick;
        }

        /**************************************************************************************************************/
        // 遥控器掉线检测
        if (current_tick - rc->vt13.last_update_tick > 200) {
            robot_ctrl.monitor.remote_online = 0;
            robot_ctrl.chassis_mode = CHASSIS_RELAX;

            // 掉线时重置所有标志和保存的速度
            yaw_align_enable = 0;
            last_wheel_active = 0;
            last_qe_active = 0;
            last_manual_vw = 0.0f;
            // 清除 Q/E 切换态与按键防抖，避免断线后滞留旋转状态
            left_rotate_toggle = 0;
            right_rotate_toggle = 0;
            last_q_pressed = 0;
            last_e_pressed = 0;
        } else {
            robot_ctrl.monitor.remote_online = 1;
            /**********************************************************************************************************/
            // 底盘模式切换
            uint8_t toggle_cmd = (KEY_PRESSED(rc->vt13.key_vt13.v, KEY_VT13_CTRL) || rc->vt13.rc_vt13.custom_r);
            uint8_t toggle_trigger = (toggle_cmd && !last_toggle_cmd);

            if (toggle_trigger) {
                if (robot_ctrl.chassis_mode != CHASSIS_RELAX) {
                    robot_ctrl.chassis_mode = CHASSIS_RELAX;
                    // 模式切换为放松时，重置所有标志
                    yaw_align_enable = 0;
                    last_wheel_active = 0;
                    last_qe_active = 0;
                    last_manual_vw = 0.0f;
                    // 切换到放松时也清除 Q/E 切换态与防抖
                    left_rotate_toggle = 0;
                    right_rotate_toggle = 0;
                    last_q_pressed = 0;
                    last_e_pressed = 0;
                } else {
                    robot_ctrl.chassis_mode = CHASSIS_FOLLOW;
                }
            }
            last_toggle_cmd = toggle_cmd;
        }

        /**************************************************************************************************************/

        if (robot_ctrl.monitor.remote_online) {
            if (robot_ctrl.chassis_mode != CHASSIS_RELAX) {
                if (robot_ctrl.chassis_mode == CHASSIS_FOLLOW) {
                    // --- A. 输入源融合 (遥控器摇杆 + 键盘) ---
                    float vx_rc = (abs(rc->vt13.rc_vt13.ch[0]) > RC_DEADZONE) ? ((float)rc->vt13.rc_vt13.ch[0] / 660.0f) : 0.0f;
                    float vy_rc = (abs(rc->vt13.rc_vt13.ch[1]) > RC_DEADZONE) ? ((float)rc->vt13.rc_vt13.ch[1] / 660.0f) : 0.0f;
                    float vw_rc = (abs(rc->vt13.rc_vt13.wheel) > RC_DEADZONE) ? ((float)rc->vt13.rc_vt13.wheel / 660.0f) : 0.0f;

                    float vx_kb = 0.0f, vy_kb = 0.0f, vw_kb = 0.0f;
                    float speed_ratio = KEY_PRESSED(rc->vt13.key_vt13.v, KEY_VT13_SHIFT) ? 0.7f : 0.5f;

                    if (KEY_PRESSED(rc->vt13.key_vt13.v, KEY_VT13_W)) vy_kb += speed_ratio;
                    if (KEY_PRESSED(rc->vt13.key_vt13.v, KEY_VT13_S)) vy_kb -= speed_ratio;
                    if (KEY_PRESSED(rc->vt13.key_vt13.v, KEY_VT13_A)) vx_kb -= speed_ratio;
                    if (KEY_PRESSED(rc->vt13.key_vt13.v, KEY_VT13_D)) vx_kb += speed_ratio;

                    // Q/E: 切换式按键（按一次切换左旋/右旋状态），使用上升沿检测实现防抖
                    uint8_t q_pressed = KEY_PRESSED(rc->vt13.key_vt13.v, KEY_VT13_Q);
                    uint8_t e_pressed = KEY_PRESSED(rc->vt13.key_vt13.v, KEY_VT13_E);
                    uint8_t q_trigger = (q_pressed && !last_q_pressed); // Q 上升沿
                    uint8_t e_trigger = (e_pressed && !last_e_pressed); // E 上升沿

                    if (q_trigger) {
                        left_rotate_toggle = !left_rotate_toggle;    // 切换左旋状态
                        if (left_rotate_toggle) right_rotate_toggle = 0; // 互斥，打开左则关闭右
                    }
                    if (e_trigger) {
                        right_rotate_toggle = !right_rotate_toggle;  // 切换右旋状态
                        if (right_rotate_toggle) left_rotate_toggle = 0; // 互斥，打开右则关闭左
                    }

                    // 根据切换状态设置 vw_kb 为固定手动速度（与 speed_ratio 同量级），或保持为 0
                    if (left_rotate_toggle) vw_kb = -speed_ratio;
                    else if (right_rotate_toggle) vw_kb = speed_ratio;

                    // 更新上一帧按键状态（防抖记录）
                    last_q_pressed = q_pressed;
                    last_e_pressed = e_pressed;

                    float total_vx = vx_rc + vx_kb;
                    float total_vy = vy_rc + vy_kb;


                    // --- B. 各向同性限速 ---
                    float v_norm = sqrtf(total_vx * total_vx + total_vy * total_vy);
                    if (v_norm > speed_ratio) {
                        total_vx = total_vx / v_norm * speed_ratio;
                        total_vy = total_vy / v_norm * speed_ratio;
                    }

                    // --- C. 跟随与旋转逻辑（扩展：Q/E+拨轮统一回正）---
                    float yaw_m_pos;
                    yaw_m->get_status(yaw_m, "POS", &yaw_m_pos);
                    float angle_error = Rad_Format(yaw_m_pos - YAW_CENTER_OFFSET);

                    // 步骤1：判断当前拨轮、Q/E是否处于激活状态
                    uint8_t current_wheel_active = (fabsf(vw_rc) > WHEEL_ACTIVE_THRESHOLD) ? 1 : 0;
                    uint8_t current_qe_active = (fabsf(vw_kb) > QE_ACTIVE_THRESHOLD) ? 1 : 0;
                    // 合并手动输入状态（拨轮或Q/E有一个激活，就认为是手动控制阶段）
                    uint8_t current_manual_active = current_wheel_active || current_qe_active;

                    // 步骤2：手动控制阶段，实时保存最后有效旋转速度（统一保存到last_manual_vw）
                    if (current_manual_active) {
                        last_manual_vw = vw_rc + vw_kb; // 保存当前拨轮+Q/E的合成速度（符合原有手动逻辑）
                    }

                    // 步骤3：检测「拨轮」或「Q/E」的松开下降沿，触发回正（二选一触发，避免冲突）
                    uint8_t wheel_release_trigger = (last_wheel_active && !current_wheel_active && !current_qe_active);
                    uint8_t qe_release_trigger = (last_qe_active && !current_qe_active && !current_wheel_active);
                    if ((wheel_release_trigger || qe_release_trigger) && !yaw_align_enable) {
                        yaw_align_enable = 1; // 开启回正使能
                    }

                    // 步骤4：优先级排序：手动控制 > 固定速度回正 > 正常跟随
                    float vw_final = 0;
                    if (current_manual_active) {
                        // 手动控制阶段：关闭回正使能，优先响应输入
                        yaw_align_enable = 0;
                        vw_final = vw_rc + vw_kb;
                    } else if (yaw_align_enable) {
                        // 自动回正阶段：直接使用松开前保存的最后手动速度，固定速度回正
                        vw_final = last_manual_vw;

                        // 回正到位判断，到位后清零所有标志和速度
                        if (fabsf(angle_error) < YAW_ALIGN_THRESHOLD) {
                            yaw_align_enable = 0;
                            vw_final = 0;
                            last_manual_vw = 0.0f;
                        }
                    } else {
                        // 正常跟随阶段：原有的云台跟随逻辑
                        vw_final = -angle_error * FOLLOW_P_GAIN;
                    }

                    // 步骤5：更新上一帧状态记录（供下一帧边缘检测使用）
                    last_wheel_active = current_wheel_active;
                    last_qe_active = current_qe_active;

                    robot_ctrl.chassis.yaw_speed = vw_final * CHASSIS_MAX_RAD;

                    // --- D. 随动坐标系变换 ---
                    float final_vx = total_vx * cosf(angle_error) - total_vy * sinf(angle_error);
                    float final_vy = total_vx * sinf(angle_error) + total_vy * cosf(angle_error);

                    // --- E. 逆运动学计算 ---
                    wheel_targets[0] = (final_vx + final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                    wheel_targets[1] = (final_vx - final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                    wheel_targets[2] = (-final_vx - final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                    wheel_targets[3] = (-final_vx + final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;

                    for (int i = 0; i < 4; i++) {
                        if (chassis[i]) chassis[i]->set_target(chassis[i], 1, wheel_targets[i]);
                    }
                }
            }
        }
        else {
            // 遥控器掉线：红灯快闪
            osDelay(100);
        }

        osDelay(2);
    }
}

