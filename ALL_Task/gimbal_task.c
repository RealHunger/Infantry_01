#include "gimbal_task.h"          // 云台任务头文件-本文件声明
#include "../Application/robot_global.h"  // 全局变量头文件-核心全局结构体、枚举定义
#include "../../Components/motor/motor.h" // 电机驱动头文件-电机设备句柄/接口函数
#include "math.h"                 // 数学库头文件-三角函数/绝对值/浮点运算
#include "stdlib.h"               // 标准库头文件-通用工具函数
#include "cmsis_os.h"             // RTOS系统头文件-系统滴答/延时/任务调度
#include "stdio.h"                // 标准输入输出-调试打印备用
#include "../../Bsp/uart/bsp_uart.h" // 串口驱动头文件-上位机/外设通信
#include "../../Bsp/led/bsp_led.h"   // LED驱动头文件-状态指示灯控制【保留灯光 不删除】
#include "../../Components/remote/remote.h" // 遥控器驱动头文件-遥控数据解析
#include "../../Application/auto_aim.h"     // 自瞄功能头文件-上位机数据解析

/***********************************************************************************************************************
* 宏定义-集中管理 【仅保留实际调用的有效宏定义，分区归类+详细注释，无冗余】
***********************************************************************************************************************/
// ===================== 云台控制-摇杆/鼠标 灵敏度&死区参数【云台核心必用】 =====================
#define RC_DEADZONE         10          // 遥控器摇杆死区：防止摇杆漂移产生无效信号
#define MOUSE_YAW_SENS      0.00005f    // 鼠标X轴-云台航向角 控制灵敏度
#define MOUSE_PIT_SENS      0.00005f    // 鼠标Y轴-云台俯仰角 控制灵敏度
#define RC_YAW_SENS         0.005f      // 遥控器摇杆-云台航向角 控制灵敏度
#define RC_PIT_SENS         0.005f      // 遥控器摇杆-云台俯仰角 控制灵敏度


// ===================== 云台核心限位 【物理机械硬限位，重中之重，严禁修改】 =====================
#define PITCH_UP_LIMIT      0.35f       // 云台俯仰角 向上最大限位 (弧度制) 防止云台撞上枪管/云台架
#define PITCH_DOWN_LIMIT    -0.45f      // 云台俯仰角 向下最大限位 (弧度制) 防止云台撞上底盘/发射机构

// ===================== 发射机构-拨弹轮堵转逃逸 核心参数【发射必用，完整保留】 =====================
#define STIR_REVERSE_CURRENT   8500     // 拨弹轮堵转判定电流阈值(mA) 超过该值判定为卡弹
#define STIR_BLOCK_TIME        150      // 堵转持续判定时间(ms) 防抖，防止瞬时大电流误判
#define STIR_REVERSE_TIME      200      // 堵转后反转逃逸时间(ms) 反转退弹，解除卡弹状态
#define SHOOT_FW_SPEED         6000.0f  // 摩擦轮目标转速 搓弹加速，提供子弹初速度
#define STIR_SHOOT_SPEED       10000.0f  // 拨弹轮正常发射转速 正向拨弹，送弹入膛
#define STIR_REVERSE_SPEED     2500.0f  // 拨弹轮堵转反转转速 低速反转，防止退弹过猛二次卡弹

// ===================== 自瞄开火新增宏定义 =====================
#define AUTO_SHOOT_TRIGGER_CNT 2        // 自瞄开火触发阈值：连续读到shoot==1的次数



/***********************************************************************************************************************
* 函数名：Rad_Format
* 功  能：角度归一化处理，将任意弧度制角度限制在 [-π, π] 区间内
* 参  数：angle 待归一化的原始弧度角度
* 返回值：归一化后的合规弧度角度
* 说  明：解决云台360°旋转时角度值跳变问题，保证闭环控制的连续性，无突变抖动
***********************************************************************************************************************/
static float Rad_Format(float angle) {
    while (angle >  M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

/***********************************************************************************************************************
* 函数名：gimbal_task_func
* 功  能：云台任务主函数 - 优先级最高的控制任务之一
* 职  责：1.云台模式切换(失能/手动/自瞄) 2.云台角度闭环控制 3.发射机构控制(摩擦轮+拨弹轮)
*         4.拨弹轮堵转逃逸保护 5.遥控器掉线急停保护 6.各状态指示灯反馈 7.自瞄数据解析与使用
* 参  数：argument RTOS任务形参，无实际使用
* 调度周期：2ms 高频率保证控制精度，云台控制核心要求
* 适配说明：已全部修改为【VT13遥控器专属】输入，无任何DT7相关代码
***********************************************************************************************************************/
void gimbal_task_func(void const * argument) {
    /**************************************** 【硬件外设初始化区】 ****************************************/
    // 获取串口1 DMA句柄并初始化：波特率115200、8位数据位、无校验、1位停止位
    struct uart_device* Uart = uart_get_device("uart1_dma");
    Uart->Init(Uart, 115200, 8, 'N', 1);

    // 获取所有电机设备句柄 - 绑定对应电机，通过句柄调用电机驱动接口
    const struct motor_device *yaw_m = motor_get_device("GM6020_YAW");    // 云台航向轴电机 GM6020
    const struct motor_device *pit_m = motor_get_device("J4310_PITCH");   // 云台俯仰轴电机 J4310
    struct motor_device *shoot_l = motor_get_device("M3508_SHOOT_L");     // 左摩擦轮电机 M3508
    struct motor_device *shoot_r = motor_get_device("M3508_SHOOT_R");     // 右摩擦轮电机 M3508
    struct motor_device *stir_m  = motor_get_device("M2006_TRIGGER");     // 拨弹轮电机 M2006

    // 自瞄功能初始化：获取USB句柄+初始化USB，用于接收上位机(视觉)数据
    struct usb_device* usb = usb_get_device();
    usb->Init(usb);
    auto_aim_init(usb);

    /**************************************** 【静态状态变量区 - 防抖/状态机/计时专用，无冗余】 ****************************************/
    static uint8_t last_relax_toggle = 0;    // 云台失能模式按键 上一帧状态 - 按键防抖，防止误触
    static uint8_t last_mode_toggle = 0;     // 云台模式切换按键 上一帧状态 - 按键防抖，防止误触
    static uint8_t last_shoot_toggle = 0;       // 发射命令 上一帧状态 - 用于自瞄连续开火计数
    static uint8_t is_initialized = 0;       // 云台初始化标志位 0-未初始化 1-已初始化 防止上电瞬间角度突变甩动
    // 拨弹轮状态机枚举：正常发射/堵转判定中/反转逃逸中 三段式状态机，卡弹处理核心逻辑
    static enum { STIR_NORMAL, STIR_BLOCKING, STIR_REVERSING } stir_state = STIR_NORMAL;
    static uint32_t block_start_tick = 0;    // 堵转开始时刻系统滴答值 - 用于累计堵转时间
    static uint32_t reverse_end_tick = 0;    // 反转结束时刻系统滴答值 - 用于控制反转时长
    // ===================== 新增：自瞄连续开火计数 =====================
    static uint8_t auto_shoot_count = 0;     // 自瞄模式下连续读到shoot==1的次数

    float world_yaw_target = 0.0f;           // 云台世界坐标系 航向角目标值 (弧度)
    float world_pit_target = 0.0f;           // 云台世界坐标系 俯仰角目标值 (弧度)

    /**************************************** 【系统上电启动保护】 ****************************************/
    // 等待传感器就绪：陀螺仪/加速度计等传感器未就绪前，云台不动作，防止失控
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);  // 上电延时1s，等待所有外设/电机/传感器稳定，硬件防冲击

    /**************************************** 【云台任务主循环 - 死循环永不退出】 ****************************************/
    while (1) {
        uint32_t current_tick = osKernelSysTick();  // 获取当前系统滴答定时器值(ms)，用于所有计时逻辑

        /**************************************** 【最高优先级】VT13遥控器掉线全局急停保护 ****************************************/
        // 遥控器超时判定：超过200ms未收到VT13遥控器数据，判定为遥控器掉线/失联
        if (current_tick - robot_ctrl.rc->vt13.last_update_tick > 200) {
            robot_ctrl.monitor.remote_online = 0;        // 置位遥控器离线标志位
            robot_ctrl.gimbal_mode = GIMBAL_RELAX;       // 云台强制进入失能模式，无动力
            robot_ctrl.shoot_mode = SHOOT_STOP;          // 发射机构强制停止，所有发射电机归零
            is_initialized = 0;                          // 云台初始化标志位清零，重连后重新初始化
            auto_shoot_count = 0;                        // 新增：掉线时清零自瞄开火计数

            // 掉线急停核心动作：所有发射电机零速指令，防止失控发射
            shoot_l->set_target(shoot_l, 1, 0);
            shoot_r->set_target(shoot_r, 1, 0);
            stir_m->set_target(stir_m, 1, 0);

            // 指示灯反馈：遥控器掉线 → 红灯闪烁 (全局最高优先级，其他状态被覆盖)
            LED_GREEN_RESET(); LED_BLUE_RESET(); LED_RED_Toggle();
            osDelay(100);
        }
        // ===================== VT13遥控器在线 正常工作逻辑 =====================
        else {
            robot_ctrl.monitor.remote_online = 1;  // 置位遥控器在线标志位

            /********************* 发射模式切换：F按键/遥控器档位 双路切换 *********************/
            // VT13 F键按下且防抖：发射就绪 ↔ 发射停止 切换
            /* KEY_VT13_F is 0x0200 (uint16_t). If we assign the raw bitmask directly to a uint8_t
               it will be truncated to 0. Convert to a 0/1 boolean explicitly to avoid this bug. */
            uint8_t shoot_ready_cmd = KEY_PRESSED(robot_ctrl.rc->vt13.key_vt13.v, KEY_VT13_F);
            uint8_t shoot_trigger = (shoot_ready_cmd && !last_shoot_toggle);     // 按键上升沿触发，防抖
            if (shoot_trigger) {
                robot_ctrl.shoot_mode = (robot_ctrl.shoot_mode == SHOOT_STOP) ? SHOOT_READY : SHOOT_STOP;
            }
            last_shoot_toggle = shoot_ready_cmd; // 更新发射按键上一帧状态，用于防抖

            // // VT13遥控器档位切换：S档(发射档) ↔ 其他档 切换，优先级与F键一致
            // if (robot_ctrl.rc->vt13.rc_vt13.sw != last_sw_state) {
            //     robot_ctrl.shoot_mode = (robot_ctrl.rc->vt13.rc_vt13.sw == RC_SW_S_VT13) ? SHOOT_READY : SHOOT_STOP;
            //     last_sw_state = robot_ctrl.rc->vt13.rc_vt13.sw;     // 更新档位上一帧状态，用于防抖
            // }
            /********************* 云台工作模式切换：失能 ↔ 手动 ↔ 自瞄 *********************/
            // 云台失能模式触发条件：VT13遥控器暂停键 或 VT13 C键 按下
            uint8_t relax_cmd = (robot_ctrl.rc->vt13.rc_vt13.pause) || KEY_PRESSED(robot_ctrl.rc->vt13.key_vt13.v, KEY_VT13_C);
            uint8_t relax_trigger = (relax_cmd && !last_relax_toggle); // 按键上升沿触发，防抖
            // 云台模式切换条件：VT13遥控器自定义左按键 或 鼠标右键 按下 (原先为 VT13 G 键)
            uint8_t mode_cmd = (robot_ctrl.rc->vt13.rc_vt13.custom_l) || (robot_ctrl.rc->vt13.mouse_vt13.press_r);
            uint8_t mode_trigger = (mode_cmd && !last_mode_toggle);     // 按键上升沿触发，防抖

            // 触发放松切换：失能 ↔ 手动 互切，同时清零初始化标志位，重连后防甩动
            if (relax_trigger) {
                robot_ctrl.gimbal_mode = (robot_ctrl.gimbal_mode == GIMBAL_RELAX) ? GIMBAL_REMOTE : GIMBAL_RELAX;
                is_initialized = 0;
                auto_shoot_count = 0;                            // 新增：切换模式时清零自瞄开火计数
            }
            // 触发模式切换：手动 ↔ 自瞄 互切，仅在云台使能状态下有效
            if (mode_trigger && robot_ctrl.gimbal_mode != GIMBAL_RELAX) {
                robot_ctrl.gimbal_mode = (robot_ctrl.gimbal_mode == GIMBAL_REMOTE) ? GIMBAL_AUTO : GIMBAL_REMOTE;
                auto_shoot_count = 0;                            // 新增：切换模式时清零自瞄开火计数
            }

            // 更新按键上一帧状态，完成防抖逻辑
            last_relax_toggle = relax_cmd;
            last_mode_toggle = mode_cmd;

            /**************************************** 云台角度闭环控制核心逻辑 ****************************************/
            if (robot_ctrl.gimbal_mode != GIMBAL_RELAX) {  // 云台非失能模式 → 使能，进入角度闭环控制
                // 云台首次使能初始化：将目标角度同步为当前实际角度，防止上电瞬间角度突变导致云台甩动
                if (is_initialized == 0) {
                    world_yaw_target = robot_ctrl.gimbal.yaw;
                    world_pit_target = robot_ctrl.gimbal.pitch;
                    is_initialized = 1;  // 置位初始化完成标志位，仅执行一次
                }

                /********************* 模式1：云台手动控制【VT13遥控器摇杆+鼠标 复合控制】 *********************/
                if (robot_ctrl.gimbal_mode == GIMBAL_REMOTE) {
                    // 指示灯反馈：云台手动使能 → 绿灯常亮
                    LED_RED_RESET(); LED_BLUE_RESET(); LED_GREEN_SET();

                    // VT13遥控器摇杆值处理：死区过滤 + 归一化到[-1,1]区间，消除无效信号
                    float ry = (abs(robot_ctrl.rc->vt13.rc_vt13.ch[2]) > RC_DEADZONE) ? robot_ctrl.rc->vt13.rc_vt13.ch[2] / 660.0f : 0.0f;
                    float rx = (abs(robot_ctrl.rc->vt13.rc_vt13.ch[3]) > RC_DEADZONE) ? robot_ctrl.rc->vt13.rc_vt13.ch[3] / 660.0f : 0.0f;
                    // VT13鼠标值处理：直接乘以灵敏度系数，转为角度增量
                    float mouse_x = (float)robot_ctrl.rc->vt13.mouse_vt13.x * MOUSE_YAW_SENS;
                    float mouse_y = (float)robot_ctrl.rc->vt13.mouse_vt13.y * MOUSE_PIT_SENS;

                    // 计算云台目标角度：摇杆控制量 + 鼠标控制量 叠加
                    world_pit_target -= (ry * RC_PIT_SENS) + mouse_y;
                    world_yaw_target -= (rx * RC_YAW_SENS) + mouse_x;


                    // 俯仰角目标值软件限位 【第一道防护】严格限制在机械限位内
                    if(world_pit_target > PITCH_UP_LIMIT)  world_pit_target = PITCH_UP_LIMIT;
                    if(world_pit_target < PITCH_DOWN_LIMIT)world_pit_target = PITCH_DOWN_LIMIT;
                }

                /********************* 模式2：云台自瞄控制【核心优化】解析全局自瞄数据，视觉闭环 *********************/
                else if (robot_ctrl.gimbal_mode == GIMBAL_AUTO) {
                    // 解析上位机视觉数据到全局结构体 robot_ctrl.target_info，返回1=有目标，0=丢目标
                    if (parse_target_data(&robot_ctrl.target_info) == 1) {
                        // 指示灯反馈：自瞄模式+有目标 → 蓝灯常亮
                        LED_RED_RESET(); LED_GREEN_RESET(); LED_BLUE_SET();
                        // 直接赋值视觉解算后的目标角度，云台跟随目标
                        world_yaw_target = robot_ctrl.target_info.aim_target_yaw;
                        world_pit_target = robot_ctrl.target_info.aim_target_pitch;
                        // 自瞄模式同样做俯仰角限位，防止视觉数据异常超限
                        if(world_pit_target > PITCH_UP_LIMIT)  world_pit_target = PITCH_UP_LIMIT;
                        if(world_pit_target < PITCH_DOWN_LIMIT)world_pit_target = PITCH_DOWN_LIMIT;

                        // ===================== 新增：自瞄连续shoot计数逻辑 =====================
                        if (robot_ctrl.target_info.shoot == 1) {
                            // 连续读到shoot==1，计数加1（最多到阈值，防止溢出）
                            if (auto_shoot_count < AUTO_SHOOT_TRIGGER_CNT) {
                                auto_shoot_count++;
                            }
                        } else {
                            // shoot==0，直接清零计数
                            auto_shoot_count = 0;
                        }
                    } else {
                        // 指示灯反馈：自瞄模式+丢目标 → 蓝灯闪烁
                        LED_RED_RESET(); LED_BLUE_Toggle(); LED_GREEN_RESET();
                        robot_ctrl.target_info.shoot = 0;  // 丢目标强制停止发射，防止盲射
                        auto_shoot_count = 0;                // 新增：丢目标清零计数
                    }
                }

                /********************* 云台角度闭环输出 + 双重限位保护 【最终防护】 *********************/
                float cur_yaw, cur_pit;
                yaw_m->get_status(yaw_m, "POS", &cur_yaw);  // 获取航向轴电机 当前实际角度
                pit_m->get_status(pit_m, "POS", &cur_pit);  // 获取俯仰轴电机 当前实际角度

                // 云台闭环控制算法：航向角带底盘速度前馈补偿，俯仰角直接位置闭环，保证跟随精度
                float yaw_out = cur_yaw + Rad_Format(world_yaw_target - robot_ctrl.gimbal.yaw);
                float pit_out = cur_pit - (world_pit_target - robot_ctrl.gimbal.pitch);

                // 俯仰角输出值二次限位 【第二道防护，终极防护】防止任何情况超限
                if (pit_out > PITCH_UP_LIMIT) pit_out = PITCH_UP_LIMIT;
                if (pit_out < PITCH_DOWN_LIMIT) pit_out = PITCH_DOWN_LIMIT;

                // 下发目标角度到电机闭环控制器，电机执行跟随
                yaw_m->set_target(yaw_m, 2, yaw_out, robot_ctrl.chassis.yaw_speed); // 航向角带底盘速度前馈
                //yaw_m->set_target(yaw_m, 2, yaw_out, 36.0f);

                pit_m->set_target(pit_m, 1, pit_out);
            }
            /********************* 模式3：云台失能模式 *********************/
            else if (robot_ctrl.gimbal_mode == GIMBAL_RELAX) {
                // 指示灯反馈：云台失能 → 红灯常亮
                LED_GREEN_RESET(); LED_BLUE_RESET(); LED_RED_SET();
                auto_shoot_count = 0;                        // 新增：失能模式清零自瞄开火计数
            }

            //调试用：
            //int16_t yaw_speed;
            //yaw_m->get_status(yaw_m, "VEL", &yaw_speed);
            //Uart->Print(Uart, "%d,%f\r\n", yaw_speed, robot_ctrl.chassis.yaw_speed); // 调试打印航向角目标值，单位：mrad
            //Uart->Print(Uart, "%f,%f\r\n", robot_ctrl.gimbal.yaw, world_yaw_target); // 调试打印航向角目标值，单位：mrad

            /**************************************** 发射机构完整控制逻辑 ****************************************/
            /********************* 1.摩擦轮转速控制 - 左右轮反向旋转，搓弹加速 *********************/
            if (robot_ctrl.shoot_mode == SHOOT_READY) {  // 发射就绪状态
                // 左右摩擦轮反向定速转动，搓弹加速
                shoot_l->set_target(shoot_l, 1,  SHOOT_FW_SPEED);
                shoot_r->set_target(shoot_r, 1, -SHOOT_FW_SPEED);
            } else {  // 发射停止状态，摩擦轮零速，停止搓弹
                shoot_l->set_target(shoot_l, 1, 0);
                shoot_r->set_target(shoot_r, 1, 0);
            }

            /********************* 2.拨弹轮控制 + 堵转逃逸保护 + 分模式精准开火逻辑【核心完整版】 *********************/
            float stir_torque = 0;
            stir_m->get_status(stir_m, "CURRENT", &stir_torque);  // 获取拨弹轮电机当前电流，用于堵转检测
            uint8_t shoot_cmd = 0;                                // 最终开火指令 0-不开火 1-开火
            // VT13拨弹轮手动反转指令：鼠标中键/遥控器C档，用于手动退弹/解除卡弹
            uint8_t reverse_cmd = (robot_ctrl.rc->vt13.mouse_vt13.press_m || robot_ctrl.rc->vt13.rc_vt13.sw == RC_SW_C_VT13);

            // ========== 核心开火判定逻辑【分模式精准控制，防误射】 ==========
            if(robot_ctrl.gimbal_mode == GIMBAL_REMOTE)
            {
                // 手动模式开火条件：VT13鼠标左键/遥控器扳机 按下 + 发射就绪 → 无额外限制，直接开火
                shoot_cmd = (robot_ctrl.rc->vt13.mouse_vt13.press_l || robot_ctrl.rc->vt13.rc_vt13.trigger) && (robot_ctrl.shoot_mode == SHOOT_READY);
            }
            else if(robot_ctrl.gimbal_mode == GIMBAL_AUTO)
            {
                // ===================== 修改：自瞄模式开火条件 =====================
                // 自瞄模式开火条件：遥控器触发 + 发射就绪 + 连续3次shoot==1
                shoot_cmd = (robot_ctrl.rc->vt13.mouse_vt13.press_l || robot_ctrl.rc->vt13.rc_vt13.trigger)
                          && (robot_ctrl.shoot_mode == SHOOT_READY)
                          && (auto_shoot_count >= AUTO_SHOOT_TRIGGER_CNT);
            }

            // ========== 拨弹轮三段式状态机：正常发射 → 堵转判定 → 反转逃逸 【完整保留】 ==========
            if (reverse_cmd) {  // 手动反转指令优先：强制反转退弹
                stir_m->set_target(stir_m, 1, STIR_REVERSE_SPEED);
                stir_state = STIR_NORMAL;  // 强制恢复正常状态
                auto_shoot_count = 0;        // 新增：手动反转清零自瞄开火计数
            }
            else if (shoot_cmd) {  // 满足开火条件，进入发射/堵转处理逻辑
                if (stir_state == STIR_REVERSING) {  // 处于反转逃逸中
                    stir_m->set_target(stir_m, 1, 2500);
                    if (current_tick > reverse_end_tick) stir_state = STIR_NORMAL;
                }
                else {  // 处于正常发射状态
                    stir_m->set_target(stir_m, 1, -STIR_SHOOT_SPEED); // 反向拨弹送弹入膛

                    // 堵转检测逻辑：电流超过阈值 且 电流值有效，防止误判
                    if (fabsf(stir_torque) > STIR_REVERSE_CURRENT && fabsf(stir_torque) < 30000) {
                        if (stir_state == STIR_NORMAL) {  // 首次检测到堵转，开始计时
                            stir_state = STIR_BLOCKING;
                            block_start_tick = current_tick;
                        }
                        else if (current_tick - block_start_tick > STIR_BLOCK_TIME) {  // 堵转持续达标，触发反转
                            stir_state = STIR_REVERSING;
                            reverse_end_tick = current_tick + STIR_REVERSE_TIME;
                        }
                    }
                    else {  // 电流正常，恢复正常发射状态
                        stir_state = STIR_NORMAL;
                    }
                }
            }
            else {  // 无开火指令，拨弹轮零速停止
                stir_m->set_target(stir_m, 1, 0);
                stir_state = STIR_NORMAL;
            }
        }
        osDelay(2);  // 云台任务调度周期 2ms，固定频率保证控制精度
    }
}

