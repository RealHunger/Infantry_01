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
* 文 件 说 明：云台任务主函数 - 集成云台姿态控制/自瞄控制/发射机构(摩擦轮+拨弹盘)控制/遥控器指令解析/异常保护
* 硬件适配：17mm弹丸 + 100mm拨弹盘 + M2006拨弹电机(1:36减速+26/72同步带) + M3508摩擦轮电机 + GM6020/J4310云台电机
* 核心功能：1.云台手动/自瞄/失能三模式切换 2.俯仰角物理限位保护 3.摩擦轮启停控制 4.拨弹盘单颗弹精准送弹/退弹
*          5.遥控器掉线全局急停 6.指示灯状态反馈 7.串口调试数据打印
* 注        意：所有角度控制均为【弧度制】，与M2006底层弧度闭环PID完全匹配，无累计误差/无乱转/无卡弹
***********************************************************************************************************************/
/***********************************************************************************************************************
* 宏定义-集中管理 【所有可调参数都在这里，无需修改代码逻辑，直接改宏定义即可适配，优先级最高】
***********************************************************************************************************************/
// ===================== 云台摇杆/鼠标控制参数 =====================
#define RC_DEADZONE         10          // 遥控器摇杆死区：小于该值视为无效操作，防止摇杆漂移误触发
#define MOUSE_YAW_SENS      0.000045f   // 鼠标横移灵敏度(航向)：值越大鼠标移动云台越快，反之越慢
#define MOUSE_PIT_SENS      0.000045f   // 鼠标纵移灵敏度(俯仰)：值越大鼠标移动云台越快，反之越慢
#define RC_YAW_SENS         0.0045f     // 遥控器摇杆航向灵敏度：值越大摇杆移动云台越快，反之越慢
#define RC_PIT_SENS         0.0045f     // 遥控器摇杆俯仰灵敏度：值越大摇杆移动云台越快，反之越慢

// ===================== 云台物理限位【核心保护，防止机械卡死】=====================
#define PITCH_UP_LIMIT      0.35f       // 云台俯仰向上限位(弧度)，超过该角度强制限位，防止顶死机械结构
#define PITCH_DOWN_LIMIT    -0.45f      // 云台俯仰向下限位(弧度)，超过该角度强制限位，防止顶死机械结构

// ===================== 发射机构-摩擦轮参数 =====================
#define SHOOT_FW_SPEED      5500.0f     // 摩擦轮目标转速(rpm)，射速核心参数，根据弹丸适配调整

// ===================== 发射机构-拨弹盘精准送弹核心参数【17mm弹丸+100mm拨弹盘专属】=====================
#define PI                      3.1415926535f       // 圆周率常量
#define BULLET_DIAMETER         17.0f               // 弹丸直径：17mm，物理尺寸，不可随意修改
#define STIR_DISK_DIAMETER      100.0f              // 拨弹盘直径：100mm，物理尺寸，不可随意修改
#define STIR_DISK_RADIUS        (STIR_DISK_DIAMETER / 2.0f)  // 拨弹盘半径：50mm，自动计算，无需修改
#define BULLET_GAP              0.02f               // 机械间隙补偿值(弧度)：解决加工/装配间隙导致的送弹不到位问题
                                                     // 卡弹→减小(0.00~0.01)，送弹不到位→增大(0.03~0.04)，微调步长±0.01
#define SINGLE_BULLET_RAD       (BULLET_DIAMETER / STIR_DISK_RADIUS + BULLET_GAP)  // 单颗弹精准旋转弧度≈0.36rad(≈20.63°)
#define SHOOT_BULLET_NUM        1.0f                // 单次开火送弹数量：1颗，想连发N颗直接改数字(如2=连发2颗)
#define STIR_SHOOT_RAD          (SINGLE_BULLET_RAD * SHOOT_BULLET_NUM)  // 单次开火总旋转弧度，自动计算
#define STIR_REVERSE_RAD        (SINGLE_BULLET_RAD * 1.5f)  // 反转退弹弧度：单颗弹的1.5倍，退弹更顺畅不卡弹
#define STIR_POS_OFFSET         0.0f                // 拨弹盘基准偏移量，保持0即可，无需修改

// ===================== 电机安全限幅【防止电机堵转/过流烧毁】=====================
#define M3508_MAX_SPEED         10000.0f    // M3508摩擦轮电机最大转速限制
#define M2006_MAX_SPEED         6000.0f     // M2006拨弹电机最大转速限制

// ===================== 指示灯闪烁频率控制【本次核心优化】=====================
#define LED_BLINK_INTERVAL      200         // 指示灯闪烁间隔(ms)，值越大闪烁越慢，默认200ms最佳视觉效果
                                            // 推荐值：150-300ms，可根据需求调整

/***********************************************************************************************************************
* 函数名：Rad_Format
* 功 能：角度归一化处理，将任意弧度值限制在 [-π, π] 区间内
* 参 数：angle - 需要归一化的原始弧度值
* 返 回：归一化后的弧度值，范围[-3.1415, 3.1415]
* 说 明：与M2006底层归一化逻辑完全一致，实现拨弹盘旋转整圈后自动清零，无累计圈数误差，完美适配拨弹盘循环送弹逻辑
***********************************************************************************************************************/
static float Rad_Format(float angle) {
    angle = fmodf(angle, 2.0f * PI);    // 取模运算，将角度限制在[0,2π]或[-2π,0]
    if (angle > PI)  angle -= 2.0f * PI;// 大于π的角度，转换为负角度，保证区间[-π,π]
    if (angle < -PI) angle += 2.0f * PI;// 小于-π的角度，转换为正角度，保证区间[-π,π]
    return angle;
}

/***********************************************************************************************************************
* 函数名：gimbal_task_func
* 功 能：云台任务主函数，系统核心任务之一，优先级高
* 参 数：argument - 任务传参，无实际使用
* 返 回：无
* 任 务 周 期：2ms (osDelay(2))，保证控制实时性，兼顾CPU利用率
* 核心逻辑顺序：初始化→系统启动保护→主循环→遥控器掉线检测→指令解析→模式切换→云台控制→发射机构控制→串口调试
***********************************************************************************************************************/
void gimbal_task_func(void const * argument) {
    /**************************************** 第一步：外设&电机句柄初始化 ****************************************/
    // 初始化串口1 DMA，用于调试数据打印，波特率115200
    struct uart_device* Uart = uart_get_device("uart1_dma");
    Uart->Init(Uart, 115200, 8, 'N', 1);

    // 获取所有电机设备句柄，与电机注册的名称一一对应，不可写错
    struct motor_device *yaw_m = motor_get_device("GM6020_YAW");        // 航向云台电机
    struct motor_device *pit_m = motor_get_device("J4310_PITCH");       // 俯仰云台电机
    struct motor_device *shoot_l = motor_get_device("M3508_SHOOT_L");   // 左摩擦轮电机
    struct motor_device *shoot_r = motor_get_device("M3508_SHOOT_R");   // 右摩擦轮电机
    struct motor_device *stir_m  = motor_get_device("M2006_TRIGGER");   // M2006拨弹电机

    // 初始化USB和自瞄模块，用于接收上位机自瞄数据
    struct usb_device* usb = usb_get_device();
    usb->Init(usb);
    auto_aim_init(usb);

    /**************************************** 第二步：静态状态变量定义 ****************************************/
    // 静态变量：上电初始化一次，值会一直保留，用于防抖/状态记忆/累计计数，核心防抖逻辑依赖
    static uint8_t last_relax_toggle = 0;        // 云台失能模式切换防抖标志位
    static uint8_t last_mode_toggle = 0;         // 云台手动/自瞄模式切换防抖标志位
    static uint8_t is_initialized = 0;           // 云台角度初始化完成标志位，防止上电瞬间云台突变
    static uint8_t last_f_key = 0;               // F键(摩擦轮启停)防抖标志位
    static float stir_base_rad = 0.0f;           // M2006拨弹电机弧度基准值，适配底层弧度闭环，上电归零使用
    static uint8_t last_l_press = 0;             // 鼠标左键(开火拨弹)防抖标志位，上升沿单次触发，防止连发射弹
    static uint8_t last_m_press = 0;             // 鼠标中键(反转退弹)防抖标志位，上升沿单次触发，防止连续退弹
    static uint32_t led_tick = 0;                // 指示灯闪烁计时变量，控制闪烁频率核心变量【新增】

    // 云台世界坐标系目标角度，手动/自瞄模式下更新该值实现云台控制
    float world_yaw_target = 0.0f;
    float world_pit_target = 0.0f;

    /**************************************** 第三步：系统启动安全保护 ****************************************/
    // 等待传感器就绪，防止传感器未初始化完成就执行控制逻辑，导致数据异常
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);   // 延时1s，等待电机/外设完全上电稳定，硬件启动保护

    /**************************************** 第四步：任务主循环【死循环，永不退出】 ****************************************/
    while (1) {
        uint32_t current_tick = osKernelSysTick();  // 获取当前系统滴答定时器值，用于超时检测/计时

        /**************************************** 【最高优先级】遥控器掉线全局急停保护 ****************************************/
        // 判定条件：遥控器超过200ms未更新数据，视为掉线
        if (current_tick - robot_ctrl.rc->last_update_tick > 200) {
            robot_ctrl.monitor.remote_online = 0;       // 标记遥控器离线
            robot_ctrl.gimbal_mode = GIMBAL_RELAX;      // 云台强制进入失能模式，解锁电机
            robot_ctrl.shoot_mode = SHOOT_STOP;         // 发射机构强制停止，摩擦轮停转
            is_initialized = 0;                         // 云台角度初始化标志位清零，重新上电后初始化

            // 所有电机归零，安全急停
            shoot_l->set_target(shoot_l, 1, 0);
            shoot_r->set_target(shoot_r, 1, 0);
            stir_m->set_target(stir_m, 1, stir_base_rad);

            // 掉线指示灯状态：红灯慢闪(200ms)，绿/蓝灯灭 【优化闪烁频率】
            if(current_tick - led_tick > LED_BLINK_INTERVAL)
            {
                LED_RED_Toggle();
                led_tick = current_tick;
            }
            LED_GREEN_RESET();
            LED_BLUE_RESET();
            osDelay(100); // 掉线后适当增大延时，降低CPU占用
        }
        /**************************************** 遥控器在线：正常执行所有控制逻辑 ****************************************/
        else {
            robot_ctrl.monitor.remote_online = 1;   // 标记遥控器在线
            led_tick = current_tick;                // 重置指示灯计时，在线时关闭闪烁计时

            /********************* 子逻辑1：F键 摩擦轮 就绪/停止 一键切换 *********************/
            if ((robot_ctrl.rc->key.v & KEY_F) && !last_f_key) {
                // 上升沿触发：F键按下瞬间切换状态，防抖处理，防止长按重复切换
                robot_ctrl.shoot_mode = (robot_ctrl.shoot_mode == SHOOT_STOP) ? SHOOT_READY : SHOOT_STOP;
            }
            last_f_key = (robot_ctrl.rc->key.v & KEY_F); // 更新F键状态，用于下一次防抖判断

            /********************* 子逻辑2：云台工作模式切换 【三模式：失能/手动/自瞄】 *********************/
            // 云台失能指令：遥控器暂停键 或 键盘C键，按下解锁云台电机，云台随动无阻力
            uint8_t relax_cmd = (robot_ctrl.rc->rc.pause) || (robot_ctrl.rc->key.v & KEY_C);
            uint8_t relax_trigger = (relax_cmd && !last_relax_toggle); // 失能模式切换上升沿
            // 云台手动/自瞄切换指令：遥控器自定义左按键 或 键盘G键，失能模式下不可切换
            uint8_t mode_cmd = (robot_ctrl.rc->rc.custom_l) || (robot_ctrl.rc->key.v & KEY_G);
            uint8_t mode_trigger = (mode_cmd && !last_mode_toggle);     // 手动/自瞄切换上升沿

            // 执行失能模式切换
            if (relax_trigger) {
                robot_ctrl.gimbal_mode = (robot_ctrl.gimbal_mode == GIMBAL_RELAX) ? GIMBAL_REMOTE : GIMBAL_RELAX;
                is_initialized = 0; // 切换失能模式后，云台角度重新初始化
            }
            // 执行手动/自瞄模式切换（失能模式下不响应）
            if (mode_trigger && robot_ctrl.gimbal_mode != GIMBAL_RELAX) {
                robot_ctrl.gimbal_mode = (robot_ctrl.gimbal_mode == GIMBAL_REMOTE) ? GIMBAL_AUTO : GIMBAL_REMOTE;
            }
            // 更新防抖标志位
            last_relax_toggle = relax_cmd;
            last_mode_toggle = mode_cmd;

            /********************* 子逻辑3：云台姿态闭环控制 【手动/自瞄模式生效】 *********************/
            if (robot_ctrl.gimbal_mode != GIMBAL_RELAX) {
                // 云台角度首次初始化：以上电后的当前角度为基准，防止云台突变
                if (is_initialized == 0) {
                    world_yaw_target = robot_ctrl.gimbal.yaw;
                    world_pit_target = robot_ctrl.gimbal.pitch;
                    is_initialized = 1;
                }

                // --------------------- 手动控制模式：遥控器摇杆+鼠标控制云台 ---------------------
                if (robot_ctrl.gimbal_mode == GIMBAL_REMOTE) {
                    // 手动模式指示灯：绿灯常亮，红灯灭，蓝灯灭
                    LED_RED_RESET();
                    LED_BLUE_RESET();
                    LED_GREEN_SET();

                    // 遥控器摇杆值归一化：-1~1区间，死区处理防止漂移
                    float ry = (abs(robot_ctrl.rc->rc.ch[2]) > RC_DEADZONE) ? robot_ctrl.rc->rc.ch[2] / 660.0f : 0.0f;
                    float rx = (abs(robot_ctrl.rc->rc.ch[3]) > RC_DEADZONE) ? robot_ctrl.rc->rc.ch[3] / 660.0f : 0.0f;
                    // 鼠标移动值直接参与控制，乘以灵敏度系数
                    float mouse_x = (float)robot_ctrl.rc->mouse.x * MOUSE_YAW_SENS;
                    float mouse_y = (float)robot_ctrl.rc->mouse.y * MOUSE_PIT_SENS;

                    // 更新云台世界坐标系目标角度
                    world_pit_target -= (ry * RC_PIT_SENS) + mouse_y;
                    world_yaw_target -= (rx * RC_YAW_SENS) + mouse_x;

                    // 俯仰角物理限位保护，强制限制在安全区间内
                    if(world_pit_target > PITCH_UP_LIMIT)  world_pit_target = PITCH_UP_LIMIT;
                    if(world_pit_target < PITCH_DOWN_LIMIT)world_pit_target = PITCH_DOWN_LIMIT;
                }
                // --------------------- 自瞄控制模式：上位机传参自动瞄准目标 ---------------------
                else if (robot_ctrl.gimbal_mode == GIMBAL_AUTO) {
                    // 解析自瞄目标数据，返回1=有目标，0=无目标
                    if (parse_target_data(&robot_ctrl.target_info) == 1) {
                        // 自瞄锁定目标：蓝灯常亮，红灯灭，绿灯灭
                        LED_RED_RESET();
                        LED_GREEN_RESET();
                        LED_BLUE_SET();
                        // 更新云台目标角度为自瞄解算的角度
                        world_yaw_target = robot_ctrl.target_info.aim_target_yaw;
                        world_pit_target = robot_ctrl.target_info.aim_target_pitch;
                        // 俯仰角物理限位保护
                        if(world_pit_target > PITCH_UP_LIMIT)  world_pit_target = PITCH_UP_LIMIT;
                        if(world_pit_target < PITCH_DOWN_LIMIT)world_pit_target = PITCH_DOWN_LIMIT;
                    }
                    // 自瞄无目标状态
                    else {
                        // 自瞄无目标指示灯：蓝灯【慢闪200ms】，红灯灭，绿灯灭 【本次核心优化，解决闪烁过快】
                        LED_RED_RESET();
                        LED_GREEN_RESET();
                        if(current_tick - led_tick > LED_BLINK_INTERVAL)
                        {
                            LED_BLUE_Toggle();
                            led_tick = current_tick;
                        }
                        robot_ctrl.target_info.shoot = 0; // 无目标时禁止开火
                    }
                }

                // 读取云台电机当前实时角度
                float cur_yaw, cur_pit;
                yaw_m->get_status(yaw_m, "POS", &cur_yaw);
                pit_m->get_status(pit_m, "POS", &cur_pit);

                // 解算云台电机最终目标角度，弧度归一化处理，保证控制连续性
                float yaw_out = cur_yaw + Rad_Format(world_yaw_target - robot_ctrl.gimbal.yaw + robot_ctrl.chassis.yaw_speed);
                float pit_out = cur_pit - (world_pit_target - robot_ctrl.gimbal.pitch);

                // 俯仰角二次限位，双重保险
                if (pit_out > PITCH_UP_LIMIT) pit_out = PITCH_UP_LIMIT;
                if (pit_out < PITCH_DOWN_LIMIT) pit_out = PITCH_DOWN_LIMIT;

                // 下发目标角度到云台电机，执行闭环控制
                yaw_m->set_target(yaw_m, 1, yaw_out);
                pit_m->set_target(pit_m, 1, pit_out);
            }
            /********************* 子逻辑4：云台失能模式 *********************/
            else if (robot_ctrl.gimbal_mode == GIMBAL_RELAX) {
                // 失能模式指示灯：红灯常亮，绿灯灭，蓝灯灭
                LED_RED_SET();
                LED_GREEN_RESET();
                LED_BLUE_RESET();
            }

            /********************* 子逻辑5：发射机构-摩擦轮转速控制 *********************/
            if (robot_ctrl.shoot_mode == SHOOT_READY) {
                // 摩擦轮就绪：下发目标转速，左右轮转速一致
                float shoot_l_speed = (SHOOT_FW_SPEED > M3508_MAX_SPEED) ? M3508_MAX_SPEED : SHOOT_FW_SPEED;
                float shoot_r_speed = (SHOOT_FW_SPEED > M3508_MAX_SPEED) ? M3508_MAX_SPEED : SHOOT_FW_SPEED;
                shoot_l->set_target(shoot_l, 1,  shoot_l_speed);
                shoot_r->set_target(shoot_r, 1, shoot_r_speed);
            }
            else {
                // 摩擦轮停止：目标转速清零
                shoot_l->set_target(shoot_l, 1, 0);
                shoot_r->set_target(shoot_r, 1, 0);
            }

            /********************* 子逻辑6：发射机构-拨弹盘精准控制【核心】1颗弹精准送弹/退弹 *********************/
            uint8_t cur_l_press = robot_ctrl.rc->mouse.press_l;  // 鼠标左键当前状态：1=按下，0=松开
            uint8_t cur_m_press = robot_ctrl.rc->mouse.press_m;  // 鼠标中键当前状态：1=按下，0=松开
            uint8_t shoot_cmd = 0;                               // 开火拨弹指令标志位
            uint8_t reverse_cmd = cur_m_press;                   // 反转退弹指令标志位

            // 开火条件判定：分模式生效，防止误触开火
            if(robot_ctrl.gimbal_mode == GIMBAL_REMOTE)
            {
                // 手动模式：鼠标左键按下 + 摩擦轮就绪，即可开火
                shoot_cmd = cur_l_press && (robot_ctrl.shoot_mode == SHOOT_READY);
            }
            else if(robot_ctrl.gimbal_mode == GIMBAL_AUTO)
            {
                // 自瞄模式：鼠标左键按下 + 摩擦轮就绪 + 自瞄有目标，才可开火，精准打击
                shoot_cmd = cur_l_press && (robot_ctrl.shoot_mode == SHOOT_READY) && (robot_ctrl.target_info.shoot == 1);
            }

            // 鼠标左键上升沿触发：单次精准拨弹，送1颗弹到位即停，无无效旋转
            if (shoot_cmd && !last_l_press) {
                stir_base_rad -= STIR_SHOOT_RAD + STIR_POS_OFFSET; // 正转拨弹：弧度递减，与拨弹盘旋转方向一致
                stir_m->set_target(stir_m, 1, stir_base_rad);      // 下发精准弧度目标到M2006底层
            }
            // 鼠标中键上升沿触发：单次精准退弹，防止弹丸卡死在拨弹盘内
            else if (reverse_cmd && !last_m_press) {
                stir_base_rad += STIR_REVERSE_RAD;                 // 反转退弹：弧度递增，反向旋转
                stir_m->set_target(stir_m, 1, stir_base_rad);      // 下发精准弧度目标到M2006底层
            }
            // 更新鼠标按键防抖标志位
            last_l_press = cur_l_press;
            last_m_press = cur_m_press;
        }

        /**************************************** 子逻辑7：串口调试数据打印 ****************************************/
        float p_des;                // M2006拨弹电机目标弧度
        float relative_pos;         // M2006拨弹电机实时弧度(拨弹盘实际角度)
        stir_m->get_status(stir_m, "p_des", &p_des);          // 读取目标弧度
        stir_m->get_status(stir_m, "relative_pos", &relative_pos); // 读取实时弧度
        // 打印格式：拨弹盘实时弧度:xx.xx, 目标弧度:xx.xx，直观查看拨弹盘旋转状态，调试必备
        //Uart->Print(Uart, "拨弹盘实时弧度:%.2f, 目标弧度:%.2f\r\n", relative_pos, p_des);

        osDelay(2); // 任务周期2ms，保证控制实时性，不可随意增大延时
    }
}