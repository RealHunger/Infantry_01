#include "motor_task.h"
#include "cmsis_os.h"
#include "../Components/motor/motor.h"
#include "../Application/robot_global.h"
#include "stdio.h"

/***********************************************************************************************************************
* 文 件 说 明：电机驱动任务主函数 - 全机电机统一控制调度核心任务
* 硬件适配：GM6020/3508/2006/J4310 全系列大疆电机
* 核心功能：1.电机系统上电初始化 2.全机所有电机句柄获取 3.模式边缘触发电机使能/失能控制 4.电机PID闭环计算调度
*          5.CAN总线控制指令下发 6.达妙电机专用协议帧发送 7.云台/底盘/发射机构电机统一管理
* 任务周期：1ms(osDelay(1))，高优先级，保证电机控制实时性
***********************************************************************************************************************/
/***********************************************************************************************************************
* 电机任务主函数 - FreeRTOS独立任务，优先级高，全机所有电机控制指令唯一下发入口
***********************************************************************************************************************/
void motor_task_func(void const * argument) {
    /**************************************** 第一步：系统启动安全保护 ****************************************/
    // 等待传感器初始化就绪，防止传感器未就绪时电机提前启动
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);       // 延时1秒，等待所有外设/传感器/电机完全上电稳定
    Motor_System_PowerOn_Init();  // 电机驱动系统上电初始化，配置底层驱动参数

    /**************************************** 第二步：获取全机所有电机设备句柄 ****************************************/
    // 云台电机句柄
    struct motor_device* pitch   = motor_get_device("J4310_PITCH");    // 云台俯仰轴 J4310 电机
    struct motor_device* yaw     = motor_get_device("GM6020_YAW");     // 云台航向轴 GM6020 电机
    // 发射机构电机句柄
    struct motor_device* shoot_l = motor_get_device("M3508_SHOOT_L");  // 发射左轮 M3508 电机
    struct motor_device* shoot_r = motor_get_device("M3508_SHOOT_R");  // 发射左轮 M3508 电机
    struct motor_device* stir_m  = motor_get_device("M2006_TRIGGER");  // 拨弹机构 M2006 电机
    // 底盘4个驱动电机句柄
    struct motor_device* chassis[4];
    for(int i=0; i<4; i++) {
        char name[25]; sprintf(name, "M3508_CHASSIS_%d", i+1);
        chassis[i] = motor_get_device(name);
    }

    /**************************************** 第三步：模式历史状态记录变量 ****************************************/
    // 用于模式边缘触发检测，记录上一帧模式状态，仅模式变化时执行指令，防止重复使能/失能电机
    static gimbal_mode_e  last_gimbal_mode  = GIMBAL_RELAX;
    static chassis_mode_e last_chassis_mode = CHASSIS_RELAX;
    static shoot_mode_e   last_shoot_mode   = SHOOT_STOP;

    /**************************************** 第四步：电机控制主循环【死循环，永不退出】 ****************************************/
    while (1) {
        /* --- A. 边缘触发：云台电机使能/失能控制 仅模式切换时执行 --- */
        if (robot_ctrl.gimbal_mode != last_gimbal_mode) {
            if (robot_ctrl.gimbal_mode == GIMBAL_RELAX) {
                if(pitch) pitch->send_disable_cmd(pitch);  // 云台放松模式：俯仰轴电机失能，无动力输出
                if(yaw)   yaw->send_disable_cmd(yaw);      // 云台放松模式：航向轴电机失能，无动力输出
            } else {
                if(pitch) pitch->send_enable_cmd(pitch);   // 云台工作模式：俯仰轴电机使能，PID闭环控制
                if(yaw)   yaw->send_enable_cmd(yaw);       // 云台工作模式：航向轴电机使能，PID闭环控制
            }
            last_gimbal_mode = robot_ctrl.gimbal_mode;     // 更新云台模式历史状态
        }

        /* --- B. 边缘触发：发射机构电机使能/失能控制 仅模式切换时执行 --- */
        if (robot_ctrl.shoot_mode != last_shoot_mode) {
            if (robot_ctrl.shoot_mode == SHOOT_STOP) {
                if(shoot_l) shoot_l->send_disable_cmd(shoot_l);  // 发射停止：左轮电机失能
                if(shoot_r) shoot_r->send_disable_cmd(shoot_r);  // 发射停止：右轮电机失能
                if(stir_m)  stir_m->send_disable_cmd(stir_m);   // 发射停止：拨弹电机失能
            } else {
                if(shoot_l) shoot_l->send_enable_cmd(shoot_l);  // 发射工作：左轮电机使能
                if(shoot_r) shoot_r->send_enable_cmd(shoot_r);  // 发射工作：右轮电机使能
                if(stir_m)  stir_m->send_enable_cmd(stir_m);   // 发射工作：拨弹电机使能
            }
            last_shoot_mode = robot_ctrl.shoot_mode;       // 更新发射模式历史状态
        }

        /* --- C. 边缘触发：底盘电机使能/失能控制 仅模式切换时执行 --- */
        if (robot_ctrl.chassis_mode != last_chassis_mode) {
            for(int i=0; i<4; i++) {
                if(!chassis[i]) continue;
                if (robot_ctrl.chassis_mode == CHASSIS_RELAX)
                    chassis[i]->send_disable_cmd(chassis[i]);  // 底盘放松模式：对应电机失能，可手动推动
                else
                    chassis[i]->send_enable_cmd(chassis[i]);   // 底盘工作模式：对应电机使能，PID闭环控制
            }
            last_chassis_mode = robot_ctrl.chassis_mode;   // 更新底盘模式历史状态
        }

        /* --- D. 硬件控制指令下发 每1ms执行一次，核心电机控制流程 --- */
        Motor_All_Update();                  // 执行所有电机的PID闭环计算，将目标值转为输出电流
        DJI_Motor_Send_CAN1_Group(&hcan1);   // 发送CAN1总线电机控制指令组帧
        DJI_Motor_Send_CAN2_Group(&hcan2);   // 发送CAN2总线电机控制指令组帧
        if(pitch) pitch->send_ctrl_cmd(pitch); // 达妙J4310俯仰电机使用专用协议帧发送控制指令

        osDelay(1);  // 电机任务周期1ms，保证电机控制实时性与精准度
    }
}