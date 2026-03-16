#include "sensor_task.h"
#include "../Components/BMI088/BMI088driver.h"
#include "../Application/robot_global.h"
#include "../Algorithm/MahonyAHRS/MahonyAHRS.h"
#include "math.h"
#include "cmsis_os.h"
#include "../Bsp/usb_cdc/bsp_usb_cdc.h"

// 姿态解算中间变量
static fp32 INS_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static fp32 ins_angle[3];
static fp32 gyro[3], accel[3], temp;

// 零偏抑制相关变量
static fp32 gyro_bias[3] = {0.0f, 0.0f, 0.0f}; // 陀螺仪零偏
static uint16_t cali_count = 0;
#define CALI_SAMPLES 1000                       // 校准采样次数 (500ms)

void sensor_task_func(void const * argument) {

    // 与上位机通信USB
    struct usb_device *Usb = usb_get_device();
    Usb->Init(Usb);

    struct uart_device* Uart = uart_get_device("uart1_dma");
    Uart->Init(Uart, 115200, 8, 'N', 1);

    // 1. 硬件初始化
    while (BMI088_init() != 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 2. 静态零偏校准 (核心逻辑：抑制0漂)
    // 提醒：此阶段机器人必须保持静止
    for (cali_count = 0; cali_count < CALI_SAMPLES; cali_count++) {
        BMI088_read(gyro, accel, &temp);
        gyro_bias[0] += gyro[0];
        gyro_bias[1] += gyro[1];
        gyro_bias[2] += gyro[2];
        vTaskDelay(1);
    }
    gyro_bias[0] /= CALI_SAMPLES;
    gyro_bias[1] /= CALI_SAMPLES;
    gyro_bias[2] /= CALI_SAMPLES;

    // 3. 标记传感器就绪
    robot_ctrl.monitor.sensor_ready = 1;

    while (1) {
        // 4. 读取传感器数据并减去零偏 (实现0漂抑制)
        BMI088_read(gyro, accel, &temp);
        gyro[0] -= gyro_bias[0];
        gyro[1] -= gyro_bias[1];
        gyro[2] -= gyro_bias[2];

        // 5. 更新四元数 (Mahony 算法)
        // 注意：传参时将 deg/s 转换为 rad/s (0.01745f)
        MahonyAHRSupdateIMU(INS_q,
                            gyro[0],
                            gyro[1],
                            gyro[2],
                            accel[0],
                            accel[1],
                            accel[2]);

        // 6. 姿态角结算 (欧拉角)
        ins_angle[0] = atan2f(2.0f*(INS_q[0]*INS_q[3]+INS_q[1]*INS_q[2]), 2.0f*(INS_q[0]*INS_q[0]+INS_q[1]*INS_q[1])-1.0f); // Yaw
        ins_angle[1] = asinf(-2.0f*(INS_q[1]*INS_q[3]-INS_q[0]*INS_q[2]));                                             // Roll
        ins_angle[2] = atan2f(2.0f*(INS_q[0]*INS_q[1]+INS_q[2]*INS_q[3]), 2.0f*(INS_q[0]*INS_q[0]+INS_q[3]*INS_q[3])-1.0f); // Pitch

        // 7. 同步到全局变量 (弧度)
        robot_ctrl.gimbal.yaw   = ins_angle[0];
        robot_ctrl.gimbal.pitch = ins_angle[2];
        robot_ctrl.gimbal.roll  = ins_angle[1];

        // 同步角速度反馈 (直接给 PID 使用经过消偏后的数据)
        robot_ctrl.gimbal.yaw_v   = gyro[2];
        robot_ctrl.gimbal.pitch_v = gyro[1];

        // =========================================================================
        // 8. 通过 USB 发送融合数据给上位机 (格式: CSV 逗号分隔)
        // 数据顺序: [0-3]四元数, [4]比赛进度, [5]剩余时间, [6]场地状态,
        //          [7]血量, [8]17mm热量, [9]缓冲能量, [10]允许发弹量,
        //          [11]受击装甲板ID, [12]扣血原因
        // // =========================================================================
        // Usb->Print(Usb, "%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
        //            INS_q[0], INS_q[1], INS_q[2], INS_q[3],
        //            robot_ctrl.gateway_referee_t.game_progress,
        //            robot_ctrl.gateway_referee_t.stage_remain_time,
        //            robot_ctrl.gateway_referee_t.place_status,
        //            robot_ctrl.gateway_referee_t.current_HP,
        //            robot_ctrl.gateway_referee_t.shooter_17mm_barrel_heat,
        //            robot_ctrl.gateway_referee_t.buffer_energy,
        //            robot_ctrl.gateway_referee_t.allow_bullet_17,
        //            robot_ctrl.gateway_referee_t.armor_id,
        //            robot_ctrl.gateway_referee_t.HP_deducation_reason);

        Usb->Print(Usb, "%.3f,%.3f,%.3f,%.3f,%d,%d\r\n",
           INS_q[0], INS_q[1], INS_q[2], INS_q[3],
           robot_ctrl.gateway_referee_t.stage_remain_time,
           robot_ctrl.gateway_referee_t.current_HP);

        //Uart->Print(Uart,"%d\r\n", robot_ctrl.rc->dt7.rc_dt7.ch[0]);

        vTaskDelay(1);
    }
}