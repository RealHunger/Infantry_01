#include "sensor_task.h"
#include "../Components/BMI088/BMI088driver.h"
#include "../Application/robot_global.h"
#include "../Algorithm/MahonyAHRS/MahonyAHRS.h"
#include "math.h"
#include "cmsis_os.h"
#include "../Bsp/usb_cdc/bsp_usb_cdc.h"
#include "main.h"

/***********************************************************************************************************************
* 文 件 说 明：传感器数据采集任务主函数 - 姿态解算核心任务 + BMI088 40℃恒温PID补偿控制
* 硬件适配：BMI088六轴惯性传感器(三轴陀螺仪+三轴加速度计)
* 核心功能：1.BMI088硬件初始化 2.陀螺仪静态零偏校准抑制零漂 3.Mahony姿态解算四元数 4.四元数转欧拉角
*          5.姿态角/角速度同步至全局变量 6.UART+USB双上位机数据打印 7.BMI088恒温40℃ PID闭环补偿控制
*          8.内置完整位置式PID实现+PWM寄存器直接输出 9.无外部头文件依赖 10.为云台/底盘提供姿态基准数据
* 任务周期：1ms(vTaskDelay(1))，高频采集保证姿态解算精度与温度控制实时性
***********************************************************************************************************************/
// ======================== 【内置位置式PID 核心实现 - 无外部依赖】 ========================
#define PID_POSITION            0           // PID控制模式：位置式PID

/**
 * @brief PID控制器结构体定义，位置式PID参数与状态量全集
 */
typedef struct
{
    uint8_t mode;                           // PID工作模式
    fp32 Kp;                                // PID比例系数
    fp32 Ki;                                // PID积分系数
    fp32 Kd;                                // PID微分系数

    fp32 target_val;                        // 控制目标值
    fp32 actual_val;                        // 反馈实际值
    fp32 err;                               // 当前偏差值 (目标-实际)
    fp32 err_last;                          // 上一帧偏差值，微分项计算用

    fp32 integral;                          // PID积分累计值
    fp32 integral_limit;                    // PID积分限幅，防止积分饱和
    fp32 output_limit;                      // PID输出限幅，限制执行器输出范围
    fp32 out;                               // PID最终输出值
} pid_type_def;

/**
 * @brief 位置式PID初始化函数
 * @param pid PID结构体指针
 * @param mode PID控制模式
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 * @param max_out PID最大输出限幅值
 * @param max_iout PID积分最大限幅值
 */
static void PID_init(pid_type_def *pid, uint8_t mode, fp32 Kp, fp32 Ki, fp32 Kd, fp32 max_out, fp32 max_iout)
{
    pid->mode = mode;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_limit = max_out;
    pid->integral_limit = max_iout;
    pid->integral = 0.0f;
    pid->out = 0.0f;
}

/**
 * @brief 位置式PID核心计算函数
 * @param pid PID结构体指针
 * @param actual_val 当前反馈的实际值
 * @param target_val 控制的目标期望值
 */
static void PID_calc(pid_type_def *pid, fp32 actual_val, fp32 target_val)
{
    pid->actual_val = actual_val;
    pid->target_val = target_val;
    pid->err = pid->target_val - pid->actual_val;

    // 位置式PID标准计算公式：输出 = KP*偏差 + KI*积分 + KD*(当前偏差-上一帧偏差)
    pid->out = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->err_last);

    // 积分限幅，防止积分饱和导致的PID失控超调
    if(pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if(pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    else pid->integral += pid->err;

    // 输出限幅，限制PWM输出范围，下限为0关闭加热，上限为设定最大值
    if(pid->out > pid->output_limit) pid->out = pid->output_limit;
    else if(pid->out < 0.0f) pid->out = 0.0f;

    pid->err_last = pid->err;                // 更新上一帧偏差值
}
// ======================== PID 核心实现结束 ========================

// ======================== BMI088温度恒温补偿配置参数 ========================
#define TEMPERATURE_TARGET      40.0f        // BMI088目标恒温值，抑制温漂最佳温度40℃
#define TEMPERATURE_PID_KP      1600.0f      // 温度环PID-比例系数
#define TEMPERATURE_PID_KI      0.2f         // 温度环PID-积分系数
#define TEMPERATURE_PID_KD      0.0f         // 温度环PID-微分系数
#define TEMPERATURE_PID_MAX_OUT 4500.0f      // PID最大输出限幅值
#define TEMPERATURE_PID_MAX_IOUT 4400.0f     // PID积分最大限幅值
pid_type_def temp_pid;                       // 温度控制PID结构体实例，命名：temp_pid

// ======================== 全局声明 & 内部缓存变量 ========================
extern SPI_HandleTypeDef hspi1;              // SPI1句柄声明，BMI088通信使用
static fp32 INS_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // 姿态解算四元数 w/x/y/z，初始化单位四元数
static fp32 ins_angle[3], gyro[3], accel[3], temp;   // 缓存变量：欧拉角/陀螺仪角速度/加速度计/传感器温度
static fp32 gyro_bias[3] = {0.0f, 0.0f, 0.0f};      // 陀螺仪三轴零偏校准值，核心防漂移参数
static uint16_t cali_count = 0;                     // 零偏校准采样计数器
#define CALI_SAMPLES 1000                           // 零偏校准采样总次数 1000次(1000ms)

/***********************************************************************************************************************
* 弧度角度归一化处理函数，将任意弧度值限制在 [-π, π] 区间内，保证角度计算连续性，无累计误差
***********************************************************************************************************************/
static float Rad_Format(float angle) {
    while (angle >  M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

/***********************************************************************************************************************
* IMU加热PWM输出函数 - 无外部头文件依赖，直接寄存器操作，控制TIM10_CH1输出PWM波驱动加热片
***********************************************************************************************************************/
static void imu_pwm_set(uint16_t pwm)
{
    TIM10->CCR1 = pwm;
}

/***********************************************************************************************************************
* 传感器任务主函数 - FreeRTOS独立任务，优先级最高，姿态解算+温度补偿唯一入口
***********************************************************************************************************************/
void sensor_task_func(void const * argument) {
    /**************************************** 第一步：外设初始化配置 ****************************************/
    // 初始化调试串口1 DMA，波特率115200，用于单独打印IMU温度数据
    struct uart_device* Uart = uart_get_device("uart1_dma");
    Uart->Init(Uart, 115200, 8, 'N', 1);

    // 初始化USB上位机通信设备，用于透传打印姿态四元数+欧拉角数据
    struct usb_device *Usb = usb_get_device();
    Usb->Init(Usb);

    // BMI088六轴传感器硬件初始化，初始化失败则延时10ms重试，直到初始化成功
    while (BMI088_init() != 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // SPI1通信速率配置：8分频，匹配BMI088传感器的通信速率要求
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

    // 初始化BMI088温度控制PID，配置参数+限幅值，启用位置式PID模式
    PID_init(&temp_pid, PID_POSITION, TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);

    /**************************************** 第二步：陀螺仪静态零偏校准【核心防漂逻辑】 ****************************************/
    // 重要提醒：此阶段机器人必须保持绝对静止，否则校准的零偏值无效，姿态解算会出现角度漂移
    for (cali_count = 0; cali_count < CALI_SAMPLES; cali_count++) {
        BMI088_read(gyro, accel, &temp);               // 读取传感器原始数据
        gyro_bias[0] += gyro[0];                       // X轴角速度采样值累加
        gyro_bias[1] += gyro[1];                       // Y轴角速度采样值累加
        gyro_bias[2] += gyro[2];                       // Z轴角速度采样值累加
        vTaskDelay(1);                                 // 采样间隔1ms，保证采样数据均匀有效
    }
    // 累加采样值求平均值，得到三轴精准的陀螺仪零偏补偿值
    gyro_bias[0] /= CALI_SAMPLES;
    gyro_bias[1] /= CALI_SAMPLES;
    gyro_bias[2] /= CALI_SAMPLES;

    /**************************************** 第三步：标记传感器就绪，释放其他任务运行条件 ****************************************/
    robot_ctrl.monitor.sensor_ready = 1;

    /**************************************** 第四步：传感器数据采集+姿态解算+温度补偿主循环【死循环】 ****************************************/
    while (1) {
        // --- 1. 读取BMI088原始数据 并 减去零偏补偿值，消除硬件固有零漂 ---
        BMI088_read(gyro, accel, &temp);
        gyro[0] -= gyro_bias[0];
        gyro[1] -= gyro_bias[1];
        gyro[2] -= gyro_bias[2];

        // --- 2. Mahony姿态解算算法 更新四元数 【单位修正：deg/s → rad/s】
        MahonyAHRSupdateIMU(INS_q,gyro[0]*0.0174533f,gyro[1]*0.0174533f,gyro[2]*0.0174533f,accel[0],accel[1],accel[2]);

        // --- 3. 四元数转欧拉角解算 姿态角输出 (单位：弧度) ---
        ins_angle[0] = atan2f(2.0f*(INS_q[0]*INS_q[3]+INS_q[1]*INS_q[2]), 2.0f*(INS_q[0]*INS_q[0]+INS_q[1]*INS_q[1])-1.0f); // Yaw 偏航角
        ins_angle[1] = asinf(-2.0f*(INS_q[1]*INS_q[3]-INS_q[0]*INS_q[2]));                                             // Roll 横滚角
        ins_angle[2] = atan2f(2.0f*(INS_q[0]*INS_q[1]+INS_q[2]*INS_q[3]), 2.0f*(INS_q[0]*INS_q[0]+INS_q[3]*INS_q[3])-1.0f); // Pitch 俯仰角

        // --- 4. 姿态数据同步至全局变量 供云台/底盘任务读取使用 ---
        robot_ctrl.gimbal.yaw = ins_angle[0];
        robot_ctrl.gimbal.pitch = ins_angle[2];
        robot_ctrl.gimbal.roll = ins_angle[1];
        robot_ctrl.gimbal.yaw_v = gyro[2]*0.0174533f;
        robot_ctrl.gimbal.pitch_v = gyro[1]*0.0174533f;

        // --- 5. BMI088温度PID闭环补偿 + PWM加热输出 ---
        PID_calc(&temp_pid, temp, TEMPERATURE_TARGET);  // PID计算：当前温度→目标40℃
        imu_pwm_set((uint16_t)temp_pid.out);            // 输出PWM值控制加热片恒温

        // --- 6. 上位机数据打印 ---
        Usb->Print(Usb, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", INS_q[0],INS_q[1],INS_q[2],INS_q[3],robot_ctrl.gimbal.yaw,robot_ctrl.gimbal.pitch);
        Uart->Print(Uart, "%f\r\n", temp);              // 串口单独打印当前IMU温度值

        vTaskDelay(1);  // 传感器任务周期1ms，保证姿态解算精度与温度控制实时性
    }
}