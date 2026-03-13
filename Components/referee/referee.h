#ifndef REFEREE_H
#define REFEREE_H

#include "stdint.h"

#define REF_RX_BUF_SIZE 256
#define REF_HEADER_SOF  0xA5

// 1. 官方协议帧头结构体 (5字节)
typedef struct __attribute__((packed)) {
    uint8_t  SOF;          // 起始字节 0xA5
    uint16_t data_length;  // 数据帧中 data 的长度
    uint8_t  seq;          // 包序号
    uint8_t  CRC8;         // 帧头 CRC8 校验
} frame_header_struct_t;

// 比赛状态数据：0x0001 (11字节)
typedef struct __attribute__((packed)) {
    uint8_t game_type : 4;       // 比赛类型: 1:RMUC, 2:RMUT, 3:RMUL, 4:3V3, 5:1V1
    uint8_t game_progress : 4;   // 比赛阶段: 0:未开始, 1:准备区, 2:自检区, 3:5秒倒计时, 4:比赛中, 5:结算中
    uint16_t stage_remain_time;  // 当前阶段剩余时间 (单位：秒)
    uint64_t SyncTimeStamp;      // 机器人与裁判系统时间同步的 UNIX 时间戳 (微秒)
} ext_game_status_t;

// 1. 0x0101（4字节）
typedef struct __attribute__((packed)) {
    uint32_t place_t;           //场地信息
} ext_place_status_t;

// 2. 机器人性能状态数据：0x0201 (13字节，10Hz发送) —— 【云台/底盘核心】
typedef struct __attribute__((packed)) {
    uint8_t robot_id;                        // 本机器人ID (1:红英雄, 3/4/5:红步兵, 7:红哨兵 | 101:蓝英雄, 103/104/105:蓝步兵...)
    uint8_t robot_level;                     // 机器人当前等级 (1级, 2级, 3级)
    uint16_t current_HP;                     // 当前血量
    uint16_t maximum_HP;                     // 血量上限
    uint16_t shooter_barrel_cooling_value;   // 枪口每秒冷却值 (由等级决定)
    uint16_t shooter_barrel_heat_limit;      // 枪口热量上限 (超过该值扣血)
    uint16_t chassis_power_limit;            // 底盘功率上限 (超过该值扣血，单位：W)
    uint8_t power_management_gimbal_output : 1;  // 供电管理: 云台 24V 输出情况 (0:无输出, 1:有输出)
    uint8_t power_management_chassis_output : 1; // 供电管理: 底盘 24V 输出情况 (0:无输出, 1:有输出)
    uint8_t power_management_shooter_output : 1; // 供电管理: 发射 24V 输出情况 (0:无输出, 1:有输出)
} ext_game_robot_status_t;

// 3. 实时功率热量数据：0x0202 (14字节，50Hz发送) —— 【拨弹轮防超热量核心】
typedef struct __attribute__((packed)) {
    uint16_t reserved1;                 // 协议保留位 (原底盘输出电压)
    uint16_t reserved2;                 // 协议保留位 (原底盘输出电流)
    float    reserved3;                 // 协议保留位 (原底盘输出功率)
    uint16_t buffer_energy;             // 底盘缓冲能量 (单位：J，小于0时开始扣血)
    uint16_t shooter_17mm_barrel_heat;  // 17mm 枪口当前热量 (打一颗加10，必须依靠它做限制)
    uint16_t shooter_42mm_barrel_heat;  // 42mm 枪口当前热量 (英雄机器人用，步兵一般为0)
} ext_power_heat_data_t;

// 4. 机器人绝对位置数据：0x0203 (12字节，10Hz发送) —— 【哨兵自主导航核心】
typedef struct __attribute__((packed)) {
    float x;    // 本机器人位置的 X 坐标 (单位：米)
    float y;    // 本机器人位置的 Y 坐标 (单位：米)
    float yaw;  // 本机器人的陀螺仪偏航角 (单位：度)
} ext_game_robot_pos_t;

//5、 机器人受击数据 ：0x0206（受击情况）
typedef struct __attribute__((packed)) {
    uint8_t armor_id : 4;
    uint8_t HP_deducation_reason : 4;
} ext_huart_robot_data_t;

//6、 机器人发弹相关 ： 0x0208（弹药情况）
typedef struct __attribute__((packed)) {
    uint16_t allow_bullet_17;
    uint16_t allow_bullet_42;
    uint16_t money_left;
    uint16_t extra_bullet;
}ext_allow_robot_data_t;

// 裁判系统总控结构体
typedef struct {
    ext_game_status_t       game_status;     // 包含：比赛阶段、剩余时间
    ext_place_status_t      place_status;    // 包含： 场地信息
    ext_game_robot_status_t robot_status;    // 包含：等级、血量、热量上限、功率上限
    ext_power_heat_data_t   power_heat_data; // 包含：当前17mm热量、缓冲能量
    ext_game_robot_pos_t    robot_pos;       // 包含：X, Y, Z, Yaw 坐标与朝向
    ext_huart_robot_data_t  huart_robot;     // 包含受伤情况
    ext_allow_robot_data_t  allow_robot;     // 弹药可用情况

    uint32_t last_update_tick; // 掉线检测时间戳
} referee_info_t;

extern referee_info_t referee_data;

// 接口声明
void Referee_Init(void);
void Referee_Data_Parse(uint8_t *rx_buf, uint16_t len);
void Referee_Send_Packet(uint16_t cmd_id, uint8_t *data, uint16_t data_len);
void Referee_Debug_Print(void);
void Referee_CAN_Forward(void);

#endif // REFEREE_H