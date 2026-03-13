#ifndef REMOTE_H
#define REMOTE_H

#include "../../Application/struct_typedef.h"
#include "stdint.h"

/******************************************************************************************
 *                                   DT7 遥控器 宏定义 (原DT7全部保留)
 ******************************************************************************************/
#define SBUS_RX_BUF_NUM_DT7      36u        // DT7 DMA缓存区长度
#define RC_FRAME_LENGTH_DT7      18u        // DT7有效数据长度 固定18字节

// DT7摇杆原始值限幅
#define RC_CH_VALUE_MIN_DT7      ((uint16_t)364)
#define RC_CH_VALUE_OFFSET_DT7   ((uint16_t)1024)
#define RC_CH_VALUE_MAX_DT7      ((uint16_t)1684)

// DT7拨杆挡位定义
#define RC_SW_UP_DT7             ((uint16_t)1)
#define RC_SW_MID_DT7            ((uint16_t)3)
#define RC_SW_DOWN_DT7           ((uint16_t)2)
#define switch_is_down(s)        (s == RC_SW_DOWN_DT7)
#define switch_is_mid(s)         (s == RC_SW_MID_DT7)
#define switch_is_up(s)          (s == RC_SW_UP_DT7)

// DT7键盘按键位映射 (Bitmask)
#define KEY_DT7_W                ((uint16_t)1 << 0)
#define KEY_DT7_S                ((uint16_t)1 << 1)
#define KEY_DT7_A                ((uint16_t)1 << 2)
#define KEY_DT7_D                ((uint16_t)1 << 3)
#define KEY_DT7_SHIFT            ((uint16_t)1 << 4)
#define KEY_DT7_CTRL             ((uint16_t)1 << 5)
#define KEY_DT7_Q                ((uint16_t)1 << 6)
#define KEY_DT7_E                ((uint16_t)1 << 7)
#define KEY_DT7_R                ((uint16_t)1 << 8)
#define KEY_DT7_F                ((uint16_t)1 << 9)
#define KEY_DT7_G                ((uint16_t)1 << 10)
#define KEY_DT7_Z                ((uint16_t)1 << 11)
#define KEY_DT7_X                ((uint16_t)1 << 12)
#define KEY_DT7_C                ((uint16_t)1 << 13)
#define KEY_DT7_V                ((uint16_t)1 << 14)
#define KEY_DT7_B                ((uint16_t)1 << 15)

/******************************************************************************************
 *                                   VT13 遥控器 宏定义 (原VT13全部保留)
 ******************************************************************************************/
#define RC_FRAME_LENGTH_VT13     21u        // VT13有效数据长度 固定21字节
#define RC_RX_BUF_SIZE_VT13      (RC_FRAME_LENGTH_VT13 * 2)

// VT13摇杆/拨轮 通道限幅
#define RC_CH_MIN_VT13           364
#define RC_CH_MID_VT13           1024
#define RC_CH_MAX_VT13           1684
#define RC_CH_RANGE_VT13         660
#define RC_CH_VALUE_OFFSET_VT13  ((uint16_t)1024)

// VT13挡位切换开关
#define RC_SW_C_VT13             ((uint8_t)0)  // 下 (对应挡位C)
#define RC_SW_N_VT13             ((uint8_t)1)  // 中 (对应挡位N)
#define RC_SW_S_VT13             ((uint8_t)2)  // 上 (对应挡位S)

// VT13状态逻辑定义
#define RC_BTN_UP_VT13           ((uint8_t)0)  // 未按下
#define RC_BTN_DOWN_VT13         ((uint8_t)1)  // 按下

// VT13键盘按键位映射 (Bitmask)
#define KEY_VT13_W               ((uint16_t)0x0001)
#define KEY_VT13_S               ((uint16_t)0x0002)
#define KEY_VT13_A               ((uint16_t)0x0004)
#define KEY_VT13_D               ((uint16_t)0x0008)
#define KEY_VT13_SHIFT           ((uint16_t)0x0010)
#define KEY_VT13_CTRL            ((uint16_t)0x0020)
#define KEY_VT13_Q               ((uint16_t)0x0040)
#define KEY_VT13_E               ((uint16_t)0x0080)
#define KEY_VT13_R               ((uint16_t)0x0100)
#define KEY_VT13_F               ((uint16_t)0x0200)
#define KEY_VT13_G               ((uint16_t)0x0400)
#define KEY_VT13_Z               ((uint16_t)0x0800)
#define KEY_VT13_X               ((uint16_t)0x1000)
#define KEY_VT13_C               ((uint16_t)0x2000)
#define KEY_VT13_V               ((uint16_t)0x4000)
#define KEY_VT13_B               ((uint16_t)0x8000)

/* Helper macro: evaluate a key bitmask against a value and return 0 or 1.
   Use this instead of assigning (v & KEY) directly to a uint8_t to avoid
   truncation when KEY has bits above 0x00FF. */
#define KEY_PRESSED(v, k)        (((v) & (k)) ? 1 : 0)

/******************************************************************************************
 *                                   VT13 原始数据结构体 (21 Bytes，原VT13保留)
 ******************************************************************************************/
typedef struct __attribute__((packed)) {
    uint8_t sof_1;          // 0xA9
    uint8_t sof_2;          // 0x53
    uint64_t ch_0:11;       // 右水平
    uint64_t ch_1:11;       // 右竖直
    uint64_t ch_2:11;       // 左竖直
    uint64_t ch_3:11;       // 左水平
    uint64_t mode_sw:2;     // 挡位 C/N/S
    uint64_t btn_pause:1;   // 暂停
    uint64_t btn_custom_l:1;// 自定义左
    uint64_t btn_custom_r:1;// 自定义右
    uint64_t wheel:11;      // 拨轮
    uint64_t btn_trigger:1; // 扳机

    int16_t mouse_x;        // 鼠标左右增量
    int16_t mouse_y;        // 鼠标前后增量
    int16_t mouse_z;        // 鼠标滚轮增量
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t key_v;         // 键盘按键位图
    uint16_t crc16;
} remote_raw_t;

/******************************************************************************************
 *                                   DT7 独立数据结构体 (原DT7逻辑结构，无修改)
 ******************************************************************************************/
typedef struct __attribute__((packed))
{
    struct __attribute__((packed))
    {
        int16_t ch[4];      // 摇杆 [-660, 660]
        int16_t wheel;
        char sw_l;          // 两个拨杆 左s[0] 右s[1]
        char sw_r;
    } rc_dt7;
    struct __attribute__((packed))
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse_dt7;
    struct __attribute__((packed))
    {
        uint16_t v;         // 键盘位图
    } key_dt7;
    uint32_t last_update_tick; // DT7最后更新时间戳
} RC_dt7_t;

/******************************************************************************************
 *                                   VT13 独立数据结构体 (原VT13逻辑结构，无修改)
 ******************************************************************************************/
typedef struct {
    struct {
        int16_t ch[4];      // 摇杆 [-660, 660]
        uint8_t sw;         // 挡位 0, 1, 2
        uint8_t pause;      // 暂停按键 0, 1
        uint8_t custom_l;   // 自定义左 0, 1
        uint8_t custom_r;   // 自定义右 0, 1
        int16_t wheel;      // 拨轮 [-660, 660]
        uint8_t trigger;    // 扳机按键 0, 1
    } rc_vt13;
    struct {
        int16_t x, y, z;
        uint8_t press_l, press_r, press_m;
    } mouse_vt13;
    struct {
        uint16_t v;         // 键盘位图
    } key_vt13;
    uint32_t last_update_tick;  // VT13最后更新时间戳
} RC_vt13_t;

/******************************************************************************************
 *                                   【核心】合并后的总遥控器结构体
 * 上层调用格式：robot_ctrl.rc->rc_dt7.ch[0]  robot_ctrl.rc->mouse_vt13.x
 ******************************************************************************************/
typedef struct {
    RC_dt7_t  dt7;    // DT7遥控器全部数据
    RC_vt13_t vt13;   // VT13遥控器全部数据
} RC_ctrl_t;

/******************************************************************************************
 *                                   函数声明
 ******************************************************************************************/
void RC_Init_DT7(void);    // DT7初始化
void RC_Init_VT13(void);   // VT13初始化
void RC_Init(void);        // 一键初始化双遥控器
const RC_ctrl_t *RC_Get_Handle(void); // 获取总遥控器句柄
void RC_Unable_DT7(void);
void RC_Unable_VT13(void);

#endif

