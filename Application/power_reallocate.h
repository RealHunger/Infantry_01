#ifndef INFANTRY_01_POWER_REALLOCATE_H
#define INFANTRY_01_POWER_REALLOCATE_H

#include <stdint.h>
#include "../../Application/struct_typedef.h"


// 统一封装的功率分配控制器参数配置
typedef struct {
    float target_energy;      // 目标维持能量 (J) - 能量环的稳态目标
    float referee_power_max;  // 裁判系统限制的基础最大功率 (W)
    float max_power_limit;    // 极限超调输出上限 (W) - 有电容时的最大爆发
    float min_power_limit;    // 极限衰减保底输出 (W) - 防止断电锁死
} Power_Allocate_Config_t;

/**
 * @brief 全局底盘功率重分配核心接口
 * * @param current_des       [4][0]:pid已经下发电流, [4][1]:当前转速
 * @param current_fb        [4][0]:真实反馈电流, [4][1]:当前转速 (用于RLS辨识)
 * @param real_power_fb     硬件真实反馈总功率 (用于RLS真实值矫正)
 * @param current_energy    超级电容当前真实剩余能量 (用于能量环)
 * @param config            系统对应的参数配置
 * @param safe_current_out  [输出] 衰减后安全的下发电流
 */
void Chassis_Power_Control_Loop(
    int16_t current_des[4][2],
    int16_t current_fb[4][2],
    float real_power_fb,
    float current_energy,
    const Power_Allocate_Config_t *config,
    int16_t safe_current_out[4]
);

// 判断电容是否断联
uint8_t verify_feedback_connection();

//
float real_power_feedback();
#endif //INFANTRY_01_POWER_REALLOCATE_H