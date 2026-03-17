#ifndef INFANTRY_01_POWER_REALLOCATE_H
#define INFANTRY_01_POWER_REALLOCATE_H

#include <stdint.h>
#include <math.h>

// 底盘电机目前的pid计算得到的，反馈的真实总功率、当前的动态功率上限
// 输出：重分配后的电流数组
void reallocate_v2(int16_t current_des[4], float real_power_now, int16_t output_current[4], float p_max);

// 能量环保持不变
float energy_power(float energy_target, float energy_now, float ref_power_max);

#endif