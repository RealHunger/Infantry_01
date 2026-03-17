#ifndef INFANTRY_01_POWER_REALLOCATE_H
#define INFANTRY_01_POWER_REALLOCATE_H

#include <stdint.h>
#include <math.h>
#include "../Bsp/RLS/RLS.h"

// 总调用对外接口
void reallocate(int16_t motor_send_data[4][2], float RLS_argument[6], int16_t reallocate_current[4], float power_max);

// 功率计算
float power_caculate(int16_t motor_send_data[4][2], float RLS_argument[6]);

// 再分配功率函数
// 传入电机数据、RLS参数、最大功率，返回衰减系数 eta
float current_caculate(int16_t motor_send_data[4][2], float RLS_argument[6], float power_max);

//能量环
//输入为目标能量值，现在能量值，规定能量上限，返回值为分配后的能量上限
float energy_power(float energy_target, float energy_now, float ref_power_max);
#endif //INFANTRY_01_POWER_REALLOCATE_H