#include "power_reallocate.h"

#define MIN_MAXPOWER_CONFIGURED 15.0f

// ==================== 能量环：动态计算功率额度 ====================
float energy_power(float energy_target, float energy_now, float ref_power_max) {
    static float last_error = 0.0f;
    float Kp = 1.8f; // 既然是实时反馈，Kp可以稍微大一点，反应更快
    float Kd = 0.15f; 

    float error = energy_target - energy_now;
    float dynamic_pmax = ref_power_max - (Kp * error) - (Kd * (error - last_error));
    last_error = error;

    // 限幅保护
    if (dynamic_pmax < MIN_MAXPOWER_CONFIGURED) dynamic_pmax = MIN_MAXPOWER_CONFIGURED;
    if (dynamic_pmax > ref_power_max + 300.0f) dynamic_pmax = ref_power_max + 300.0f;

    return dynamic_pmax;
}

// ==================== 功率分配 V2：直接比例缩放 ====================
void reallocate_v2(int16_t current_des[4], float real_power_now, int16_t output_current[4], float p_max) {
    // 1. 如果当前真实功率已经在安全范围内，且没有超支风险
    if (real_power_now <= p_max) {
        for(int i=0; i<4; i++) output_current[i] = current_des[i];
    } 
    // 2. 如果超功率了，或者功率非常接近上限
    else {
        // 计算缩放系数 (安全系数建议设为 0.95 留一点余量)
        float eta = (p_max / real_power_now) * 0.95f;
        
        // 防御性截断，防止eta计算异常
        if (eta > 1.0f) eta = 1.0f;
        if (eta < 0.0f) eta = 0.0f;

        for(int i=0; i<4; i++) {
            output_current[i] = (int16_t)(current_des[i] * eta);
        }
    }
}