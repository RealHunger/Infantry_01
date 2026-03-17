//
// Created by ASUS on 2026/3/17.
//

#include "power_reallocate.h"
// 定义极限情况下的最小保底功率 (防止机器人断电锁死)
#define MIN_MAXPOWER_CONFIGURED 15.0f

// 能量环函数，得到功率上限
// Pmax = Prefmax - Kp*e(t) - Kd*(e(t) - e(t-1))
float energy_power(float energy_target, float energy_now, float ref_power_max) {
    // 静态变量保存上一次的误差，用于计算微分(D)项
    static float last_error = 0.0f;

    //还需要慢慢测试喵
    float Kp = 1.5f;
    float Kd = 0.1f;

    float error = energy_target - energy_now;

    float p_term = Kp * error;
    float d_term = Kd * (error - last_error);

    last_error = error;

    float dynamic_pmax = ref_power_max - p_term - d_term;

    //加上最低下限
    if (dynamic_pmax < MIN_MAXPOWER_CONFIGURED) {
        dynamic_pmax = MIN_MAXPOWER_CONFIGURED;
    }

    // 再加一个上限，比如极限能直接多加100w
    float max_power_limit = ref_power_max + 100.0f;
    if (dynamic_pmax > max_power_limit) {
        dynamic_pmax = max_power_limit;
    }

    return dynamic_pmax;
}

// ============================================================================
// 计算当前底盘的总功率
// ============================================================================
// 单个电机的功率 P = k0 + k1*I + k2*w + k3*I*w + k4*I^2 + k5*w^2
float power_caculate(int16_t motor_send_data[4][2], float RLS_argument[6]) {
    float total_power = 0.0f;

    float k0 = RLS_argument[0];
    float k1 = RLS_argument[1];
    float k2 = RLS_argument[2];
    float k3 = RLS_argument[3];
    float k4 = RLS_argument[4];
    float k5 = RLS_argument[5];

    for (int i = 0; i < 4; i++) {
        float I = (float)motor_send_data[i][0]; // [0] 是预计下发电流
        float w = (float)motor_send_data[i][1]; // [1] 是当前反馈角速度

        float p_i = k0 + k1 * I + k2 * w + k3 * I * w + k4 * I * I + k5 * w * w;
        total_power += p_i;
    }
    return total_power;
}

// ============================================================================
// 总调用接口：重分配逻辑
// ============================================================================
void reallocate(int16_t motor_send_data[4][2], float RLS_argument[6], int16_t reallocate_current[4], float power_max)
{
    float now_power = power_caculate(motor_send_data, RLS_argument);

    if (now_power <= power_max)
    {
        for(int i = 0; i < 4; i++){
            reallocate_current[i] = motor_send_data[i][0];
        }
    }
    else
    {
        float eta = current_caculate(motor_send_data, RLS_argument, power_max);
        for(int i = 0; i < 4; i++)
        {
            reallocate_current[i] = (int16_t)(motor_send_data[i][0] * eta);
        }
    }
}

// ============================================================================
// 核心：基于衰减电流法计算衰减系数 \eta
// ============================================================================
float current_caculate(int16_t motor_send_data[4][2], float RLS_argument[6], float power_max)
{
    float k0 = RLS_argument[0];
    float k1 = RLS_argument[1];
    float k2 = RLS_argument[2];
    float k3 = RLS_argument[3];
    float k4 = RLS_argument[4];
    float k5 = RLS_argument[5];

    // 构建关于 \eta 的一元二次方程: A * \eta^2 + B * \eta + C = 0
    float A = 0.0f;
    float B = 0.0f;
    float C = -power_max; // 将等式右边的 P_max 移到左边

    for (int i = 0; i < 4; i++) {
        float I = (float)motor_send_data[i][0];
        float w = (float)motor_send_data[i][1];

        // 根据公式推导累加 A, B, C 项
        A += k4 * I * I;
        B += (k1 * I) + (k3 * I * w);
        C += k0 + (k2 * w) + (k5 * w * w);
    }

    // 防御性编程：如果 A 极其接近 0，退化为一元一次方程 B * \eta + C = 0
    if (fabs(A) < 1e-6f) {
        if (fabs(B) > 1e-6f) {
            float eta = -C / B;
            return (eta >= 0.0f && eta <= 1.0f) ? eta : 0.0f;
        }
        return 0.0f;
    }

    // 计算判别式 \Delta
    float delta = B * B - 4.0f * A * C;

    // 1. 如果方程无解
    if (delta < 0.0f) {
        return 0.0f;
    }

    // 2. 如果方程有解，求解两个根
    float sqrt_delta = sqrtf(delta);
    float eta1 = (-B + sqrt_delta) / (2.0f * A);
    float eta2 = (-B - sqrt_delta) / (2.0f * A);

    // 检查解是否在合理区间 [0, 1] 内
    // (考虑到浮点数精度问题，稍微放宽一点点容忍度到 1.001)
    int valid_eta1 = (eta1 >= 0.0f && eta1 <= 1.001f);
    int valid_eta2 = (eta2 >= 0.0f && eta2 <= 1.001f);

    // 如果超出了 1.0 极小的一点点，强制截断为 1.0
    if (valid_eta1 && eta1 > 1.0f) eta1 = 1.0f;
    if (valid_eta2 && eta2 > 1.0f) eta2 = 1.0f;

    // 3. 根的筛选逻辑 (对应你的文档要求)
    if (valid_eta1 && valid_eta2) {
        // 两个都在 0~1 之间，取较大的那个 (保留最大的动力输出)
        return (eta1 > eta2) ? eta1 : eta2;
    } else if (valid_eta1) {
        // 只有一个有效
        return eta1;
    } else if (valid_eta2) {
        // 只有一个有效
        return eta2;
    } else {
        // 都不在 0~1 之间，不合理，给电机发 0
        return 0.0f;
    }
}