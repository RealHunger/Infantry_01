#include "power_reallocate.h"
#include <math.h>
#include "../Bsp/RLS/RLS.h"

// ============================================================================
// 私有静态状态 (对应用层完全隐藏)
// ============================================================================
static double rls_history_theta[HISTORY_SIZE][NUM_PARAMS] = {0};
static double rls_output_avg[NUM_PARAMS] = {0};

//最大功率能量环替换
static float energy_power_loop(float target, float now, float ref_max, float min_lim, float max_lim) {
    static float last_error = 0.0f;
    float Kp = 1.5f;
    float Kd = 0.1f;

    float error = target - now;
    float p_term = Kp * error;
    float d_term = Kd * (error - last_error);
    last_error = error;

    float dynamic_pmax = ref_max - p_term - d_term;

    if (dynamic_pmax < min_lim) dynamic_pmax = min_lim;
    if (dynamic_pmax > max_lim) dynamic_pmax = max_lim;

    return dynamic_pmax;
}

// ============================================================================
// 全局核心接口：将 RLS辨识、能量环、二次方程预测 完美串联
// ============================================================================
void Chassis_Power_Control_Loop(
    int16_t current_des[4][2], int16_t current_fb[4][2],
    float real_power_fb, float current_energy,
    const Power_Allocate_Config_t *config, int16_t safe_current_out[4])
{
    // ------------------------------------------------------------------------
    // 步骤一：提取聚合特征，进行 RLS 在线辨识 (闭环物理真实性保障)
    // ------------------------------------------------------------------------
    double rls_input[2][4];
    float sum_I_des = 0.0f;
    float sum_w_des = 0.0f;

    for (int i = 0; i < 4; i++) {
        // 供 RLS 使用上一帧真实状态
        rls_input[0][i] = (double)current_fb[i][0];
        rls_input[1][i] = (double)current_fb[i][1];

        // 供后续模型预测使用的期望聚合值
        sum_I_des += (float)current_des[i][0];
        sum_w_des += (float)current_des[i][1];
    }

    // 喂入真实功率，迭代辨识出的最新模型参数存储在 rls_output_avg 中
    RLS_Process(rls_input, (double)real_power_fb, rls_history_theta, rls_output_avg);

    // ------------------------------------------------------------------------
    // 步骤二：执行能量环，获取当前毫秒的战略级功率上限
    // ------------------------------------------------------------------------
    float p_max = energy_power_loop(
        config->target_energy, current_energy,
        config->referee_power_max, config->min_power_limit, config->max_power_limit
    );

    // ------------------------------------------------------------------------
    // 步骤三：利用 RLS 模型进行前向预测 (严格匹配 RLS.c 中的多项式特征)
    // 多项式: P = k0 + k1*ΣI + k2*Σw + k3*(ΣI)^2 + k4*(Σw)^2 + k5*(ΣI*Σw)
    // ------------------------------------------------------------------------
    float k0 = (float)rls_output_avg[0];
    float k1 = (float)rls_output_avg[1];
    float k2 = (float)rls_output_avg[2];
    float k3 = (float)rls_output_avg[3];
    float k4 = (float)rls_output_avg[4];
    float k5 = (float)rls_output_avg[5];

    // 计算如果不加任何干预，底盘将爆发的预测功率
    float P_origin = k0 + (k1 * sum_I_des) + (k2 * sum_w_des) +
                     (k3 * sum_I_des * sum_I_des) + (k4 * sum_w_des * sum_w_des) +
                     (k5 * sum_I_des * sum_w_des);

    // 如果预测未超额度，直接通行
    if (P_origin <= p_max) {
        for (int i = 0; i < 4; i++) safe_current_out[i] = current_des[i][0];
        return;
    }

    // ------------------------------------------------------------------------
    // 步骤四：超额拦截，构建并求解关于 eta 的一元二次方程
    // A*eta^2 + B*eta + C = 0
    // ------------------------------------------------------------------------
    float A = k3 * (sum_I_des * sum_I_des);
    float B = (k1 * sum_I_des) + (k5 * sum_I_des * sum_w_des);
    float C = k0 + (k2 * sum_w_des) + (k4 * sum_w_des * sum_w_des) - p_max;

    float eta = 0.0f;
    if (fabsf(A) < 1e-6f) {
        if (fabsf(B) > 1e-6f) eta = -C / B;
    } else {
        float delta = B * B - 4.0f * A * C;
        if (delta >= 0.0f) {
            float sqrt_delta = sqrtf(delta);
            float eta1 = (-B + sqrt_delta) / (2.0f * A);
            float eta2 = (-B - sqrt_delta) / (2.0f * A);

            int valid1 = (eta1 >= 0.0f && eta1 <= 1.0f);
            int valid2 = (eta2 >= 0.0f && eta2 <= 1.0f);

            if (valid1 && valid2) eta = (eta1 > eta2) ? eta1 : eta2;
            else if (valid1) eta = eta1;
            else if (valid2) eta = eta2;
        }
    }

    // 防御性安全越界保护
    if (eta > 1.0f) eta = 1.0f;
    if (eta < 0.0f) eta = 0.0f;

    // ------------------------------------------------------------------------
    // 步骤五：等比例扭矩衰减并输出
    // ------------------------------------------------------------------------
    for (int i = 0; i < 4; i++) {
        safe_current_out[i] = (int16_t)(current_des[i][0] * eta);
    }
}