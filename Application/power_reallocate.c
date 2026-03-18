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

// 检验是否断联
uint8_t verify_feedback_connection()
{
    return 1;
}

// 超级电容反馈的底盘实际功率
float real_power_feedback()
{
    return 0.0f;
}

// =====================对外接口=======================================================
void Chassis_Power_Control_Loop(
    int16_t current_des[4][2], int16_t current_fb[4][2],
    float real_power_fb, float current_energy,
    const Power_Allocate_Config_t *config, int16_t safe_current_out[4])
{

    // --------------------接收参数，用于接下来RLS参数更新-------------------------------
    double rls_input[2][4];
    float sum_I_des = 0.0f;
    float sum_w_des = 0.0f;

    for (int i = 0; i < 4; i++) {
        // 上一帧率电流以及转速记录传给rls
        rls_input[0][i] = (double)current_fb[i][0];
        rls_input[1][i] = (double)current_fb[i][1];

        // pid计算后的电流值加和用于接下来功率预测
        sum_I_des += (float)current_des[i][0];
        sum_w_des += (float)current_des[i][1];
    }

    // 调用更新RLS参数
    RLS_Process(rls_input, (double)real_power_fb, rls_history_theta, rls_output_avg);

    // ----------------------------能量环依据剩余能量以及目标能量等更新限制的最大输出功率--------------------------------------------
    float p_max = energy_power_loop(
        config->target_energy, current_energy,
        config->referee_power_max, config->min_power_limit, config->max_power_limit
    );

    // ---------------------------提取RLS参数拟合电机模型---------------------------------------------
    // 模型: P = k0 + k1*ΣI + k2*Σw + k3*(ΣI)**2 + k4*(Σw)**2 + k5*(ΣI*Σw)
    float k0 = (float)rls_output_avg[0];
    float k1 = (float)rls_output_avg[1];
    float k2 = (float)rls_output_avg[2];
    float k3 = (float)rls_output_avg[3];
    float k4 = (float)rls_output_avg[4];
    float k5 = (float)rls_output_avg[5];

    // pid计算结果直接下发
    float P_origin = k0 + (k1 * sum_I_des) + (k2 * sum_w_des) +
                     (k3 * sum_I_des * sum_I_des) + (k4 * sum_w_des * sum_w_des) +
                     (k5 * sum_I_des * sum_w_des);

    // 执行功率再分配
    if (P_origin <= p_max) {
        for (int i = 0; i < 4; i++) safe_current_out[i] = current_des[i][0];
        return;
    }


    // -------------------------功率超限，执行衰减电流法-----------------------------------------------
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

    // --------------------更新电流分配数值----------------------------------------------------
    for (int i = 0; i < 4; i++) {
        safe_current_out[i] = (int16_t)(current_des[i][0] * eta);
    }
}
