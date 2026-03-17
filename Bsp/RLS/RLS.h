#ifndef RLS_H
#define RLS_H

#define HISTORY_SIZE 1000
#define NUM_PARAMS 6 // 二元二次函数的 6 个参数: 常数, x1, x2, x1^2, x2^2, x1*x2

/**
 * @brief RLS 极简调用接口
 * * @param input_x       [输入] 二元数组 [2][4]，分别求和得到 x1 和 x2
 * @param y             [输入] 当前时刻的真实观测值，用于误差校正
 * @param history_theta [输出/记录] 记录历史参数的二元数组 [1000][6]
 * @param output_avg    [输出] 各个参数在历史记录中的平均值 [6]
 */
void RLS_Process(double input_x[2][4],
                 double y,
                 double history_theta[HISTORY_SIZE][NUM_PARAMS],
                 double output_avg[NUM_PARAMS]);

#endif // RLS_H