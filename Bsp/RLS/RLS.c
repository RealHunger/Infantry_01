#include "RLS.h"

// ====================================================================
// 内部静态状态 (对外部调用者隐藏)
// ====================================================================
static double P[NUM_PARAMS][NUM_PARAMS];  // 协方差矩阵
static double current_theta[NUM_PARAMS] = {0.0}; // 当前参数的估计值
static int is_initialized = 0;            // 初始化标志位
static int history_idx = 0;               // 环形缓冲区的当前写入索引
static int valid_count = 0;               // 记录当前缓冲区里的有效数据量(最大1000)

// ====================================================================
// 接口函数实现
// ====================================================================
void RLS_Process(double input_x[2][4],
                 double y,
                 double history_theta[HISTORY_SIZE][NUM_PARAMS],
                 double output_avg[NUM_PARAMS])
{
    // ----------------------------------------------------------------
    // 1. 自动初始化 (仅在第一次调用时执行)
    // ----------------------------------------------------------------
    if (!is_initialized) {
        for (int i = 0; i < NUM_PARAMS; i++) {
            for (int j = 0; j < NUM_PARAMS; j++) {
                P[i][j] = (i == j) ? 1000.0 : 0.0;
            }
        }
        is_initialized = 1;
    }

    // ----------------------------------------------------------------
    // 2. 解析输入：将每个维度的 4 个元素求和得到 x1 和 x2
    // ----------------------------------------------------------------
    double x1 = 0.0;
    double x2 = 0.0;
    for (int i = 0; i < 4; i++) {
        x1 += input_x[0][i];
        x2 += input_x[1][i];
    }

    // 构建 6 维扩展输入向量
    double x[NUM_PARAMS] = { 1.0, x1, x2, x1*x1, x2*x2, x1*x2 };

    // ----------------------------------------------------------------
    // 3. RLS 核心递推更新计算 (高度优化版)
    // ----------------------------------------------------------------
    double lambda = 0.99;
    double inv_lambda = 1.0 / lambda;
    double Px[NUM_PARAMS] = {0.0};
    double xTPx = 0.0;

    for (int i = 0; i < NUM_PARAMS; i++) {
        double sum = 0.0;
        for (int j = 0; j < NUM_PARAMS; j++) {
            sum += P[i][j] * x[j];
        }
        Px[i] = sum;
        xTPx += x[i] * sum;
    }

    double inv_den = 1.0 / (lambda + xTPx);
    double K[NUM_PARAMS];
    double y_pred = 0.0;

    for (int i = 0; i < NUM_PARAMS; i++) {
        K[i] = Px[i] * inv_den;
        y_pred += x[i] * current_theta[i];
    }

    double e = y - y_pred; // 计算误差

    for (int i = 0; i < NUM_PARAMS; i++) {
        current_theta[i] += K[i] * e; // 更新参数向量

        for (int j = i; j < NUM_PARAMS; j++) {
            double new_P = (P[i][j] - K[i] * Px[j]) * inv_lambda;
            P[i][j] = new_P;
            P[j][i] = new_P; // 对称赋值
        }
    }

    // ----------------------------------------------------------------
    // 4. 更新历史记录 (环形缓冲区管理)
    // ----------------------------------------------------------------
    for (int i = 0; i < NUM_PARAMS; i++) {
        history_theta[history_idx][i] = current_theta[i];
    }

    // 索引往前推一格，如果超过 999 就回到 0 (覆盖最旧数据)
    history_idx = (history_idx + 1) % HISTORY_SIZE;

    // 有效计数器：不足 1000 时自增，达到 1000 后封顶
    if (valid_count < HISTORY_SIZE) {
        valid_count++;
    }

    // ----------------------------------------------------------------
    // 5. 计算所有有效历史记录的平均值并输出
    // ----------------------------------------------------------------
    for (int i = 0; i < NUM_PARAMS; i++) {
        double sum = 0.0;
        for (int j = 0; j < valid_count; j++) {
            sum += history_theta[j][i];
        }
        output_avg[i] = sum / valid_count;
    }
}