#include "kalman_filter.h"
#include <math.h>

void Kalman_Init(KalmanState* state, 
                float q_angle, float q_bias,
                float base_r, float alpha,
                float min_r, float max_r,
                float gamma)
{
    // 初始化过程噪声参数
    state->base_q_angle = q_angle;
    state->base_q_bias = q_bias;
    state->q_angle = q_angle;
    state->q_bias = q_bias;
    
    // 初始状态
    state->angle = 0.0f;
    state->bias = 0.0f;
    
    // 初始化协方差矩阵
    state->P[0][0] = 0.0f;
    state->P[0][1] = 0.0f;
    state->P[1][0] = 0.0f;
    state->P[1][1] = 0.0f;
    
    // 测量噪声参数
    state->base_r = base_r;
    state->alpha = alpha;
    state->min_r = min_r;
    state->max_r = max_r;
    state->r_measure = base_r;
    
    // 动态调整参数
    state->gamma = gamma;
    state->prev_y_sq = 0.0f;
}

float Kalman_Update(KalmanState* state, 
                   float new_angle, float new_rate,
                   float dt)
{
    // 使用上一次的残差调整过程噪声
    float q_scale = 1.0f + state->gamma * state->prev_y_sq;
    state->q_angle = state->base_q_angle * q_scale;
    state->q_bias = state->base_q_bias * q_scale;
    
    // 预测步骤
    const float rate = new_rate - state->bias;
    state->angle += dt * rate;
    
    // 更新协方差矩阵
    const float dtP11 = dt * state->P[1][1];
    state->P[0][0] += dt * (dt * state->P[1][1] - state->P[0][1] - state->P[1][0] + state->q_angle);
    state->P[0][1] -= dt * state->P[1][1];
    state->P[1][0] = state->P[0][1];
    state->P[1][1] += state->q_bias * dt;
    
    // 计算当前残差
    const float y = new_angle - state->angle;
    const float y_sq = y * y;
    state->prev_y_sq = y_sq; // 保存当前残差供下次使用
    
    // 动态调整测量噪声
    state->r_measure = state->base_r / (1.0f + state->alpha * y_sq);
    state->r_measure = fmaxf(state->min_r, fminf(state->r_measure, state->max_r));
    
    // 计算卡尔曼增益
    const float S = state->P[0][0] + state->r_measure;
    const float invS = 1.0f / S;
    const float K0 = state->P[0][0] * invS;
    const float K1 = state->P[1][0] * invS;
    
    // 更新状态
    state->angle += K0 * y;
    state->bias += K1 * y;
    
    // 更新协方差
    const float P00 = state->P[0][0];
    const float P01 = state->P[0][1];
    state->P[0][0] -= K0 * P00;
    state->P[0][1] -= K0 * P01;
    state->P[1][0] -= K1 * P00;
    state->P[1][1] -= K1 * P01;
    
    return state->angle;
}
