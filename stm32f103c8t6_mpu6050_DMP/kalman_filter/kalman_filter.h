#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // 基础过程噪声参数
    float base_q_angle;
    float base_q_bias;
    float q_angle;
    float q_bias;
    
    // 状态估计
    float angle;
    float bias;
    
    // 协方差矩阵
    float P[2][2];
    
    // 自适应测量噪声参数
    float base_r;
    float alpha;
    float min_r;
    float max_r;
    float r_measure;
    
    // 动态过程噪声调整参数
    float gamma;
    float prev_y_sq; // 上一次的残差平方
} KalmanState;

void Kalman_Init(KalmanState* state, 
                float q_angle, float q_bias,
                float base_r, float alpha,
                float min_r, float max_r,
                float gamma);

float Kalman_Update(KalmanState* state, 
                   float new_angle, float new_rate,
                   float dt);

#ifdef __cplusplus
}
#endif

#endif // KALMAN_FILTER_H
