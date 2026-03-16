/**
 * @file gimbal_yaw_predictor.h
 * @brief 大云台 Yaw 轴预测跟踪控制器 - 提高响应速度和稳定性
 * @author QHJ
 * @version 1.0
 * @date 2026-03-06
 */

#ifndef GIMBAL_YAW_PREDICTOR_H
#define GIMBAL_YAW_PREDICTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 预测控制器结构体
 */
typedef struct
{
    // 状态变量
    float last_error;              ///< 上一次误差
    float error_integral;          ///< 误差积分
    float last_yaw_cmd;            ///< 上一次 yaw 指令
    float yaw_velocity;            ///< yaw 角速度估计
    
    // 预测参数
    float predict_time;            ///< 预测时间 (秒)
    float velocity_gain;           ///< 速度前馈增益
    float accel_gain;              ///< 加速度前馈增益
    
    // 自适应参数
    float adaptive_gain;           ///< 自适应增益系数
    float dead_zone;               ///< 死区阈值
    float max_output;              ///< 最大输出限幅
    
    // 滤波器
    float filtered_velocity;       ///< 滤波后的速度
    float velocity_filter_alpha;   ///< 速度滤波系数
    
    uint8_t initialized;           ///< 初始化标志
} YawPredictor_t;

/**
 * @brief 初始化 Yaw 预测控制器
 * @param predictor 预测器指针
 * @param dt 控制周期 (秒)
 * @param predict_time 预测时间 (秒)，推荐 0.03~0.06
 */
void YawPredictor_Init(YawPredictor_t *predictor, float dt, float predict_time);

/**
 * @brief 更新预测控制器并计算输出
 * @param predictor 预测器指针
 * @param yaw_error 当前 yaw 误差 (目标值 - 当前值)
 * @param yaw_cmd 原始 yaw 指令
 * @return 调整后的 Gimbal_T 增量
 * 
 * @note 输出公式：delta = Kp*error + Kv*velocity + Ka*accel + adaptive_gain
 */
float YawPredictor_Update(YawPredictor_t *predictor, float yaw_error, float yaw_cmd);

/**
 * @brief 重置预测器状态
 */
void YawPredictor_Reset(YawPredictor_t *predictor);

/**
 * @brief 设置自适应参数
 * @param predictor 预测器指针
 * @param dead_zone 死区阈值，一般为 0.05~0.15
 * @param max_output 最大输出限幅
 */
void YawPredictor_SetParams(YawPredictor_t *predictor, float dead_zone, float max_output);

/**
 * @brief 获取估计的角速度
 */
float YawPredictor_GetVelocity(YawPredictor_t *predictor);

#ifdef __cplusplus
}
#endif

#endif // GIMBAL_YAW_PREDICTOR_H
