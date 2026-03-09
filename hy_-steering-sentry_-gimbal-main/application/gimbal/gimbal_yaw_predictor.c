/**
 * @file gimbal_yaw_predictor.c
 * @brief 大云台 Yaw 轴预测跟踪控制器实现
 * @author QHJ
 * @version 1.0
 * @date 2026-03-06
 */

#include "gimbal_yaw_predictor.h"
#include "arm_math.h"
#include "math.h"

/* ==================== Yaw 预测控制器实现 ==================== */

/**
 * @brief 初始化 Yaw 预测控制器
 */
void YawPredictor_Init(YawPredictor_t *predictor, float dt, float predict_time)
{
    // 状态变量清零
    predictor->last_error = 0.0f;
    predictor->error_integral = 0.0f;
    predictor->last_yaw_cmd = 0.0f;
    predictor->yaw_velocity = 0.0f;
    
    // 预测参数设置
    predictor->predict_time = predict_time;
    predictor->velocity_gain = 1.5f;      // 速度前馈增益 (根据实验调整)
    predictor->accel_gain = 0.5f;         // 加速度前馈增益 (预留)
    
    // 自适应参数
    predictor->adaptive_gain = 1.0f;
    predictor->dead_zone = 0.08f;         // 死区阈值，默认 0.08
    predictor->max_output = 0.15f;        // 最大输出限幅
    
    // 滤波器参数
    predictor->filtered_velocity = 0.0f;
    predictor->velocity_filter_alpha = 0.3f;  // 速度滤波系数
    
    predictor->initialized = 1;
}

/**
 * @brief 更新预测控制器并计算输出
 */
float YawPredictor_Update(YawPredictor_t *predictor, float yaw_error, float yaw_cmd)
{
    if (!predictor->initialized)
    {
        return 0.0f;
    }

    // ========== Step 1: 计算误差变化率 (速度估计) ==========
    float error_derivative = (yaw_error - predictor->last_error);
    
    // 一阶低通滤波去除噪声
    predictor->yaw_velocity = predictor->velocity_filter_alpha * error_derivative 
                            + (1.0f - predictor->velocity_filter_alpha) * predictor->yaw_velocity;
    
    // ========== Step 2: 计算预测位置 ==========
    // 基于当前速度和预测时间，估算未来位置
    float predicted_error = yaw_error + predictor->yaw_velocity * predictor->predict_time;
    
    // ========== Step 3: 自适应增益计算 ==========
    float abs_error = fabsf(predicted_error);
    
    if (abs_error > predictor->dead_zone)
    {
        // 大误差：提高增益，加快响应
        predictor->adaptive_gain = 1.0f + (abs_error - predictor->dead_zone) / predictor->dead_zone;
        
        // 限制最大增益
        if (predictor->adaptive_gain > 3.0f)
        {
            predictor->adaptive_gain = 3.0f;
        }
    }
    else
    {
        // 小误差：降低增益，防止震荡
        predictor->adaptive_gain = 0.5f + 0.5f * abs_error / predictor->dead_zone;
    }
    
    // ========== Step 4: 计算控制输出 ==========
    // 比例项：基于预测误差
    float proportional = predicted_error * predictor->adaptive_gain;
    
    // 速度前馈项：提前补偿运动
    float feedforward = predictor->yaw_velocity * predictor->velocity_gain;
    
    // 总输出
    float output = proportional + feedforward;
    
    // ========== Step 5: 输出限幅 ==========
    if (output > predictor->max_output)
    {
        output = predictor->max_output;
    }
    else if (output < -predictor->max_output)
    {
        output = -predictor->max_output;
    }
    
    // ========== Step 6: 更新状态变量 ==========
    predictor->last_error = yaw_error;
    predictor->last_yaw_cmd = yaw_cmd;
    
    return output;
}

/**
 * @brief 重置预测器状态到初始值
 * 
 * 将所有状态变量清零，用于在系统重启、模式切换或异常情况下
 * 重置预测器的内部状态，确保控制器从初始状态开始工作
 * 
 * @param predictor Yaw 预测控制器实例指针
 *        - last_error: 清零，重置历史误差
 *        - error_integral: 清零，重置积分累积
 *        - last_yaw_cmd: 清零，重置上次指令
 *        - yaw_velocity: 清零，重置角速度估计
 *        - filtered_velocity: 清零，重置滤波后速度
 *        - adaptive_gain: 恢复到默认增益 1.0
 */
void YawPredictor_Reset(YawPredictor_t *predictor)
{
    predictor->last_error = 0.0f;
    predictor->error_integral = 0.0f;
    predictor->last_yaw_cmd = 0.0f;
    predictor->yaw_velocity = 0.0f;
    predictor->filtered_velocity = 0.0f;
    predictor->adaptive_gain = 1.0f;
}

/**
 * @brief 设置自适应参数
 */
void YawPredictor_SetParams(YawPredictor_t *predictor, float dead_zone, float max_output)
{
    predictor->dead_zone = dead_zone;
    predictor->max_output = max_output;
}

/**
 * @brief 获取估计的角速度
 */
float YawPredictor_GetVelocity(YawPredictor_t *predictor)
{
    return predictor->yaw_velocity;
}
