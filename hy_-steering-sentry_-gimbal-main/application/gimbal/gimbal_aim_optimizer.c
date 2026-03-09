/**
 * @file gimbal_aim_optimizer.c
 * @brief 云台自瞄优化算法 - 提高响应速度和稳定性
 * @author QHJ
 * @version 1.0
 * @date 2026-03-06
 */

#include "gimbal_aim_optimizer.h"
#include "arm_math.h"
#include "math.h"
#include "string.h"

/* ==================== 一阶延迟滤波（低通滤波）实现 ==================== */

/**
 * @brief 初始化一阶延迟滤波器
 * @param filter 滤波器结构体指针，指向待初始化的滤波器对象
 * @param alpha 滤波系数 (0.0~1.0)，决定滤波器的响应特性
 *              - 值越小：滤波效果越强，信号更平滑，但延迟越大
 *              - 值越大：响应越快，跟踪更及时，但滤波效果越弱
 *              推荐值：yaw 轴 0.3~0.5，pitch 轴 0.4~0.6
 */
void FirstOrderFilter_Init(FirstOrderFilter_t *filter, float alpha)
{
    /** 
     * 设置滤波系数，该参数决定了输入信号与历史输出信号的权重比例
     * 计算公式：y[n] = alpha * x[n] + (1-alpha) * y[n-1]
     */
    filter->alpha = alpha;
    
    /** 初始化当前滤波值为 0，等待第一次有效输入 */
    filter->filtered_value = 0.0f;
    
    /** 初始化上一次滤波值为 0，作为递推公式的初始状态 */
    filter->last_filtered_value = 0.0f;
    
    /** 标记滤波器尚未完成首次初始化，首次更新时需特殊处理 */
    filter->initialized = 0;
}

/**
 * @brief 一阶延迟滤波更新
 * @param filter 滤波器结构体指针
 * @param input 输入值
 * @return 滤波后的值
 * 
 * @note 公式：y[n] = alpha * x[n] + (1-alpha) * y[n-1]
 */
float FirstOrderFilter_Update(FirstOrderFilter_t *filter, float input)
{
    if (!filter->initialized)
    {
        filter->filtered_value = input;
        filter->last_filtered_value = input;
        filter->initialized = 1;
        return input;
    }
    
    filter->filtered_value = filter->alpha * input + (1.0f - filter->alpha) * filter->last_filtered_value;
    filter->last_filtered_value = filter->filtered_value;
    
    return filter->filtered_value;
}

/**
 * @brief 重置滤波器状态
 */
void FirstOrderFilter_Reset(FirstOrderFilter_t *filter)
{
    filter->initialized = 0;
    filter->filtered_value = 0.0f;
    filter->last_filtered_value = 0.0f;
}

/* ==================== 滑动平均滤波实现 ==================== */

/**
 * @brief 初始化滑动平均滤波器
 * @param filter 滤波器结构体指针
 * @param window_size 滑动窗口大小 (推荐 3~5)
 */
void MovingAverageFilter_Init(MovingAverageFilter_t *filter, uint8_t window_size)
{
    filter->window_size = window_size;
    filter->index = 0;
    filter->sum = 0.0f;
    filter->count = 0;
    memset(filter->buffer, 0, sizeof(filter->buffer));
}

/**
 * @brief 滑动平均滤波更新
 * @param filter 滤波器结构体指针
 * @param input 输入值
 * @return 滤波后的值
 */
float MovingAverageFilter_Update(MovingAverageFilter_t *filter, float input)
{
    filter->sum -= filter->buffer[filter->index];
    filter->buffer[filter->index] = input;
    filter->sum += input;
    
    filter->index++;
    if (filter->index >= filter->window_size)
    {
        filter->index = 0;
    }
    
    if (filter->count < filter->window_size)
    {
        filter->count++;
    }
    
    return filter->sum / filter->count;
}

/**
 * @brief 重置滤波器状态
 */
void MovingAverageFilter_Reset(MovingAverageFilter_t *filter)
{
    filter->index = 0;
    filter->sum = 0.0f;
    filter->count = 0;
    memset(filter->buffer, 0, sizeof(filter->buffer));
}

/* ==================== 预测补偿算法实现 ==================== */

/**
 * @brief 初始化预测补偿器
 * @param predictor 预测器结构体指针
 * @param dt 时间间隔 (秒)，通常为控制周期
 */
void PredictionCompensator_Init(PredictionCompensator_t *predictor, float dt)
{
    predictor->dt = dt;
    predictor->last_yaw = 0.0f;
    predictor->last_pitch = 0.0f;
    predictor->yaw_velocity = 0.0f;
    predictor->pitch_velocity = 0.0f;
    predictor->yaw_accel = 0.0f;
    predictor->pitch_accel = 0.0f;
    predictor->initialized = 0;
}

/**
 * @brief 更新预测模型并计算补偿量
 * @param predictor 预测器结构体指针
 * @param yaw 当前 yaw 角
 * @param pitch 当前 pitch 角
 * @param compensated_yaw 补偿后的 yaw 角输出
 * @param compensated_pitch 补偿后的 pitch 角输出
 * @param predict_time 预测时间 (秒)，根据弹丸飞行时间确定
 */
void PredictionCompensator_Update(PredictionCompensator_t *predictor, 
                                   float yaw, float pitch,
                                   float *compensated_yaw, float *compensated_pitch,
                                   float predict_time)
{
    if (!predictor->initialized)
    {
        predictor->last_yaw = yaw;
        predictor->last_pitch = pitch;
        predictor->initialized = 1;
        *compensated_yaw = yaw;
        *compensated_pitch = pitch;
        return;
    }
    
    // 计算速度 (一阶差分)
    predictor->yaw_velocity = (yaw - predictor->last_yaw) / predictor->dt;
    predictor->pitch_velocity = (pitch - predictor->last_pitch) / predictor->dt;
    
    // 简单的一阶预测：position = current_position + velocity * time
    *compensated_yaw = yaw + predictor->yaw_velocity * predict_time;
    *compensated_pitch = pitch + predictor->pitch_velocity * predict_time;
    
    // 保存上一次数据
    predictor->last_yaw = yaw;
    predictor->last_pitch = pitch;
}

/**
 * @brief 重置预测器状态
 */
void PredictionCompensator_Reset(PredictionCompensator_t *predictor)
{
    predictor->initialized = 0;
    predictor->yaw_velocity = 0.0f;
    predictor->pitch_velocity = 0.0f;
    predictor->yaw_accel = 0.0f;
    predictor->pitch_accel = 0.0f;
}

/* ==================== 自适应增益控制器实现 ==================== */

/**
 * @brief 初始化自适应增益控制器
 * @param controller 控制器结构体指针
 * @param base_gain 基础增益系数
 * @param threshold 误差阈值
 */
void AdaptiveGainController_Init(AdaptiveGainController_t *controller, 
                                  float base_gain, float threshold)
{
    controller->base_gain = base_gain;
    controller->threshold = threshold;
    controller->current_gain = base_gain;
}

/**
 * @brief 根据误差大小动态调整增益
 * @param controller 控制器结构体指针
 * @param error 当前误差
 * @return 调整后的增益
 * 
 * @note 大误差时使用高增益提高响应速度，小误差时降低增益防止超调
 */
float AdaptiveGainController_Update(AdaptiveGainController_t *controller, float error)
{
    float abs_error = fabsf(error);
    
    if (abs_error > controller->threshold)
    {
        // 大误差，提高增益以加快响应
        controller->current_gain = controller->base_gain * (1.0f + (abs_error - controller->threshold) / controller->threshold);
    }
    else
    {
        // 小误差，降低增益以提高稳定性
        controller->current_gain = controller->base_gain * (0.5f + 0.5f * abs_error / controller->threshold);
    }
    
    // 限制增益范围
    if (controller->current_gain < controller->base_gain * 0.3f)
    {
        controller->current_gain = controller->base_gain * 0.3f;
    }
    else if (controller->current_gain > controller->base_gain * 3.0f)
    {
        controller->current_gain = controller->base_gain * 3.0f;
    }
    
    return controller->current_gain;
}

/* ==================== 复合滤波器（级联）实现 ==================== */

/**
 * @brief 初始化复合滤波器
 * @param filter 复合滤波器结构体指针
 * @param first_order_alpha 一阶滤波系数
 * @param ma_window_size 滑动平均窗口大小
 */
void CascadeFilter_Init(CascadeFilter_t *filter, float first_order_alpha, uint8_t ma_window_size)
{
    FirstOrderFilter_Init(&filter->first_order, first_order_alpha);
    MovingAverageFilter_Init(&filter->moving_avg, ma_window_size);
}

/**
 * @brief 级联滤波更新 (先一阶延迟，再滑动平均)
 * @param filter 复合滤波器结构体指针
 * @param input 输入值
 * @return 滤波后的值
 */
float CascadeFilter_Update(CascadeFilter_t *filter, float input)
{
    float temp = FirstOrderFilter_Update(&filter->first_order, input);
    return MovingAverageFilter_Update(&filter->moving_avg, temp);
}

/**
 * @brief 重置复合滤波器
 */
void CascadeFilter_Reset(CascadeFilter_t *filter)
{
    FirstOrderFilter_Reset(&filter->first_order);
    MovingAverageFilter_Reset(&filter->moving_avg);
}

/* ==================== 应用示例函数 ==================== */

/**
 * @brief 完整的自瞄优化处理流程
 * @param optimizer 优化器实例指针
 * @param raw_yaw 原始 yaw 输入
 * @param raw_pitch 原始 pitch 输入
 * @param optimized_yaw 优化后的 yaw 输出
 * @param optimized_pitch 优化后的 pitch 输出
 * 
 * @note 此函数整合了所有优化算法，可直接在 GimbalSessionStart() 中调用
 */
void AimOptimizer_Process(AimOptimizer_t *optimizer,
                          float raw_yaw, float raw_pitch,
                          float *optimized_yaw, float *optimized_pitch)
{
    // Step 1: 滤波处理，去除噪声
    float filtered_yaw = CascadeFilter_Update(&optimizer->cascade_filter_yaw, raw_yaw);
    float filtered_pitch = CascadeFilter_Update(&optimizer->cascade_filter_pitch, raw_pitch);
    
    // Step 2: 运动预测，补偿延迟
    float predicted_yaw, predicted_pitch;
    PredictionCompensator_Update(&optimizer->predictor, 
                                 filtered_yaw, filtered_pitch,
                                 &predicted_yaw, &predicted_pitch,
                                 optimizer->predict_time);
    
    // Step 3: 自适应增益调整 (可选，用于手动遥控模式)
    // float gain = AdaptiveGainController_Update(&optimizer->gain_controller, predicted_yaw);
    
    *optimized_yaw = predicted_yaw;
    *optimized_pitch = predicted_pitch;
}

/**
 * @brief 初始化完整的自瞄优化器
 * @param optimizer 优化器实例指针
 * @param dt 控制周期 (秒)
 * @param predict_time 预测时间 (秒)
 */
void AimOptimizer_Init(AimOptimizer_t *optimizer, float dt, float predict_time)
{
    // 初始化滤波器
    // yaw 轴噪声较大，使用较强的滤波
    CascadeFilter_Init(&optimizer->cascade_filter_yaw, 0.4f, 3);
    // pitch 轴相对稳定，使用较弱的滤波
    CascadeFilter_Init(&optimizer->cascade_filter_pitch, 0.5f, 3);
    
    // 初始化预测器
    PredictionCompensator_Init(&optimizer->predictor, dt);
    
    // 初始化自适应增益控制器
    AdaptiveGainController_Init(&optimizer->gain_controller, 1.0f, 5.0f);
    
    optimizer->predict_time = predict_time;
}
