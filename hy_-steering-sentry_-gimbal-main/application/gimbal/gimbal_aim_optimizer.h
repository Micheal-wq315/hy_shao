/**
 * @file gimbal_aim_optimizer.h
 * @brief 云台自瞄优化算法头文件
 * @author QHJ
 * @version 1.0
 * @date 2026-03-06
 */

#ifndef GIMBAL_AIM_OPTIMIZER_H
#define GIMBAL_AIM_OPTIMIZER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 一阶延迟滤波器 ==================== */

/**
 * @brief 一阶延迟滤波器结构体
 */
typedef struct
{
    float alpha;              /**< 滤波系数 (0.0~1.0) */
    float filtered_value;     /**< 滤波后的值 */
    float last_filtered_value;/**< 上一次滤波值 */
    uint8_t initialized;      /**< 初始化标志 */
} FirstOrderFilter_t;

/**
 * @brief 初始化一阶延迟滤波器
 */
void FirstOrderFilter_Init(FirstOrderFilter_t *filter, float alpha);

/**
 * @brief 更新一阶延迟滤波器
 */
float FirstOrderFilter_Update(FirstOrderFilter_t *filter, float input);

/**
 * @brief 重置一阶延迟滤波器
 */
void FirstOrderFilter_Reset(FirstOrderFilter_t *filter);

/* ==================== 滑动平均滤波器 ==================== */

#define MOVING_AVG_MAX_WINDOW 10  /**< 滑动平均最大窗口大小 */

/**
 * @brief 滑动平均滤波器结构体
 */
typedef struct
{
    uint8_t window_size;        /**< 窗口大小 */
    uint8_t index;              /**< 当前索引 */
    uint8_t count;              /**< 已填充数量 */
    float sum;                  /**< 累加和 */
    float buffer[MOVING_AVG_MAX_WINDOW];  /**< 数据缓冲区 */
} MovingAverageFilter_t;

/**
 * @brief 初始化滑动平均滤波器
 */
void MovingAverageFilter_Init(MovingAverageFilter_t *filter, uint8_t window_size);

/**
 * @brief 更新滑动平均滤波器
 */
float MovingAverageFilter_Update(MovingAverageFilter_t *filter, float input);

/**
 * @brief 重置滑动平均滤波器
 */
void MovingAverageFilter_Reset(MovingAverageFilter_t *filter);

/* ==================== 预测补偿器 ==================== */

/**
 * @brief 预测补偿器结构体
 */
typedef struct
{
    float dt;                   /**< 时间间隔 (秒) */
    float last_yaw;             /**< 上一次 yaw 值 */
    float last_pitch;           /**< 上一次 pitch 值 */
    float yaw_velocity;         /**< yaw 角速度 */
    float pitch_velocity;       /**< pitch 角速度 */
    float yaw_accel;            /**< yaw 角加速度 (预留) */
    float pitch_accel;          /**< pitch 角加速度 (预留) */
    uint8_t initialized;        /**< 初始化标志 */
} PredictionCompensator_t;

/**
 * @brief 初始化预测补偿器
 */
void PredictionCompensator_Init(PredictionCompensator_t *predictor, float dt);

/**
 * @brief 更新预测补偿器并计算补偿量
 */
void PredictionCompensator_Update(PredictionCompensator_t *predictor,
                                   float yaw, float pitch,
                                   float *compensated_yaw, float *compensated_pitch,
                                   float predict_time);

/**
 * @brief 重置预测补偿器
 */
void PredictionCompensator_Reset(PredictionCompensator_t *predictor);

/* ==================== 自适应增益控制器 ==================== */

/**
 * @brief 自适应增益控制器结构体
 */
typedef struct
{
    float base_gain;            /**< 基础增益 */
    float threshold;            /**< 误差阈值 */
    float current_gain;         /**< 当前增益 */
} AdaptiveGainController_t;

/**
 * @brief 初始化自适应增益控制器
 */
void AdaptiveGainController_Init(AdaptiveGainController_t *controller,
                                  float base_gain, float threshold);

/**
 * @brief 根据误差更新增益
 */
float AdaptiveGainController_Update(AdaptiveGainController_t *controller, float error);

/* ==================== 级联复合滤波器 ==================== */

/**
 * @brief 级联滤波器结构体 (一阶延迟 + 滑动平均)
 */
typedef struct
{
    FirstOrderFilter_t first_order;   /**< 一阶延迟滤波器 */
    MovingAverageFilter_t moving_avg; /**< 滑动平均滤波器 */
} CascadeFilter_t;

/**
 * @brief 初始化级联滤波器
 */
void CascadeFilter_Init(CascadeFilter_t *filter, float first_order_alpha, uint8_t ma_window_size);

/**
 * @brief 更新级联滤波器
 */
float CascadeFilter_Update(CascadeFilter_t *filter, float input);

/**
 * @brief 重置级联滤波器
 */
void CascadeFilter_Reset(CascadeFilter_t *filter);

/* ==================== 完整优化器 ==================== */

/**
 * @brief 完整的自瞄优化器结构体
 */
typedef struct
{
    CascadeFilter_t cascade_filter_yaw;      /**< yaw 轴级联滤波器 */
    CascadeFilter_t cascade_filter_pitch;    /**< pitch 轴级联滤波器 */
    PredictionCompensator_t predictor;        /**< 预测补偿器 */
    AdaptiveGainController_t gain_controller; /**< 自适应增益控制器 */
    float predict_time;                        /**< 预测时间 (秒) */
} AimOptimizer_t;

/**
 * @brief 初始化自瞄优化器
 */
void AimOptimizer_Init(AimOptimizer_t *optimizer, float dt, float predict_time);

/**
 * @brief 执行完整的优化处理流程
 */
void AimOptimizer_Process(AimOptimizer_t *optimizer,
                          float raw_yaw, float raw_pitch,
                          float *optimized_yaw, float *optimized_pitch);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_AIM_OPTIMIZER_H */
