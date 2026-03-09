#ifndef GIMBAL_H
#define GIMBAL_H

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台IMU初始化,从惯导模块获取数据，传送给视觉
 * 
 */
void gimbal_IMU_Task(void);

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();

#endif // GIMBAL_H