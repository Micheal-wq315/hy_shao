#ifndef SHOOT_H
#define SHOOT_H

#include "dji_motor.h"

/**
 * @brief: 双发射机构shoot实例 
 * @return {*}
 */
typedef struct 
{
    DJIMotorInstance *friction_l;
    DJIMotorInstance *friction_r;
    DJIMotorInstance *loader;
    int stall_flag;

} ShootInstance;




/**
 * @brief 发射初始化,会被RobotInit()调用
 * 
 */
void ShootInit();

/**
 * @brief 发射任务
 * 
 */
void ShootTask();

/**
 * @brief 拨弹轮堵转检测函数
 * 
 * @return 
 */
void LoaderStallDetection();

#endif // SHOOT_H