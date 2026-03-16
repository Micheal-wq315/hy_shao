/*
 * @Description: 
 * @Author: 
 * @brief: 
 * @version: 
 * @Date: 2025-01-16 11:52:57
 * @LastEditors:  
 * @LastEditTime: 2025-01-17 17:27:31
 */
#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
#include "chassis.h"
#endif

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"
#include "dm_imu.h"
#endif

uint8_t freq_1k=0;
static uint32_t tick_ms=0;

void RobotInit()
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    
    BSPInit();
    imu_init(0x01,0x11,&hfdcan3);
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDInit();
    GimbalInit();
    ShootInit();
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisInit();
#endif

    OSTaskInit(); // 创建基础任务

    // 初始化完成,开启中断
    __enable_irq();
}

void RobotTask()
{
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDTask();
    GimbalTask();
    ShootTask();
    // DM-IMU任务
	if(freq_1k)
	{
		tick_ms++;
			
		if(tick_ms%3==0)
		{
			imu_request_accel();
		}
		else if(tick_ms%2==0)
		{
			imu_request_gyro();
		}
		else if(tick_ms%1==0)
		{
			imu_request_quat();
		}
		
		if(tick_ms>1000)
			tick_ms=0;
            
		freq_1k=0;
	}
    else{
        freq_1k=1;
    }
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisTask();
#endif

}