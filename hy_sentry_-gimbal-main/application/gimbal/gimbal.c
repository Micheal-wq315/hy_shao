    #include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "bsp_dwt.h"
#include "gimbal_aim_optimizer.h"
#include "gimbal_yaw_predictor.h"

#define YAW_L_INIT_ANGLE 0 // 云台初始角度
#define PITCH_L_INIT_ANGLE 100 // 云台初始俯仰角度   -117.0f

#define YAW_R_INIT_ANGLE 0 // 云台初始角度
#define PITCH_R_INIT_ANGLE 160.0f // 云台初始俯仰角度   -118.0f
#define PITCH_R_MIN 28 // 右云台经IMU测出下限时的pitch角度 25.3
#define PITCH_L_MIN 28

// 电机软件限位
#define YAW_L_LIMIT_MIN -76
#define YAW_L_LIMIT_MAX 20

#define PITCH_L_LIMIT_MIN 97
#define PITCH_L_LIMIT_MAX 130

#define YAW_R_LIMIT_MIN -110
#define YAW_R_LIMIT_MAX 60
#define PITCH_R_LIMIT_MIN 130
#define PITCH_R_LIMIT_MAX 160

#define YAW_COEFF_REMOTE 0.036363636f //云台遥控系数
#define PITCH_COEFF_REMOTE 0.134848485f //云台俯仰遥控系数
#define YAW_VISION_OFFSET 12

static attitude_t *gimbal_IMU_data; // 云台IMU数据
static attitude_T *Gimbal_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_l_motor, *yaw_r_motor, *pitch_l_motor, *pitch_r_motor; // 云台电机实例
static DMMotorInstance  *Gimbal_Base;
static float vision_l_yaw_tar;
static float vision_l_pitch_tar;             
static float Gimbal_T = 0.0f; // 云台扫描周期
static float GimbalDirection =0.05f; // 云台扫描方向 
static int time_t = 0; // 计时  

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Publisher_t *vision_gimbal_pub;            //云台视觉信息
static Subscriber_t *vision_recv_data_sub_l, *vision_recv_data_sub_r;
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static Vision_Gimbal_Data_s vision_gimbal_data; // 自瞄时云台数据(为方便计算，定义了相对角度)
static Vision_Recv_s vision_recv_data_l, vision_recv_data_r;
static float Yaw_single_angle, Yaw_angle_sum;

/*
    自瞄滤波算法变量实现
*/
static AimOptimizer_t aim_optimizer;
static float optimized_yaw, optimized_pitch;

/*
    Yaw 预测控制器变量
*/
static YawPredictor_t yaw_predictor;  // ← 新增：Yaw 轴预测控制器实例

static void DMMotor_Enable_Can(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
    memset(motor->motor_can_instace->tx_buff, 0, 8);    //清空
}

void GimbalInit()
{   
    float gimbal_base_angle_feed_ptr = gimbal_IMU_data->YawTotalAngle;

    Gimbal_IMU_data = INS_ptr();

    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 100, // Me:30
                .Ki = 60, 
                .Kd = 0,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 100,
                .Output_LPF_RC=0.00005,
                .MaxOut = 2000,
            },
            .speed_PID = {
                .Kp = 35,  // 50
                .Ki = 50, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement |PID_OutputFilter,
                .Output_LPF_RC=0.00649999983,
                .IntegralLimit = 2000,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 100, //  Me:20
                .Ki = 20 , // 3
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2000, // 30
                .MaxOut = 800,
            },
            .speed_PID = {
                .Kp = 40,  // 15    空载k = 10  Me: 46
                .Ki = 20, // 500
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .Output_LPF_RC=0.00649999983,
                .IntegralLimit = 8500,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimbal_IMU_data->Gyro[0]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };  
 
    Motor_Init_Config_s DMmotor_Motor_Config = {
    .controller_setting_init_config.angle_feedback_source = OTHER_FEED,
    .controller_setting_init_config.speed_feedback_source = MOTOR_FEED,
    .controller_setting_init_config.close_loop_type = ANGLE_LOOP,
    .controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
    .controller_setting_init_config.feedforward_flag = SPEED_FEEDFORWARD,
    .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    .controller_setting_init_config.outer_loop_type = ANGLE_LOOP | SPEED_LOOP,
    .can_init_config.can_handle = &hcan1,
    // .can_init_config.can_module_callback = &DMMotorLostCallback,
    // .can_init_config.id = (void *)0x00,
    .can_init_config.rx_id = 0x10,
    .can_init_config.tx_id = 0x20F,


    .controller_param_init_config = {
        .angle_PID = {
            .Kp = 0.09, // 0.06
            .Ki = 0.05, // 0.05
            .Kd = 0,
            .MaxOut = 30,   // 30
            .IntegralLimit = 10,
            .Output_LPF_RC = 0.00649999983,
            .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter
        },

        .speed_PID = {
            .Kp = 5,
            .Ki = 0.7,
            .Kd = 0,
            .MaxOut = 30,   // 50
            .IntegralLimit = 15,
            .IntegralLimit = PID_Integral_Limit | PID_Derivative_On_Measurement
        },
        .other_angle_feedback_ptr = &Gimbal_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
        .other_speed_feedback_ptr = &Gimbal_IMU_data->Gyro[0],
        .speed_feedforward_ptr = &Gimbal_IMU_data->Gyro[0],
    }
};

    /*
        抬头参数
    */
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_l_motor = DJIMotorInit(&yaw_config);
    // yaw_config.can_init_config.can_handle = &hcan2;
    // yaw_r_motor = DJIMotorInit(&yaw_config);

    pitch_l_motor = DJIMotorInit(&pitch_config);
    // pitch_config.can_init_config.can_handle = &hcan2;
    // pitch_r_motor = DJIMotorInit(&pitch_config);
 
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    vision_gimbal_pub = PubRegister("vision_gimbal_data",sizeof(Vision_Gimbal_Data_s));
    vision_recv_data_sub_l = SubRegister("vision_recv_l_data", sizeof(Vision_Recv_s));
    vision_recv_data_sub_r = SubRegister("vision_recv_r_data", sizeof(Vision_Recv_s));

    gimbal_IMU_data = INS_Init(); 
    Gimbal_IMU_data = INS_Init(); 

    Gimbal_Base = DMMotorInit(&DMmotor_Motor_Config);

    // ========== 新增：初始化 Yaw 预测控制器 ==========
    // 参数：dt=5ms (200Hz 控制周期), 预测时间=40ms (补偿系统延迟)
    YawPredictor_Init(&yaw_predictor, 0.005f, 0.040f);
    
    // 自瞄优化器初始化 (需要添加到 Makefile 后才能使用)
    AimOptimizer_Init(&aim_optimizer, 0.005f, 0.040f);  // dt=5ms, 预测时间=40ms
}


/**
 * @brief 母云台达妙电机总角度计算
 * @return 
 */

static void YawAngleCalculate()
{
    static float angle, last_angle, temp, rad_sum;
    angle = Gimbal_Base->measure.position; // 从云台获取的当前yaw电机角度
    
    if(fabs(angle - last_angle) > 0.0001)
    {
        if((angle - last_angle) < -12.5)
        {
            rad_sum += 25 + angle - last_angle;
        }
        else if((angle - last_angle) >  12.5)
        {
            rad_sum += 25 + last_angle - angle;
        }
        else
        {
            rad_sum += angle - last_angle;
        }
    }
    Yaw_angle_sum = rad_sum * RAD_2_DEGREE;
    temp = fmodf(Yaw_angle_sum, 360.0);
    Yaw_single_angle = fabs(temp);
    Gimbal_Base->measure.single_angle = Yaw_single_angle;
    Gimbal_Base->measure.total_angle = Yaw_angle_sum;
    last_angle = angle;
}

/**
 * @brief ins数据获取,从惯导模块获取数据，传送给视觉
 * @return
 */
void gimbal_IMU_Task(){
    // float *q = INS_Q();
    // gimbal_IMU_data->q[0] = q[0];
    // gimbal_IMU_data->q[1] = q[1];
    // gimbal_IMU_data->q[2] = q[2];
    // gimbal_IMU_data->q[3] = q[3];

    // float *Accel = INS_ACCEL();
    // gimbal_IMU_data->Accel[0] = Accel[0];
    // gimbal_IMU_data->Accel[1] = Accel[1];   
    // gimbal_IMU_data->Accel[2] = Accel[2];

    // float *Gyro = INS_GYRO();
    // gimbal_IMU_data->Gyro[0] = Gyro[0]; 
    // gimbal_IMU_data->Gyro[1] = Gyro[1];
    // gimbal_IMU_data->Gyro[2] = Gyro[2];

    // // gimbal_IMU_data->YawTotalAngle = INS_YawTotalAngle();
    // // gimbal_IMU_data->Roll = INS_Roll();
    // // gimbal_IMU_data->Pitch = INS_Pitch();
    // // gimbal_IMU_data->Yaw = INS_Yaw();
    // gimbal_IMU_data->YawTotalAngle = Gimbal_IMU_data->YawTotalAngle;
    // gimbal_IMU_data->Roll = Gimbal_IMU_data->Roll;
    // gimbal_IMU_data->Pitch = Gimbal_IMU_data->Pitch;
    // gimbal_IMU_data->Yaw = Gimbal_IMU_data->Yaw;
}

/**
 * @brief 角度计算
 */

static void VisionAngleCalc()
{   
    // vision_gimbal_data.Vision_l_yaw = yaw_l_motor->measure.total_angle - YAW_L_INIT_ANGLE;
    // vision_gimbal_data.Vision_l_pitch = pitch_l_motor->measure.total_angle - PITCH_L_INIT_ANGLE;

    // // vision_gimbal_data.Vision_r_yaw = gimbal_IMU_data->Yaw + yaw_r_motor->measure.total_angle - YAW_R_INIT_ANGLE;
    // vision_gimbal_data.Vision_set_l_yaw = gimbal_cmd_recv.yaw + YAW_L_INIT_ANGLE;
    // vision_gimbal_data.Vision_set_l_pitch = gimbal_cmd_recv.pitch + PITCH_L_INIT_ANGLE - PITCH_L_MIN;

    // vision_gimbal_data.Vision_set_r_yaw = vision_gimbal_data.Vision_r_yaw_tar + YAW_R_INIT_ANGLE - gimbal_IMU_data->Yaw;
    // vision_gimbal_data.Vision_set_r_yaw = vision_gimbal_data.Vision_r_yaw_tar + YAW_R_INIT_ANGLE;
    // vision_gimbal_data.Vision_set_r_pitch = vision_gimbal_data.Vision_r_pitch_tar + PITCH_R_INIT_ANGLE - PITCH_R_MIN;

    // vision_gimbal_data.Vision_r_yaw = yaw_r_motor->measure.total_angle - YAW_R_INIT_ANGLE;
    // vision_gimbal_data.Vision_r_pitch = pitch_r_motor->measure.total_angle  - PITCH_R_INIT_ANGLE;
}

static float temp_statue;

static void YawTrackingControl()
{
    // ========== 使用预测控制器替代原有逻辑 ==========
    
    // 1. 计算 yaw 误差（目标角度即为误差），需要实装测量来确定正负
    float yaw_error = gimbal_cmd_recv.yaw;
    
    // 2. 使用预测控制器计算 Gimbal_T 增量
    float delta_gimbal_T = YawPredictor_Update(&yaw_predictor, 
                                               yaw_error, 
                                               gimbal_cmd_recv.yaw);

    // 3. 更新 Gimbal_T
    Gimbal_T += delta_gimbal_T;

    // ============ 原有逻辑注释 ==============
    /*
    //方案一，大 yaw 响应会相对慢一些
    if(abs(gimbal_cmd_recv.yaw)>0.15)
    {
        if(gimbal_cmd_recv.yaw>0) 
        {
            Gimbal_T += GimbalDirection;
        }
        else
        {
            Gimbal_T -= GimbalDirection;
        }
    }

    //方案二，大 yaw 响应快一些，但会导致震荡现象
    if(gimbal_cmd_recv.yaw>0)
    {
        Gimbal_T += GimbalDirection*(0.3-abs(gimbal_cmd_recv.yaw))*(0.3-abs(gimbal_cmd_recv.yaw))/0.09;
    }
    else
    {
        Gimbal_T -= GimbalDirection*(0.3-abs(gimbal_cmd_recv.yaw))*(0.3-abs(gimbal_cmd_recv.yaw))/0.09;
    }
    */
}

/** 
 * @brief 自瞄优化版本
 * 使用说明：
 * 1. 在 Makefile 中添加 application/gimbal/gimbal_aim_optimizer.c
 * 2. 取消 gimbal.c中相关代码的注释
 * 3. 详细文档参考：gimbal_aim_optimizer.md
 */

static void GimbalSessionStart()
{
    static float time, last_time, diff_time, time_T, sint, cnt;
    static float tracking_start_time = 0;  // 记录开始检测到目标的时间
    static uint8_t confirmed_tracking = 0;  // 确认的跟踪状态标志

    time = DWT_GetTimeline_ms();
    time_T = DWT_GetTimeline_s();
    sint = arm_sin_f32(time_T);
    
    if(vision_recv_data_r.target_state == NO_TARGET)
    {
        // 重置跟踪状态和时间
        confirmed_tracking = 0;
        tracking_start_time = 0;
        
        diff_time += time - last_time;
        if(diff_time > 500)
        {   
            //Gimbal_T += GimbalDirection;
            //vision_l_yaw_tar = ((YAW_L_LIMIT_MIN + YAW_L_LIMIT_MAX) / 2) + (((YAW_L_LIMIT_MAX - YAW_L_LIMIT_MIN) / 2) * arm_sin_f32(time_T));
            // vision_l_yaw_tar = YAW_L_LIMIT_MIN + (YAW_L_LIMIT_MAX -  YAW_L_LIMIT_MIN) / 2;
            // vision_l_pitch_tar = ((PITCH_L_LIMIT_MIN + PITCH_L_LIMIT_MAX) / 2) + (((PITCH_L_LIMIT_MAX - PITCH_L_LIMIT_MIN) / 2) * arm_sin_f32(time_T * 5));
            vision_l_yaw_tar =-32.5f;
            vision_l_pitch_tar = 110.0f;
        }
    }
    else{//自瞄逻辑实现
        // 检测目标状态变化，只有持续 500ms 确认为 TRACKING 才执行自瞄
        if(vision_recv_data_r.target_state == TRACKING)
        {
            // 第一次检测到目标，记录时间
            if(tracking_start_time == 0)
            {
                tracking_start_time = time;
            }
            
            // 检查是否已经持续 500ms
            if((time - tracking_start_time) >= 500)
            {
                confirmed_tracking = 1;  // 确认进入跟踪状态
            }
        }
        else
        {
            // 如果是其他状态（如 READY_TO_FIRE），重置计时
            tracking_start_time = 0;
            confirmed_tracking = 0;
        }
        
        // 只有在确认的跟踪状态下才执行自瞄逻辑
        if(confirmed_tracking)
        {
            // vision_l_yaw_tar = yaw_l_motor->measure.total_angle + gimbal_cmd_recv.yaw*RAD_2_DEGREE*0.2 + gimbal_cmd_recv.yaw_vel*14*(-1);
            // vision_l_pitch_tar = pitch_l_motor->measure.total_angle + (gimbal_cmd_recv.pitch+0.125)*18*(-1);

            // 1. 准备原始输入数据
            float raw_yaw_input = gimbal_cmd_recv.yaw * RAD_2_DEGREE *0.3; // 将输入转换为角度误差
            float raw_pitch_input = (gimbal_cmd_recv.pitch + 0.125f) * 20 *(-1.0f);

            // 2. 执行优化处理（滤波 + 预测）
            AimOptimizer_Process(&aim_optimizer, 
                                raw_yaw_input, raw_pitch_input,
                                &optimized_yaw, &optimized_pitch);

            // 3. 使用优化后的数据计算目标角度
            // vision_l_yaw_tar = yaw_l_motor->measure.total_angle 
            //                   + optimized_yaw * 0.2f
            //                   + gimbal_cmd_recv.yaw_vel * 14.0f * (-1.0f);

            vision_l_yaw_tar = yaw_l_motor->measure.total_angle 
                              + optimized_yaw * 0.3f;

            vision_l_pitch_tar = pitch_l_motor->measure.total_angle 
                                + optimized_pitch *0.5f;
        }
        // 如果还未确认跟踪状态，保持当前的 vision_l_yaw_tar 和 vision_l_pitch_tar 值不变
    }
    if(vision_recv_data_r.target_state == TRACKING)
    {  
        diff_time = 0;
    }
    gimbal_cmd_recv.gimbal_angle = Gimbal_T;
    last_time = time;
}

/**
 * @brief 右头未识别到时扫描
 * @return  
 */

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{   
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    SubGetMessage(vision_recv_data_sub_l, &vision_recv_data_l);
    SubGetMessage(vision_recv_data_sub_r, &vision_recv_data_r);
    
    //为了避免Gimbal_Base在云台上电时失能，每次都要判断一下，如果是失能状态就使能它，保持云台IMU数据的更新，保证云台的稳定性和响应速度
    // if(Gimbal_Base->stop_flag == MOTOR_STOP)
    // {
    //     Gimbal_Base->stop_flag = MOTOR_ENALBED;
    //     DJIMotorEnable(Gimbal_Base);
    // }
    yaw_l_motor->stop_flag = MOTOR_ENALBED;
    time_t++;
    if(time_t%500 == 0)
    {
        DMMotor_Enable_Can(DM_CMD_MOTOR_MODE, Gimbal_Base);
    }

    gimbal_IMU_Task();
    // YawAngleCalculate();
    temp_statue = gimbal_cmd_recv.gimbal_mode;
    if(gimbal_cmd_recv.gimbal_mode == GIMBAL_VISION)
    {   
        vision_gimbal_data.Vision_l_yaw_tar = gimbal_cmd_recv.yaw;
        vision_gimbal_data.Vision_l_pitch_tar = gimbal_cmd_recv.pitch;
        vision_gimbal_data.yaw_r_motor_angle = yaw_l_motor->measure.total_angle;
        vision_gimbal_data.pitch_r_motor_angle = pitch_l_motor->measure.total_angle;

        // vision_gimbal_data.Vision_r_yaw_tar = gimbal_cmd_recv.yaw;
        // vision_gimbal_data.Vision_r_pitch_tar = gimbal_cmd_recv.pitch;
        // vision_gimbal_data.yaw_r_motor_angle = yaw_r_motor->measure.total_angle;
        // vision_gimbal_data.pitch_r_motor_angle = pitch_r_motor->measure.total_angle;

        vision_gimbal_data.vision_statue = GIMBAL_VISION;
    }
    else
    {
        vision_gimbal_data.vision_statue = temp_statue;
    }

    GimbalSessionStart();

    VisionSetAltitude(vision_gimbal_data.Vision_r_yaw * DEGREE_2_RAD, vision_gimbal_data.Vision_r_pitch * DEGREE_2_RAD, 0);
    VisionSetImuData(gimbal_IMU_data->q[0], gimbal_IMU_data->q[1], gimbal_IMU_data->q[2], gimbal_IMU_data->q[3], Gimbal_IMU_data->Gyro[2], Gimbal_IMU_data->Gyro[0]);
    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_l_motor);
        DJIMotorStop(pitch_l_motor);
        DJIMotorStop(yaw_r_motor);
        DJIMotorStop(pitch_r_motor);

        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        // DJIMotorEnable(yaw_l_motor);
        // DJIMotorEnable(pitch_l_motor);
        // DJIMotorChangeFeed(yaw_l_motor, ANGLE_LOOP, OTHER_FEED);
        // DJIMotorChangeFeed(yaw_r_motor, ANGLE_LOOP, OTHER_FEED);
        // DJIMotorChangeFeed(pitch_l_motor, ANGLE_LOOP, OTHER_FEED);
        // DJIMotorChangeFeed(pitch_r_motor, ANGLE_LOOP, OTHER_FEED);
        // DJIMotorSetRef(yaw_l_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        // DJIMotorSetRef(pitch_l_motor, gimbal_cmd_recv.pitch);
        DJIMotorStop(yaw_l_motor);
        DJIMotorStop(pitch_l_motor);
        DJIMotorStop(yaw_r_motor);
        DJIMotorStop(pitch_r_motor);
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        // DJIMotorEnable(yaw_l_motor);
        // DJIMotorEnable(pitch_l_motor);
        // DJIMotorEnable(yaw_r_motor);
        // DJIMotorEnable(pitch_r_motor);
        // DJIMotorChangeFeed(yaw_l_motor, ANGLE_LOOP, MOTOR_FEED);
        // DJIMotorChangeFeed(yaw_r_motor, ANGLE_LOOP, MOTOR_FEED);
        // DJIMotorChangeFeed(pitch_l_motor, ANGLE_LOOP, MOTOR_FEED);
        // DJIMotorChangeFeed(pitch_r_motor, ANGLE_LOOP, MOTOR_FEED);
        // // DJIMotorSetRef(yaw_l_motor, -gimbal_cmd_recv.yaw + YAW_L_INIT_ANGLE); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        // // DJIMotorSetRef(pitch_l_motor, pitch_r_angle);
        // DJIMotorSetRef(yaw_r_motor, -gimbal_cmd_recv.yaw + YAW_R_INIT_ANGLE);
        // DJIMotorSetRef(pitch_r_motor, -gimbal_cmd_recv.pitch + PITCH_R_INIT_SET_ANGLE - 5.0);
        DJIMotorStop(yaw_l_motor);
        DJIMotorStop(pitch_l_motor);
        DJIMotorStop(yaw_r_motor);
        DJIMotorStop(pitch_r_motor);
        break;
    // 云台自瞄模式，自瞄计算使用相对母云台角度，发送时转换为实际角度
    case GIMBAL_VISION: 
        DJIMotorEnable(yaw_l_motor);
        DJIMotorEnable(pitch_l_motor);
        DJIMotorEnable(yaw_r_motor);   
        DJIMotorEnable(pitch_r_motor);
        DMMotorEnable(Gimbal_Base);
        DJIMotorChangeFeed(yaw_l_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(yaw_r_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(pitch_l_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(pitch_r_motor, ANGLE_LOOP, MOTOR_FEED);

        LIMIT_MIN_MAX(vision_l_yaw_tar, YAW_L_LIMIT_MIN, YAW_L_LIMIT_MAX);
        LIMIT_MIN_MAX(vision_l_pitch_tar, PITCH_L_LIMIT_MIN, PITCH_L_LIMIT_MAX);

        DJIMotorSetRef(yaw_l_motor, vision_l_yaw_tar);
        DJIMotorSetRef(pitch_l_motor, vision_l_pitch_tar);
        DMMotorSetRef (Gimbal_Base, gimbal_cmd_recv.gimbal_angle);

    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_l_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
    PubPushMessage(vision_gimbal_pub,(void *)&vision_gimbal_data);
}
  