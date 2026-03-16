#include "shoot.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
// static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机

// static servo_instance *lid; 需要增加弹舱盖

#define DETECTION_MAX_OUTPUT 4500.0f
#define SHOOT_RATE 44000.0f 

extern Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static ShootInstance shoot_l, shoot_r; // 左右发射机构实例
static Publisher_t *shoot_pub;
static Subscriber_t *vision_recv_r_data_sub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息
static Subscriber_t *vision_gimbal_sub;
static Vision_Gimbal_Data_s vision_gimbal_data_recv;
static Vision_Recv_s vision_recv_data_2_shoot_r;
static float output[2]; //存储拨弹电机的输出值
static int count[2] = {0, 0};       //用于堵转计数
static int enter_count[2]; //用于进入计数
// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;
static float detection_start = 0; // 用于检测是否进入堵转状态
static float detection_dt = 0; 

void ShootInit()
{
    // 左云台发射机构初始化摩擦轮
    Motor_Init_Config_s friction_Ll_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20, // 20
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508};
    friction_Ll_config.can_init_config.tx_id = 3,
    friction_Ll_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    shoot_l.friction_l = DJIMotorInit(&friction_Ll_config);

    friction_Ll_config.can_init_config.tx_id = 4; // 右摩擦轮,改txid和方向就行
    friction_Ll_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    // friction_Ll_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    shoot_l.friction_r = DJIMotorInit(&friction_Ll_config);


    // 右云台发射机构初始化摩擦轮
    Motor_Init_Config_s friction_Rl_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20, // 20
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M3508};
    friction_Rl_config.can_init_config.tx_id = 3,
    shoot_r.friction_l = DJIMotorInit(&friction_Rl_config);

    friction_Rl_config.can_init_config.tx_id = 4; // 右摩擦轮,改txid和方向就行
    friction_Rl_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    shoot_r.friction_r = DJIMotorInit(&friction_Rl_config);


    // 云台发射机构初始化拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 8,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 50, // 10
                .Ki = 10,
                .Kd = 0,
                .MaxOut = 500,
                .Improve = PID_Integral_Limit | PID_DerivativeFilter,
                .IntegralLimit = 20000,
            },
            .speed_PID = {
                .Kp = 30, // 20
                .Ki = 2, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit | PID_DerivativeFilter,
                .IntegralLimit = 5000,
                .MaxOut = 20000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit | PID_DerivativeFilter,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };

    loader_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    shoot_l.loader = DJIMotorInit(&loader_config);     // 左云台发射机构初始化拨盘电机
    loader_config.can_init_config.can_handle = &hcan2;
    loader_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    shoot_r.loader = DJIMotorInit(&loader_config);     // 右云台发射机构初始化拨盘电机

    shoot_l.stall_flag = 0; //初始化堵转标志位
    shoot_r.stall_flag = 0;

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    vision_gimbal_sub = SubRegister("vision_gimbal_data", sizeof(Vision_Gimbal_Data_s));
    vision_recv_r_data_sub = SubRegister("vision_recv_data", sizeof(Vision_Recv_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{   
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    SubGetMessage(vision_gimbal_sub, &vision_gimbal_data_recv);
    SubGetMessage(vision_recv_r_data_sub, &vision_recv_data_2_shoot_r);

    VisionBulletData(20,output[0]); //将弹速数据发送给master_process模块
    if(vision_gimbal_data_recv.vision_statue == GIMBAL_VISION)
    {
        shoot_cmd_recv.shoot_mode = SHOOT_ON;
        shoot_cmd_recv.friction_mode = FRICTION_ON;
        // if((vision_gimbal_data_recv.Vision_set_r_yaw - vision_gimbal_data_recv.yaw_r_motor_angle) < 1.0 && vision_recv_data_2_shoot.target_state == TRACKING 
        //  && (vision_gimbal_data_recv.Vision_set_r_pitch - vision_gimbal_data_recv.pitch_r_motor_angle) < 0.8)
        if((vision_gimbal_data_recv.Vision_set_r_yaw - vision_gimbal_data_recv.yaw_r_motor_angle) < 0.8 && vision_recv_data_2_shoot_r.target_state == TRACKING )
        {
            shoot_cmd_recv.load_mode = LOAD_BURSTFIRE;
        } 
    }
    // shoot_cmd_recv.load_mode = LOAD_BURSTFIRE;
    

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(shoot_l.friction_l);
        DJIMotorStop(shoot_l.friction_r);
        DJIMotorStop(shoot_r.friction_l);
        DJIMotorStop(shoot_r.friction_r);
        DJIMotorStop(shoot_l.loader);
        DJIMotorStop(shoot_r.loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(shoot_l.friction_l);
        DJIMotorEnable(shoot_l.friction_r);
        DJIMotorEnable(shoot_r.friction_l);
        DJIMotorEnable(shoot_r.friction_r);
        DJIMotorEnable(shoot_l.loader);
        DJIMotorEnable(shoot_r.loader);
    }
    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    //     return;

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
    // 停止拨盘
    case LOAD_STOP:
        DJIMotorOuterLoop(shoot_l.loader, SPEED_LOOP); // 切换到速度环
        DJIMotorOuterLoop(shoot_r.loader, SPEED_LOOP);
        DJIMotorSetRef(shoot_l.loader, 0); // 设定参考值为0,这样停止的速度最快
        DJIMotorSetRef(shoot_r.loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        break;
    // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET:                                                                     // 激活能量机关/干扰对方用,英雄用.
        DJIMotorOuterLoop(shoot_l.loader, ANGLE_LOOP);                                              // 切换到角度环
        DJIMotorOuterLoop(shoot_r.loader, ANGLE_LOOP); 
        DJIMotorSetRef(shoot_l.loader, shoot_l.loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
        DJIMotorSetRef(shoot_r.loader, shoot_r.loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
        hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
        dead_time = 150;                                                                    // 完成1发弹丸发射的时间
        break;
    // 三连发,如果不需要后续可能删除
    case LOAD_3_BULLET:
        DJIMotorOuterLoop(shoot_l.loader, ANGLE_LOOP);                                              // 切换到角度环
        DJIMotorOuterLoop(shoot_r.loader, ANGLE_LOOP); 
        DJIMotorSetRef(shoot_l.loader, shoot_l.loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE); // 增加3发
        DJIMotorSetRef(shoot_r.loader, shoot_r.loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE); // 增加3发
        hibernate_time = DWT_GetTimeline_ms();                                                  // 记录触发指令的时间
        dead_time = 300;                                                                        // 完成3发弹丸发射的时间
        break;
    // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    case LOAD_BURSTFIRE:
        if(shoot_l.stall_flag == 1 || shoot_r.stall_flag == 1)
        {
            if(shoot_l.stall_flag == 1) 
            {
                DJIMotorOuterLoop(shoot_l.loader, ANGLE_LOOP);                                              // 切换到角度环
                DJIMotorSetRef(shoot_l.loader, (shoot_l.loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE)); // 控制量减少一发弹丸的角度
                if(enter_count[0] == 20)
                {
                    shoot_l.stall_flag = 0;
                    enter_count[0] = 0;
                }
                
                enter_count[0]++;
            }
            if(shoot_r.stall_flag == 1)
            {
                DJIMotorOuterLoop(shoot_r.loader, ANGLE_LOOP);
                DJIMotorSetRef(shoot_r.loader, -(shoot_r.loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE)); // 控制量减少一发弹丸的角度
                if(enter_count[1] == 20)
                {
                    shoot_r.stall_flag = 0;
                    enter_count[1] = 0;
                }
                    enter_count[1]++;
            }
        }
        else
        {   DJIMotorOuterLoop(shoot_l.loader, SPEED_LOOP);                                              // 切换到速度环
            DJIMotorOuterLoop(shoot_r.loader, SPEED_LOOP);
            DJIMotorSetRef(shoot_l.loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8); // 设定速度
            DJIMotorSetRef(shoot_r.loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8); // 设定速度

        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
        }

        break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // 也有可能需要从switch-case中独立出来
    case LOAD_REVERSE:
        DJIMotorOuterLoop(shoot_l.loader, SPEED_LOOP);                                              // 切换到速度环
        DJIMotorOuterLoop(shoot_r.loader, SPEED_LOOP);
        // ...
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }
  
    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            DJIMotorSetRef(shoot_l.friction_l, 0);
            DJIMotorSetRef(shoot_l.friction_r, 0);
            DJIMotorSetRef(shoot_r.friction_l, 0);
            DJIMotorSetRef(shoot_r.friction_r, 0);
            break;
        case SMALL_AMU_18:
            DJIMotorSetRef(shoot_l.friction_l, 0);
            DJIMotorSetRef(shoot_l.friction_r, 0);
            DJIMotorSetRef(shoot_r.friction_l, 0);
            DJIMotorSetRef(shoot_r.friction_r, 0);
            break;
        case SMALL_AMU_30:
            DJIMotorSetRef(shoot_l.friction_l, 0);
            DJIMotorSetRef(shoot_l.friction_r, 0);
            DJIMotorSetRef(shoot_r.friction_l, 0);
            DJIMotorSetRef(shoot_r.friction_r, 0);
            break;
        default: // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
            DJIMotorSetRef(shoot_l.friction_l, SHOOT_RATE);  //44000
            DJIMotorSetRef(shoot_l.friction_r, SHOOT_RATE);
            DJIMotorSetRef(shoot_r.friction_l, SHOOT_RATE);  //44000
            DJIMotorSetRef(shoot_r.friction_r, SHOOT_RATE);
            
            break;
        }
    }
    else// 关闭摩擦轮
    {
        DJIMotorSetRef(shoot_l.friction_l, 0);
        DJIMotorSetRef(shoot_l.friction_r, 0);
        DJIMotorSetRef(shoot_r.friction_l, 0);
        DJIMotorSetRef(shoot_r.friction_r, 0);
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //... 
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}

/**
 * @brief 检测拨弹轮是否卡弹
 * 
 * 该函数通过检测拨弹轮的实际电流值来判断拨弹轮是否卡弹。
 * 如果电流值超过设定的最大值（DETECTION_MAX_OUTPUT），则增加计数器。
 * 如果电流值低于最大值，则减少计数器并限制其范围在0到10之间。
 * 当计数器超过100且电流值仍然超过最大值时，设置卡弹标志并重置计数器。
 */

void LoaderStallDetection()
{   
    // 获取当前时间戳
    detection_start = DWT_GetTimeline_ms();
    // 获取左拨弹轮的实际电流值
    output[0] = fabs(shoot_l.loader->measure.real_current);
    // 获取右拨弹轮的实际电流值
    output[1] = fabs(shoot_r.loader->measure.real_current);

    // 如果计数器小于等于0
    if(count[0] <= 0 || count[1] <= 0)
    {
        // 如果左拨弹轮计数器小于0，则增加计数器
        if(count[0] < 0)    count[0]++;
        // 如果右拨弹轮计数器小于0，则增加计数器
        if(count[1] < 0)    count[1]++;
    }

    // 如果左拨弹轮或右拨弹轮的电流值超过最大值
    if(output[0] >= DETECTION_MAX_OUTPUT || output[1] >= DETECTION_MAX_OUTPUT)
    {
        // 如果左拨弹轮的电流值超过最大值，则增加计数器
        if(output[0] >= DETECTION_MAX_OUTPUT)    count[0]++;
        // 如果右拨弹轮的电流值超过最大值，则增加计数器
        if(output[1] >= DETECTION_MAX_OUTPUT)    count[1]++;
    }

    // 如果左拨弹轮或右拨弹轮的电流值低于最大值
    if(output[0] < DETECTION_MAX_OUTPUT || output[1] < DETECTION_MAX_OUTPUT)
    {
        // 如果左拨弹轮的电流值低于最大值，则减少计数器并限制其范围在0到10之间
        if(output[0] < DETECTION_MAX_OUTPUT)
        {
            count[0]--;
            LIMIT_MIN_MAX(count[0], 0, 30);
        }
        // 如果右拨弹轮的电流值低于最大值，则减少计数器并限制其范围在0到10之间
        if(output[1] < DETECTION_MAX_OUTPUT)
        {
            count[1]--;
            LIMIT_MIN_MAX(count[1], 0, 30);
        }
    }

    // 如果计数器超过100且电流值仍然超过最大值
    if((count[0] > 30 || count[1] > 30) && (output[0] > DETECTION_MAX_OUTPUT || output[1] > DETECTION_MAX_OUTPUT))
    {
        // 如果左拨弹轮计数器超过100，则设置卡弹标志并重置计数器
        if(count[0] > 30)
        {
            shoot_l.stall_flag = 1;
            count[0] = 0;
        }
        // 如果右拨弹轮计数器超过100，则设置卡弹标志并重置计数器
        if(count[1] > 30)
        {
            shoot_r.stall_flag = 1;
            count[1] = 0;
        }
    }
}
