#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
static const float LQR_K[2] = {0.045f, 0.001f}; // 角度增益K1、角速度增益K2
 
#define DM_PID_MAX 16
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
    memset(motor->motor_can_instace->tx_buff, 0, 8);    //清空
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
}

static void DMMotorLostCallback(void *motor_ptr)
{
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
    
    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    motor->speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    motor->current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    DMMotorCaliEncoder(motor);
    DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if(loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    if(loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
}

//使用lqr控制器替换原有pid控制器,保持接口不变
void DMMotorTask(void const *argument)
{
    float pid_measure, pid_ref, set;
    DMMotorInstance *motor = (DMMotorInstance *)argument;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    static uint8_t *vbuf;

    while (1)
    {
        pid_ref = motor->pid_ref;

        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;
       
        // ========== 仅替换这段PID逻辑为LQR ==========
        // 原PID代码
        // if((setting->close_loop_type & ANGLE_LOOP) && (setting->outer_loop_type & ANGLE_LOOP))
        // {
        //     if(setting->angle_feedback_source == OTHER_FEED)
        //     {
        //         pid_measure = *motor->other_angle_feedback_ptr;
        //         motor->measure.other_angle = pid_measure;
        //     }
        //     else
        //         pid_measure = motor->measure.total_angle;

        //     pid_ref = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
            
        // }

        // 新增LQR计算（仅几行，最小改动）
        float theta = *motor->other_angle_feedback_ptr; // Yaw角度（IMU）
        float omega = *motor->other_speed_feedback_ptr * 57.2958f; // Gyro[0] rad/s → °/s
        float theta_err = pid_ref - theta; // 角度误差
        float omega_err = 0 - omega;      // 角速度误差
        pid_ref = LQR_K[0] * theta_err + LQR_K[1] * omega_err; // LQR核心计算
        // ==========================================

        // 速度前馈,目前加上前馈没有明显效果,后续可以调整增益或者改进前馈算法
        // if ((setting->feedforward_flag & SPEED_FEEDFORWARD) && (motor->speed_feedforward_ptr != NULL)) {
        //     float speed_feed = *motor->speed_feedforward_ptr;
        //     if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE) {
        //         speed_feed *= -1;
        //      }
        //     static float speed_feed_filtered = 0.0f;
        //     speed_feed_filtered = 0.9f * speed_feed_filtered + 0.1f * speed_feed;
        //     pid_ref -= speed_feed_filtered; 
        // }

        set = pid_ref*(-1); 
        if(abs(set) > DM_PID_MAX)
            set = DM_PID_MAX * (set/abs(set));
        
        if(motor->stop_flag == MOTOR_STOP) 
            set = 0;

        // 保留原有速度模式CAN帧（不改为MIT模式，最小改动）
        vbuf = (uint8_t *)&set;
        motor->motor_can_instace->tx_buff[0] = *vbuf;
        motor->motor_can_instace->tx_buff[1] = *(vbuf + 1);
        motor->motor_can_instace->tx_buff[2] = *(vbuf + 2);
        motor->motor_can_instace->tx_buff[3] = *(vbuf + 3);

        CANTransmit(motor->motor_can_instace, 1);
        osDelay(2);
    }
}

//@Todo: 目前只实现了力控，更多位控PID等请自行添加
// void DMMotorTask(void const *argument)
// {
//     float  pid_measure, pid_ref, set;
//     DMMotorInstance *motor = (DMMotorInstance *)argument;
//    //DM_Motor_Measure_s *measure = &motor->measure;
//     Motor_Control_Setting_s *setting = &motor->motor_settings;
//     //CANInstance *motor_can = motor->motor_can_instace;
//     //uint16_t tmp;
//     DMMotor_Send_s motor_send_mailbox;
//     while (1)
//     {   
//         pid_ref = motor->pid_ref;

//         if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
//             pid_ref *= -1;
       
//         LIMIT_MIN_MAX(set, DM_V_MIN, DM_V_MAX);
//         // motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
//         // // motor_send_mailbox.velocity_des = float_to_uint(pid_ref, DM_V_MIN, DM_V_MAX, 16);
//         // motor_send_mailbox.velocity_des = 0;
//         // motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
//         // motor_send_mailbox.Kp = 0;
//         // motor_send_mailbox.Kd = 0;

//             // motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 16);

//         /* MIT模式控制帧 */
//         // motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
//         // motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
//         // motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
//         // motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
//         // motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
//         // motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
//         // motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
//         // motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

//         /*位置环计算*/
//         if((setting->close_loop_type & ANGLE_LOOP) && (setting->outer_loop_type & ANGLE_LOOP))
//         {
//             if(setting->angle_feedback_source == OTHER_FEED)
//             {
//                 pid_measure = *motor->other_angle_feedback_ptr;
//                 motor->measure.other_angle = pid_measure;
//             }
//             else
//                 pid_measure = motor->measure.total_angle;

//             pid_ref = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
            
//         }

//     // 速度前馈（启用且指针有效时）
//     if ((setting->feedforward_flag & SPEED_FEEDFORWARD) && (motor->speed_feedforward_ptr != NULL)) {
//         float speed_feed = *motor->speed_feedforward_ptr;
    
//         // 电机反转时前馈同步反转
//         if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE) {
//             speed_feed *= -1;
//          }

//         // 前馈滤波
//         static float speed_feed_filtered = 0.0f;
//         speed_feed_filtered = 0.95f * speed_feed_filtered + 0.05f * speed_feed; // 一阶低通滤波，可调整系数
    
//          // 叠加滤波后的前馈
//         pid_ref -= speed_feed_filtered; 
//     }

//         set = pid_ref*(-1); // 方向反转
//         if(abs(set) > DM_PID_MAX)
//             set = DM_PID_MAX * (set/abs(set));
        
//         if(motor->stop_flag == MOTOR_STOP)
//             set = 0;
 
//         /* 速度模式控制帧 */
//         static uint8_t *vbuf;
//         vbuf = (uint8_t *)&set;

//         motor->motor_can_instace->tx_buff[0] = *vbuf;
//         motor->motor_can_instace->tx_buff[1] = *(vbuf + 1);
//         motor->motor_can_instace->tx_buff[2] = *(vbuf + 2);
//         motor->motor_can_instace->tx_buff[3] = *(vbuf + 3);

//         CANTransmit(motor->motor_can_instace, 1);

//         osDelay(2);
//     }
// }

void DMMotorControlInit()
{
    char dm_task_name[5] = "dm";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char dm_id_buff[2] = {0};
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);
        osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
    }
}
