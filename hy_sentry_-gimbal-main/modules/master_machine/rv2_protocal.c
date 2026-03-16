/*
 * @Description: 
 * @Author: changfeng
 * @brief: 
 * @version: 
 * @Date: 2025-02-14 13:28:09
 * @LastEditors:  
 * @LastEditTime: 2025-02-16 16:53:29
 */
//
// Created by 26090 on 25-1-13.
//

#include "rv2_protocal.h"

#include <bsp_log.h>
#include <crc16.h>
#include <string.h>

static rv2_recv_protocol_s rv2_recv_data={0};
static rv2_recv_protocol_TongJi_s rv2_recv_data_TongJi={0};

rv2_recv_protocol_s *rv2_protocol_init(void)
{
    memset(&rv2_recv_data,0,sizeof(rv2_recv_data));
    memset(&rv2_recv_data_TongJi,0,sizeof(rv2_recv_data_TongJi));
    return &rv2_recv_data;
}

void build_rv2_send_data(Vision_Send_s *send,uint8_t *tx_buf,uint16_t *tx_buf_len)
{
    // static rv2_send_protocol_s rv2_send_data={
    //     .header = RV2_PROTOCAL_HEADER,
    //     .reserved = 0x00,
    //     .aim_x = 0,
    //     .aim_y = 0,
    //     .aim_z = 0};

    static rv2_send_protocol_TongJi rv2_send_data={
        .header = RV2_PROTOCAL_HEADER,
        .mode = 1,
    };
    // // 姿态部分
    // rv2_send_data.roll=send->roll;
    rv2_send_data.pitch=send->pitch;
    rv2_send_data.yaw=send->yaw;

    rv2_send_data.pitch_vel=send->pitch_vel;
    rv2_send_data.yaw_vel=send->yaw_vel;

    rv2_send_data.bullet_speed=20;

    rv2_send_data.q[0]=send->q[0];
    rv2_send_data.q[1]=send->q[1];
    rv2_send_data.q[2]=send->q[2];
    rv2_send_data.q[3]=send->q[3];
    // rv2_send_data.pitch=send->pitch;
    // rv2_send_data.yaw=send->yaw;
    //对局信息部分

    // rv2_send_data.reset_tracker=0;

    //CRC校验
    rv2_send_data.crc_16=crc_16((uint8_t *)&rv2_send_data,sizeof(rv2_send_data)-2);

    memcpy(tx_buf,&rv2_send_data,sizeof(rv2_send_data));
    *tx_buf_len=sizeof(rv2_send_data);
}


void parse_rv2_receive_data(Vision_Recv_s *receive, uint8_t *rx_buf, uint16_t rx_buf_len)
{
    //包头校验
    if(rx_buf[0]==0xA5)
    {
        //CRC校验
        uint16_t checksum=crc_16(rx_buf,rx_buf_len-2);
    //    if(rx_buf[rx_buf_len-1]==((checksum&0xFF00)>>8)&&(rx_buf[rx_buf_len-2]==(checksum&0x00FF)))
        if(1) //暂时不校验crc
        {
            //(&rv2_recv_data,rx_buf,sizeof(rv2_recv_data));
           memcpy(&rv2_recv_data_TongJi,rx_buf,sizeof(rv2_recv_data_TongJi));
           receive->yaw=rv2_recv_data_TongJi.yaw;
           receive->pitch=rv2_recv_data_TongJi.pitch;
           receive->yaw_vel=rv2_recv_data_TongJi.yaw_vel;
           receive->pitch_vel=rv2_recv_data_TongJi.pitch_vel;
           //根据接收到的数据设置状态和模式
           if(rv2_recv_data_TongJi.target_state==1)
           {
                receive->target_state = TRACKING;
                receive->fire_mode = AUTO_AIM;
            }
            else if(rv2_recv_data_TongJi.target_state==0)
            {
                receive->target_state = NO_TARGET;
                receive->fire_mode = NO_FIRE;
            }
            else if(rv2_recv_data_TongJi.target_state==2){
                receive->target_state = READY_TO_FIRE;
                receive->fire_mode = AUTO_FIRE;
            }
            receive->target_type = rv2_recv_data_TongJi.target_type;

        //    receive->offline=0;
        //    receive->target_state=rv2_recv_data_TongJi.tracking;
            // receive->target_state=rv2_recv_data.tracking;
            //判别识别号码和装甲板数量，属于为了兼容的转换性设置 
            // switch (rv2_recv_data.id) {
            //     case 0:
            //         receive->target_type = OUTPOST;
            //         break;
            //        case 1:
            //     case 2:
            //     case 3:
            //     case 4:
            //     case 5:
            //         receive->target_type = rv2_recv_data.id;
            //         break;
            //     case 6:
            //         receive->target_type = SENTRY;
            //         break;
            //     case 7:
            //         receive->target_type = BASE;
            //         break;
            //     default:
            //         break;
            // }
        }
        else
        {
            LOGERROR("RV2 Receive+ checksum error");
            return;
        }
    }
    else
    {
        receive->target_state=NO_TARGET;
        LOGERROR("RV2 Receive Header error");
        return;
    }
}
