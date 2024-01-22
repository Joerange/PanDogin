//
// Created by 19734 on 2022/12/26.
//

#ifndef DOG12_MOTOR_H
#define DOG12_MOTOR_H

#include "stdint-gcc.h"

#define MOTOR_SEND_LENGTH      17         //发送给电机的控制命令是17个字节
#define MOTOR_ID_NUM           12
#define pi                     3.1415926535f
#define position               1
#define Zero_torque            2
#define MOTOR_RECEIVE_SIZE 16

extern int32_t began_pos[MOTOR_ID_NUM];                   //电机运动初始位置
extern int32_t end_pos[MOTOR_ID_NUM];                     //电机运动结束位置
extern int16_t real_speed[MOTOR_ID_NUM];
extern int8_t temperature[MOTOR_ID_NUM];
extern uint8_t merror[MOTOR_ID_NUM];
extern uint16_t rec_crc;
extern uint16_t uart1_rec_crc;
extern int8_t MOTOR_Send[MOTOR_ID_NUM][MOTOR_SEND_LENGTH];
extern float speed_kp;
extern uint8_t LeftLeg_ReceiverBuffer[MOTOR_RECEIVE_SIZE];
extern uint8_t RightLeg_ReceiverBuffer[MOTOR_RECEIVE_SIZE];

void modify_data(void);
void motor_test(void);
void motor_stop();
void MOTOR_Send_Init(void);
void motor_pos_controll(uint8_t id,float rad,uint8_t motor_mode);
void motor_speed_controll_with_kw(uint8_t id,float w,float kw);
void motor_torque_controll(uint8_t id, float torque);
void Encoder_calibration(uint8_t id);
void Set_Motor_Mode(uint8_t id,uint8_t mode);
void Motor_stop(uint8_t id);
void Get_motor_began_pos(void);
void Change_motor_PID(float k_p,float k_w);
void Get_motor_began_pos1(void);
void Get_motor_began_pos2(void);
void Get_motor_began_pos3(void);
void Get_motor_began_pos4(void);
void Pos_Controll(uint8_t id,int32_t theat_set,uint8_t motor_mode,float KP,float KW);
void AllLegsSpeedLimit(float speedlimit);
void no1_2_LegsSpeedLimit(float speedlimit);
void no3_4_LegsSpeedLimit(float speedlimit);
void leg_pos_controll(void );
void leg_pos_controll02(void );
void Speed_Controll(void );
void Speed_Controll02(void );
void AllLegsTorqueLimit(float torquelimit);
void UART_SendMessage(uint8_t id);
/**
 * @brief 电机模式控制信息
 *
 */
typedef struct
{
    uint8_t id     :4;      // 电机ID: 0,1...,14 15表示向所有电机广播数据(此时无返回)
    uint8_t status :3;      // 工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
    uint8_t none   :1;      // 保留位
} RIS_Mode_t;   // 控制模式 1Byte

/**
 * @brief 电机状态控制信息
 *
 */
typedef struct
{
    float tor_des;        // 期望关节输出扭矩 unit: N.m     (q8)
    float spd_des;        // 期望关节输出速度 unit: rad/s   (q8)
    float pos_des;        // 期望关节输出位置 unit: rad     (q15)
    float  k_p;        // 期望关节刚度系数 unit: 0.0-1.0 (q15)
    float  k_w;        // 期望关节阻尼系数 unit: 0.0-1.0 (q15)

} RIS_Comd_t;   // 控制参数 12Byte


/**
 * @brief 控制数据包格式
 *
 */
typedef struct
{
    uint8_t head[2];    // 包头         2Byte
    RIS_Mode_t mode;    // 电机控制模式  1Byte
    RIS_Comd_t comd;    // 电机期望数据 12Byte
    uint16_t   CRC16;   // CRC          2Byte

} ControlData_t;    // 主机控制命令     17Byte

#endif //DOG12_MOTOR_H
