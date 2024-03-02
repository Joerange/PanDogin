//
// Created by 19734wtf on 2023/3/27.
//

#ifndef DOG12_PID_H
#define DOG12_PID_H

#include "stdint.h"

//最大转速30rad/s(24V供电时)，最大力矩是23.7NM
#define SpeedMode_VERYEX    25.0f
#define SpeedMode_EXTREME   20.0f           //官网说最快转速是30rad/s,这里留有一定裕量，用于跳跃使用
#define SpeedMode_EARLYEX   15.0f
#define SpeedMode_VERYFAST  10.0f
#define SpeedMode_FAST      5.0f
#define SpeedMode_SLOW      3.1415926f
#define SpeedMode_VERYSLOW  2.0f

//PID结构体对象
typedef struct
{
    float Setpoint;

    float SumError;
    float SumError_limit;

    float P;
    float D;
    float I;

    float Last_error;
    float LLast_error;

    float Out_put;
    float Last_Out_put;
    float Output_limit;
}PIDTypeDef;

extern PIDTypeDef AngleLoop[9];
extern PIDTypeDef SpeedLoop[9];
extern PIDTypeDef VisualLoop;
extern uint8_t OnlyPosLoop;
extern PIDTypeDef Yaw_PID_Loop,Roll_PID_Loop,Pitch_PID_Loop;

void PID_Init(PIDTypeDef *pid);
void PID_Set_KP_KI_KD(PIDTypeDef *pid,float kp,float ki,float kd);
void SetPoint(PIDTypeDef *pid,float want,uint8_t id);
void SetPoint_Visual(PIDTypeDef *pid,float want);
void PID_PosLocCalc(PIDTypeDef *pid,int32_t feedbackpos);
void PID_IncCalc(PIDTypeDef *pid,int16_t feedbackspeed);
void ChangeGainOfPID(float pos_kp,float pos_kd,float sp_kp,float sp_ki);
void Eight_PID_Init(void );
void Change_speed_kp(float K_W);
void SetPoint_IMU(PIDTypeDef *pid,float want);
void ChangeYawOfPID(float Yaw_Kp,float Yaw_Kd,float error,float output_limit);

#endif //DOG12_PID_H
