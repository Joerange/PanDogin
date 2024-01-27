//
// Created by 1 on 2023-11-07.
//
#include "Attitude_Task.h"

float TargetAngle1 = 0,TargetAngle2 = 0;
enum GPStates gpstate = STOP;
enum DPStates dpstate = NONE;
float NewHeartbeat = 0;//心跳值
//全局姿态控制
int Global_IMU_Control = 0;
uint8_t Mark_flag = 0;

void StandUp_Posture(void)
{
    AllLegsSpeedLimit(SpeedMode_FAST);
    Get_Target(0,PI);
    SetCoupledThetaPositionAll();
}

void LieDown_Posture(void)
{
    AllLegsSpeedLimit(SpeedMode_VERYSLOW);
    for(int i = 1;i < 9;i ++)
    {
        AngleWant_MotorX[i] = 0;
    }
}
void MarkingTime(void)
{
    Mark_flag = 1;
    AllLegsSpeedLimit(SpeedMode_FAST);
    ChangeGainOfPID(3.8f,0,0.6f,0);
//    ChangeYawOfPID(50.0f,0.5f,2500.0f,10.0f);
//    YawControl(yawwant, &state_detached_params[2], 1.0f);
    gait_detached(state_detached_params[2],0.0f, 0.5f, 0.5f, 0.0f,
                  1.0f,1.0f,1.0f,1.0f);
}
//实际运行Trot步态
void Trot(float direction,int8_t kind)
{
    switch(kind)
    {
        case 0://小步Trot
            AllLegsSpeedLimit(SpeedMode_VERYFAST);
            NewHeartbeat = 6;
            ChangeGainOfPID(10.0f,0,0.6f,0);
            ChangeYawOfPID(200.0f,2.0f,3000.0f,10.0f);
            YawControl(yawwant, &state_detached_params[4], direction);
            gait_detached(state_detached_params[4],0.0f, 0.5f, 0.5f, 0.0f,
                          direction,direction,direction,direction);
            break;
        case 1://大步Trot
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            NewHeartbeat = 4;
//            Change_SinStateDetachedParams(&state_detached_params[1],1,1,15.0f,64,14.9f,13.5f,0.32f,4.0f);
            ChangeGainOfPID(17.0f,1.0f,0.6f,0);
            ChangeYawOfPID(5000.0f,10.0f,4000.0f,15.0f);
            YawControl(yawwant, &state_detached_params[1], direction);
            gait_detached(state_detached_params[1],0.0f, 0.5f, 0.5f, 0.0f,
                          direction,direction,direction,direction);
            break;
        default:
            break;
    }
}

//慢步
void Walk(float direction,uint8_t speed)
{
    NewHeartbeat = 4;
    AllLegsSpeedLimit(SpeedMode_FAST);
    ChangeGainOfPID(3.5f,0,0.6f,0);
    ChangeYawOfPID(100.0f,0.5f,2500.0f,10.0f);
    YawControl(yawwant, &state_detached_params[3], direction);
    gait_detached(state_detached_params[3],0.0,0.75,0.5,0.25,direction,direction,direction,direction);
}
//转弯步态
void Turn(int state_flag,int speed_flag)
{
    float length;

    if(speed_flag == 'f')
    {
        length = 15.0f;
        state_detached_params[0].detached_params_0.freq = 4.0f;
        state_detached_params[0].detached_params_1.freq = 4.0f;
        state_detached_params[0].detached_params_2.freq = 4.0f;
        state_detached_params[0].detached_params_3.freq = 4.0f;
    }
    else if(speed_flag == 's')
    {
        length = 5.0f;
        state_detached_params[0].detached_params_0.freq = 1.5f;
        state_detached_params[0].detached_params_1.freq = 1.5f;
        state_detached_params[0].detached_params_2.freq = 1.5f;
        state_detached_params[0].detached_params_3.freq = 1.5f;
    }

    NewHeartbeat = 5;
    AllLegsSpeedLimit(SpeedMode_EXTREME);
    ChangeGainOfPID(10.0f,0,0.6f,0);
    switch (state_flag) {
        case 'l':
            state_detached_params[0].detached_params_0.step_length = -length;
            state_detached_params[0].detached_params_1.step_length = -length;
            state_detached_params[0].detached_params_2.step_length = length;
            state_detached_params[0].detached_params_3.step_length = length;
            break;
        case 'r':
            state_detached_params[0].detached_params_0.step_length = length;
            state_detached_params[0].detached_params_1.step_length = length;
            state_detached_params[0].detached_params_2.step_length = -length;
            state_detached_params[0].detached_params_3.step_length = -length;
            break;
        default:
            break;
    }
    gait_detached(state_detached_params[0],  0.0f, 0.5f, 0.5f, 0.0f,
                  1.0f, 1.0f, 1.0f,1.0f);
}

void EndPosture(void)
{
    motor_pos_controll(0,0,position);                    //3号电机与0号电机镜像对称
    HAL_Delay(1);
    motor_pos_controll(1,0,position);                    //3号电机与0号电机镜像对称
    HAL_Delay(1);
    motor_pos_controll(2,0,position);                    //3号电机与0号电机镜像对称
    HAL_Delay(1);
    motor_pos_controll(3,0,position);                    //3号电机与0号电机镜像对称
    HAL_Delay(1);
    motor_pos_controll(4,0,position);                    //3号电机与0号电机镜像对称
    HAL_Delay(1);
    motor_pos_controll(5,0,position);                    //3号电机与0号电机镜像对称
    HAL_Delay(1);
    motor_pos_controll(6,0,position);                    //3号电机与0号电机镜像对称
    HAL_Delay(1);
    motor_pos_controll(7,0,position);                    //3号电机与0号电机镜像对称
    HAL_Delay(1);
    motor_pos_controll(8,0,position);                    //3号电机与0号电机镜像对称
    HAL_Delay(1);
    motor_pos_controll(9,0,position);                    //3号电机与0号电机镜像对称
    HAL_Delay(1);
    motor_pos_controll(10,0,position);                    //3号电机与0号电机镜像对称
    HAL_Delay(1);
    motor_pos_controll(11,0,position);                    //3号电机与0号电机镜像对称
    HAL_Delay(1);
}

void Dog_Posture(void)
{
    switch (gpstate) {
        case HALT:
            StandUp_Posture();
            break;
        case END:
            LieDown_Posture();
            break;
        case TURN_LEFT:
            Turn('l','s');
            break;
        case TURN_RIGHT:
            Turn('r','s');
            break;
        case MARCH:
            Walk(Forward,0);
            break;
        case MARCH_BACK:
            Walk(Backward,0);
            break;
        default:
            break;
    }
}