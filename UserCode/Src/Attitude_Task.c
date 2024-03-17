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
float Direct = 0;
float TargetAngle = 0;
int Race_count = 0;

void StandUp_Posture(void)
{
    AllLegsSpeedLimit(SpeedMode_VERYFAST - 3);
    Get_Target(0,PI);
    SetCoupledThetaPositionAll();
}

void StandUp_Posture_IMU(void)
{
    static float x_want = 0, y_want = 0;
    AllLegsSpeedLimit(SpeedMode_VERYFAST - 3);

    if (x_want == 0 && y_want == 0)
    {
        SetCartesianPositionAll_Delay(x_want,21.3,500);
    }
    if ((-90.0f + IMU_EulerAngle.EulerAngle[Pitch]) <= 0)
    {
        x_want =  21.3f * cosf((-90.0f + IMU_EulerAngle.EulerAngle[Pitch]) * PI / 180.0f);
        y_want = -21.3f * sinf((-90.0f + IMU_EulerAngle.EulerAngle[Pitch]) * PI / 180.0f);
    }

    SetCartesianPositionAll_Delay(x_want,y_want,0);
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
    Direct = direction;
    switch(kind)
    {
        case 0://小步Trot
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            Target_offset2 = 0.112f;
            NewHeartbeat = 6;
            ChangeGainOfPID(12.0f,0,0.6f,0);
            ChangeYawOfPID(200.0f,2.0f,3000.0f,10.0f);
            YawControl(yawwant, &state_detached_params[4], direction);
            gait_detached(state_detached_params[4],0.0f, 0.5f, 0.5f, 0.0f,
                          direction,direction,direction,direction);
            break;
        case 1://大步Trot
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            Target_offset2 = 0.112f;
            NewHeartbeat = 5;
            ChangeGainOfPID(18.5f,0.0f,0.6f,0);
            ChangeYawOfPID(0.4f,0.01f,3000.0f,10.0f);
            YawControl(yawwant, &state_detached_params[1], direction);
            gait_detached(state_detached_params[1],0.0f, 0.5f, 0.5f, 0.0f,
                          direction,direction,direction,direction);
            break;
        case 3://双木桥
            AllLegsSpeedLimit(SpeedMode_EARLYEX);
            Target_offset2 = 0.112f;
            NewHeartbeat = 4;
            ChangeGainOfPID(15.5f,0.0f,0.6f,0);
            ChangeYawOfPID(0.3f,0.01f,3000.0f,10.0f);
            YawControl(yawwant, &state_detached_params[4], direction);
            gait_detached(state_detached_params[4],0.0f, 0.5f, 0.5f, 0.0f,
                          direction,direction,direction,direction);
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
        length = 7.0f;
        state_detached_params[0].detached_params_0.freq = 2.0f;
        state_detached_params[0].detached_params_1.freq = 2.0f;
        state_detached_params[0].detached_params_2.freq = 2.0f;
        state_detached_params[0].detached_params_3.freq = 2.0f;
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

void Handshake(void)
{
    //整体限制
    AllLegsSpeedLimit(SpeedMode_SLOW);
    ChangeGainOfPID(7.0f,0.0f,0.8f,0);
    //左前腿
    x =  LegLenthMax*cos((30)*PI/180);
    y = -LegLenthMax*sin((30)*PI/180);
    CartesianToTheta();
    SetCoupledThetaPosition(0);
    //其它腿
    x =  LegLenthMin*cos(60*PI/180);
    y =  LegLenthMin*sin(60*PI/180);
    CartesianToTheta();
    SetCoupledThetaPosition(1);
    SetCoupledThetaPosition(2);
    SetCoupledThetaPosition(3);
    gpstate = STOP;

}

void StretchPosture(void)
{
    AllLegsSpeedLimit(SpeedMode_FAST);
    x = -(LegStandLenth + 5) * cos(60 * PI / 180);
    y = (LegStandLenth + 5) * sin(60 * PI / 180);
    CartesianToTheta();
    SetCoupledThetaPosition(1);
    SetCoupledThetaPosition(3);
    osDelay(800);
    x = LegLenthMax * cos(30 * PI / 180);
    y = LegLenthMax * sin(30 * PI / 180);
    CartesianToTheta();
    SetCoupledThetaPosition(0);
    osDelay(1500);
    SetCoupledThetaPosition(2);
    gpstate = STOP;
}

void SquatPosture(void)
{
    x=0;
    y=LegSquatLenth;
    //控制
    AllLegsSpeedLimit(SpeedMode_SLOW);
    CartesianToTheta();//进行坐标转换
    SetCoupledThetaPositionAll();
    gpstate = STOP;
    osDelay(400);
}
void FBwAaLitAir(void)
{
    /*自动转换*/
    x=0;
    y=-LegSquatLenth;
    //使用低刚度和大量的阻尼来处理下降
    ChangeGainOfPID(8.0,0.1,0.25,100);
    AllLegsSpeedLimit(SpeedMode_SLOW);
    //后腿反方向（向后）转动
    ReverseMoveOpen();//方向反向控制
    CartesianToTheta();
    SetCoupledThetaPosition(1);
    SetCoupledThetaPosition(3);
    //前腿正常（向前）转动
    ReverseMoveClose();//恢复（其实不写也没事儿）
    CartesianToTheta();
    SetCoupledThetaPosition(0);
    SetCoupledThetaPosition(2);
    gpstate = STOP;//回到停止态
    osDelay(2000);
}
void Race_Competition(void)
{
    if(Race_count == 0 && Distance >= 115.0f)
        Trot(Forward,1);
    if(Distance < 115.0f && Race_count == 0)
    {
        while (IMU_EulerAngle.EulerAngle[Yaw] > -42.0f)
        {
            Turn('r','f');
        }
        for (int i = 0; i < 5; ++i) {
            distance[i] = 300;
        }
        Distance = 300.0f;
        visual.offset = 0;
        TargetAngle = -149;
        yawwant = -42.0f;
        Race_count++;
    }
    if(Race_count == 1 && Distance >= 69.0f)
    {
        Trot(Backward,1);
    }
    if(Distance < 69.0f && Race_count == 1)
    {
        while (IMU_EulerAngle.EulerAngle[Yaw] > -89.5f)
        {
            Turn('r','f');
        }
        for (int i = 0; i < 5; ++i) {
            distance[i] = 300;
        }
        Distance = 300.0f;
        visual.offset = 0;
        TargetAngle = 0;
        yawwant = -90.0f;
        Race_count++;
    }
    if(Race_count == 2 && Distance >= 50.0f)
        Trot(Forward,1);
    if(Race_count == 2 && Distance < 50.0f)
        StandUp_Posture();
}