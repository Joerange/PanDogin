//
// Created by 1 on 2023-11-07.
//
#include "Attitude_Task.h"
uint8_t delay_flag = 0;
float TargetAngle1 = 0,TargetAngle2 = 0;
enum GPStates gpstate = STOP;
enum DPStates dpstate = NONE;
float NewHeartbeat = 0;//心跳值
//全局姿态控制
int Global_IMU_Control = 0;
float Direct = 0;
float TargetAngle = 0;
int Race_count = 0;
uint8_t IMU_Stand_flag = 0;
uint8_t Solpe_flag = 0;

void StandUp_Posture(void)
{
    ChangeGainOfPID(5.0f,0.2f,0.03f,0.05f);//初始化pid
    AllLegsSpeedLimit(SpeedMode_VERYFAST);
    Get_Target(0,PI);
    SetCoupledThetaPositionAll();
}

void StandUp_Posture_IMU(void)
{
    static float x_want = 0, y_want = 0;
    AllLegsSpeedLimit(SpeedMode_VERYFAST);

    if (x_want == 0 && y_want == 0)
    {
        SetCartesianPositionAll_Delay(x_want,21.3,500);
    }
    if ((-90.0f + IMU_EulerAngle.EulerAngle[Pitch]) <= 0)
    {
        x_want =  21.3f * cosf((-90.0f + IMU_EulerAngle.EulerAngle[Pitch]) * PI / 180.0f);
        y_want = -21.3f * sinf((-90.0f + IMU_EulerAngle.EulerAngle[Pitch]) * PI / 180.0f);
    }

    IMU_Stand_flag = 1;

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
    AllLegsSpeedLimit(SpeedMode_VERYFAST);
    ChangeGainOfPID(10.0f,0.2f,0.6f,0);
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
            NewHeartbeat = 6;
            ChangeGainOfPID(12.0f,0.2f,0.6f,0);
            ChangeYawOfPID(200.0f,2.0f,3000.0f,10.0f);
            YawControl(yawwant, &state_detached_params[4], direction);
            gait_detached(state_detached_params[4],0.0f, 0.5f, 0.5f, 0.0f,
                          direction,direction,direction,direction);
            break;
        case 1://大步Trot
            Solpe_flag = 0;
            AllLegsSpeedLimit(30.0f);
            NewHeartbeat = 5;
            ChangeGainOfPID(20.0f,0.2f,0.6f,0);
            ChangeYawOfPID(0.0f,0.0f,3000.0f,10.0f);
            YawControl(yawwant, &state_detached_params[1], direction);
            gait_detached(state_detached_params[1],0.0f, 0.5f, 0.5f, 0.0f,
                          direction,direction,direction,direction);
            break;
        case 2://双木桥
            Solpe_flag = 0;
            AllLegsSpeedLimit(SpeedMode_EARLYEX);
            NewHeartbeat = 5;
            ChangeGainOfPID(15.5f,0.2f,0.6f,0);
            ChangeYawOfPID(0.35f,0.035f,3000.0f,10.0f);
            YawControl(yawwant, &state_detached_params[4], direction);
            gait_detached(state_detached_params[4],0.0f, 0.5f, 0.5f, 0.0f,
                          direction,direction,direction,direction);
            break;
        case 3://斜坡
            Solpe_flag = 1;
            AllLegsSpeedLimit(30.0f);
            NewHeartbeat = 5;
            ChangeGainOfPID(23.0f,0.2f,0.6f,0);
            ChangeYawOfPID(0.1f,0.01f,3000.0f,10.0f);
            YawControl(yawwant, &state_detached_params[8], direction);
            gait_detached(state_detached_params[8],0.0f, 0.5f, 0.5f, 0.0f,
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
        length = 20.0f;
        state_detached_params[0].detached_params_0.freq = 4.5f;
        state_detached_params[0].detached_params_1.freq = 4.5f;
        state_detached_params[0].detached_params_2.freq = 4.5f;
        state_detached_params[0].detached_params_3.freq = 4.5f;
    }
    else if(speed_flag == 's')
    {
        length = 12.0f;
        state_detached_params[0].detached_params_0.freq = 3.2f;
        state_detached_params[0].detached_params_1.freq = 3.2f;
        state_detached_params[0].detached_params_2.freq = 3.2f;
        state_detached_params[0].detached_params_3.freq = 3.2f;
    }

    NewHeartbeat = 5;
    AllLegsSpeedLimit(SpeedMode_EXTREME);
    ChangeGainOfPID(17.0f,0.2f,0.6f,0);
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

void Translate(int direction)
{
    switch (direction) {
        case 'r':
            AllLegsSpeedLimit(30.0f);
            NewHeartbeat = 5;
            ChangeGainOfPID(23.0f,2.3f,0.6f,0);
            gait_detached(state_detached_params[6],0.5f, 0.5f, 0.0f, 0.0f,
                          direction,direction,direction,direction);
            break;
        case 'l':
            AllLegsSpeedLimit(30.0f);
            NewHeartbeat = 5;
            ChangeGainOfPID(23.0f,2.3f,0.6f,0);
            gait_detached(state_detached_params[7],0.5f, 0.5f, 0.0f, 0.0f,
                          direction,direction,direction,direction);
            break;
        default:
            break;
    }
}

void WarnPosture(void)//警戒
{
    AllLegsSpeedLimit(SpeedMode_EXTREME);
    //使用刚度小，阻尼大的增益
    ChangeGainOfPID(8.0f,0.25f,70,0.22);
    SetPolarPositionAll_Delay(-60,LegLenthMin,700);
    gpstate=STOP;
}

void KneelPosture(void)//跪下
{
    ChangeGainOfPID(8,0.1f,80,0.22);//恢复正常PD
    AllLegsSpeedLimit(SpeedMode_SLOW);
    TargetAngle1=-(199.89f+offset_front_0-180.0f);//-(199.89+offset_front_1-180)
    TargetAngle2=180.0f-(88.8f-offset_front_1);//180-(88.8-offset_front_1)
    SetCoupledThetaPositionAll();
    gpstate = STOP;
    osDelay(700);
}

void Race_Competition(void)
{
    if(Race_count == 0 && Distance >= 80.0f)
    {
//        if(Distance <= 200.0f)
//        {
//            Change_SinStateDetachedParams(state_detached_params,1,1,22.3f, 20.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//            Change_SinStateDetachedParams(state_detached_params,1,2,22.3f, 20.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//            Change_SinStateDetachedParams(state_detached_params,1,3,22.3f, 20.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//            Change_SinStateDetachedParams(state_detached_params,1,4,22.3f, 20.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//
//        }

        Trot(Backward,1);
    }

    if(Distance < 80.0f && Race_count == 0)
    {
//        StandUp_Posture();
//        osDelay(800);
        while (IMU_EulerAngle.EulerAngle[Yaw] < 135.0f)
        {
            Turn('l','s');
        }
//        TargetAngle = -151.5f;
        for (int i = 0; i < 5; ++i) {
            distance[i] = 300;
        }
        Distance = 300.0f;
        visual.offset = 100;
        yawwant = 135.0f;
        Race_count++;
        StandUp_Posture();
        osDelay(200);

//        Change_SinStateDetachedParams(state_detached_params,1,1,22.3f, 27.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//        Change_SinStateDetachedParams(state_detached_params,1,2,22.3f, 27.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//        Change_SinStateDetachedParams(state_detached_params,1,3,22.3f, 27.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//        Change_SinStateDetachedParams(state_detached_params,1,4,22.3f, 27.0f,  2.2f, 0.15f, 0.35f, 5.2f);


    }
    if(Race_count == 1 && Distance >= 71.0f)
    {
//        if(Distance <= 200.0f)
//        {
//            Change_SinStateDetachedParams(state_detached_params,1,1,22.3f, 20.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//            Change_SinStateDetachedParams(state_detached_params,1,2,22.3f, 20.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//            Change_SinStateDetachedParams(state_detached_params,1,3,22.3f, 20.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//            Change_SinStateDetachedParams(state_detached_params,1,4,22.3f, 20.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//
//        }

        Trot(Backward,1);
    }
    if(Distance < 71.0f && Race_count == 1)
    {
//        StandUp_Posture();
//        osDelay(800);
        while (IMU_EulerAngle.EulerAngle[Yaw] < -91.0f || IMU_EulerAngle.EulerAngle[Yaw] > -89.0f)
        {
            Turn('l','s');
        }
//        TargetAngle = 0;
        for (int i = 0; i < 5; ++i) {
            distance[i] = 300;
        }
        Distance = 300.0f;
        visual.offset = 100;
        yawwant = -90.0f;
        Race_count++;
        StandUp_Posture();
        osDelay(200);

//        Change_SinStateDetachedParams(state_detached_params,1,1,22.3f, 27.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//        Change_SinStateDetachedParams(state_detached_params,1,2,22.3f, 27.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//        Change_SinStateDetachedParams(state_detached_params,1,3,22.3f, 27.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//        Change_SinStateDetachedParams(state_detached_params,1,4,22.3f, 27.0f,  2.2f, 0.15f, 0.35f, 5.2f);


    }
    if(Race_count == 2 && Distance >= 80.0f)
    {
//        if(Distance <= 200.0f)
//        {
//            Change_SinStateDetachedParams(state_detached_params,1,1,22.3f, 20.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//            Change_SinStateDetachedParams(state_detached_params,1,2,22.3f, 20.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//            Change_SinStateDetachedParams(state_detached_params,1,3,22.3f, 20.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//            Change_SinStateDetachedParams(state_detached_params,1,4,22.3f, 20.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//
//        }

        Trot(Backward,1);
    }
    if(Race_count == 2 && Distance < 80.0f)
    {
//        StandUp_Posture();
//        osDelay(800);
        while (IMU_EulerAngle.EulerAngle[Yaw] > 136.0f || IMU_EulerAngle.EulerAngle[Yaw] < 134.0f)
        {
            Turn('r','s');
        }
//        TargetAngle = 0;
        for (int i = 0; i < 5; ++i) {
            distance[i] = 300;
        }
        Distance = 300.0f;
        visual.offset = 100;
        yawwant = -45.0f;
        Race_count++;
        StandUp_Posture();
        osDelay(200);

//        Change_SinStateDetachedParams(state_detached_params,1,1,22.3f, 27.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//        Change_SinStateDetachedParams(state_detached_params,1,2,22.3f, 27.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//        Change_SinStateDetachedParams(state_detached_params,1,3,22.3f, 27.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//        Change_SinStateDetachedParams(state_detached_params,1,4,22.3f, 27.0f,  2.2f, 0.15f, 0.35f, 5.2f);


    }
    if(Race_count == 3 && Distance >= 70.0f)
    {
//        if(Distance <= 200.0f)
//        {
//            Change_SinStateDetachedParams(state_detached_params,1,1,22.3f, 20.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//            Change_SinStateDetachedParams(state_detached_params,1,2,22.3f, 20.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//            Change_SinStateDetachedParams(state_detached_params,1,3,22.3f, 20.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//            Change_SinStateDetachedParams(state_detached_params,1,4,22.3f, 20.0f,  2.2f, 0.15f, 0.35f, 5.2f);
//
//        }

        Trot(Backward,1);
    }
    if(Race_count == 3 && Distance < 70.0f)
    {
        StandUp_Posture();
//        osDelay(200);
    }
}

int ll(void)
{
    while(visual.data_8[1] > 96)
    {
        Trot(Backward,1);
    }

    StandUp_Posture();
    osDelay(200);

    for (int i = 0; i < 500; ++i) {
        MarkingTime();
    }

    StandUp_Posture();
    osDelay(200);

    ExecuteJump(High_Jump,70);
    osDelay(600);
    while(visual.data_8[5] < 11 || visual.data_8[5] > 18)
    {
        if(visual.data_8[5] < 11)
            Trot(Forward,2);
        if(visual.data_8[5] > 18)
            Trot(Backward,2);
    }
    while(IMU_EulerAngle.EulerAngle[Yaw] > 0.5f || IMU_EulerAngle.EulerAngle[Yaw] < -0.5f)
    {
        if(IMU_EulerAngle.EulerAngle[Yaw] > 0.5f)
            Turn('r','s');
        if(IMU_EulerAngle.EulerAngle[Yaw] < -0.5f)
            Turn('l','s');
    }



    StandUp_Posture();
    osDelay(200);

    for (int i = 0; i < 500; ++i) {
        MarkingTime();
    }

    StandUp_Posture();
    osDelay(200);

    ExecuteJump(High_Jump,70);
    osDelay(700);

    while(visual.data_8[5] < 11 || visual.data_8[5] > 18)
    {
        if(visual.data_8[5] < 11)
            Trot(Forward,2);
        if(visual.data_8[5] > 18)
            Trot(Backward,2);
    }

    while(IMU_EulerAngle.EulerAngle[Yaw] > 0.5f || IMU_EulerAngle.EulerAngle[Yaw] < -0.5f)
    {
        if(IMU_EulerAngle.EulerAngle[Yaw] > 0.5f)
            Turn('r','s');
        if(IMU_EulerAngle.EulerAngle[Yaw] < -0.5f)
            Turn('l','s');
    }



    StandUp_Posture();
    osDelay(200);

    for (int i = 0; i < 500; ++i) {
        MarkingTime();
    }

    StandUp_Posture();
    osDelay(200);

    ExecuteJump(High_Jump,70);
    osDelay(700);

//    while(visual.data_8[5] < 11 || visual.data_8[5] > 18)
//    {
//        if(visual.data_8[5] < 11)
//            Trot(Forward,2);
//        if(visual.data_8[5] > 18)
//            Trot(Backward,2);
//    }

    while(IMU_EulerAngle.EulerAngle[Yaw] > 0.5f || IMU_EulerAngle.EulerAngle[Yaw] < -0.5f)
    {
        if(IMU_EulerAngle.EulerAngle[Yaw] > 0.5f)
            Turn('r','s');
        if(IMU_EulerAngle.EulerAngle[Yaw] < -0.5f)
            Turn('l','s');
    }

    StandUp_Posture();
    osDelay(200);

    for (int i = 0; i < 500; ++i) {
        MarkingTime();
    }

    StandUp_Posture();
    osDelay(200);

    ExecuteJump(High_Jump,70);

    return 0;

}