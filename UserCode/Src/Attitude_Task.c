//
// Created by 1 on 2023-11-07.
//
#include "Attitude_Task.h"

float TargetAngle1 = 0,TargetAngle2 = 0;
enum GPStates gpstate = STOP;
enum DPStates dpstate = NONE;
float NewHeartbeat = 0;//����ֵ
//ȫ����̬����
int Global_IMU_Control = 0;
float Direct = 0;

void StandUp_Posture(void)
{
    AllLegsSpeedLimit(SpeedMode_EARLYEX);
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
    AllLegsSpeedLimit(SpeedMode_FAST);
    ChangeGainOfPID(3.8f,0,0.6f,0);
//    ChangeYawOfPID(50.0f,0.5f,2500.0f,10.0f);
//    YawControl(yawwant, &state_detached_params[2], 1.0f);
    gait_detached(state_detached_params[2],0.0f, 0.5f, 0.5f, 0.0f,
                  1.0f,1.0f,1.0f,1.0f);
}
//ʵ������Trot��̬
void Trot(float direction,int8_t kind)
{
    Direct = direction;
    switch(kind)
    {
        case 0://С��Trot
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            NewHeartbeat = 6;
            ChangeGainOfPID(12.0f,0,0.6f,0);
            ChangeYawOfPID(200.0f,2.0f,3000.0f,10.0f);
            YawControl(yawwant, &state_detached_params[4], direction);
            gait_detached(state_detached_params[4],0.0f, 0.5f, 0.5f, 0.0f,
                          direction,direction,direction,direction);
            break;
        case 1://��Trot
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            NewHeartbeat = 5;
            ChangeGainOfPID(17.0f,0.0f,0.6f,0);
            gait_detached(state_detached_params[1],0.0f, 0.5f, 0.5f, 0.0f,
                          direction,direction,direction,direction);
            break;
        default:
            break;
    }
}

//����
void Walk(float direction,uint8_t speed)
{
    NewHeartbeat = 4;
    AllLegsSpeedLimit(SpeedMode_FAST);
    ChangeGainOfPID(3.5f,0,0.6f,0);
    ChangeYawOfPID(100.0f,0.5f,2500.0f,10.0f);
    YawControl(yawwant, &state_detached_params[3], direction);
    gait_detached(state_detached_params[3],0.0,0.75,0.5,0.25,direction,direction,direction,direction);
}
//ת�䲽̬
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
    motor_pos_controll(0,0,position);                    //3�ŵ����0�ŵ������Գ�
    HAL_Delay(1);
    motor_pos_controll(1,0,position);                    //3�ŵ����0�ŵ������Գ�
    HAL_Delay(1);
    motor_pos_controll(2,0,position);                    //3�ŵ����0�ŵ������Գ�
    HAL_Delay(1);
    motor_pos_controll(3,0,position);                    //3�ŵ����0�ŵ������Գ�
    HAL_Delay(1);
    motor_pos_controll(4,0,position);                    //3�ŵ����0�ŵ������Գ�
    HAL_Delay(1);
    motor_pos_controll(5,0,position);                    //3�ŵ����0�ŵ������Գ�
    HAL_Delay(1);
    motor_pos_controll(6,0,position);                    //3�ŵ����0�ŵ������Գ�
    HAL_Delay(1);
    motor_pos_controll(7,0,position);                    //3�ŵ����0�ŵ������Գ�
    HAL_Delay(1);
    motor_pos_controll(8,0,position);                    //3�ŵ����0�ŵ������Գ�
    HAL_Delay(1);
    motor_pos_controll(9,0,position);                    //3�ŵ����0�ŵ������Գ�
    HAL_Delay(1);
    motor_pos_controll(10,0,position);                    //3�ŵ����0�ŵ������Գ�
    HAL_Delay(1);
    motor_pos_controll(11,0,position);                    //3�ŵ����0�ŵ������Գ�
    HAL_Delay(1);
}

void Handshake(void)
{
    //��������
    AllLegsSpeedLimit(SpeedMode_SLOW);
    ChangeGainOfPID(7.0f,0.0f,0.8f,0);
    //��ǰ��
    x =  LegLenthMax*cos((30)*PI/180);
    y = -LegLenthMax*sin((30)*PI/180);
    CartesianToTheta();
    SetCoupledThetaPosition(0);
    //������
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
    //����
    AllLegsSpeedLimit(SpeedMode_SLOW);
    CartesianToTheta();//��������ת��
    SetCoupledThetaPositionAll();
    gpstate = STOP;
    osDelay(400);
}
void FBwAaLitAir(void)
{
    /*�Զ�ת��*/
    x=0;
    y=-LegSquatLenth;
    //ʹ�õ͸նȺʹ����������������½�
    ChangeGainOfPID(8.0,0.1,0.25,100);
    AllLegsSpeedLimit(SpeedMode_SLOW);
    //���ȷ��������ת��
    ReverseMoveOpen();//���������
    CartesianToTheta();
    SetCoupledThetaPosition(1);
    SetCoupledThetaPosition(3);
    //ǰ����������ǰ��ת��
    ReverseMoveClose();//�ָ�����ʵ��дҲû�¶���
    CartesianToTheta();
    SetCoupledThetaPosition(0);
    SetCoupledThetaPosition(2);
    gpstate = STOP;//�ص�ֹ̬ͣ
    osDelay(2000);
}