//
// Created by Joerange on 2023/11/18.
//
#include <string.h>
#include "imu.h"
//�������ڽ��ղ�ͬIMU��Ϣ��������
//imu_measure IMU_EulerAngle={0};
imu_measure IMU_LinearAcc={0};
imu_measure IMU_Distance={0};
imu_measure IMU_Heave={0};
imu_measure IMU_RawAcc={0};
IMU_EulerAngle_u IMU_EulerAngle={0};

//IMU���ڽ���BUF
uint8_t IMU_RX_BUF[IMU_REC_LEN]={0};
//�Ƿ���IMU����
uint8_t IMU_Control_Flag = 0;
//IMU��PID��Kpֵ
float IMU_Kp_Intensity = 4.0f;
//�����Ƕ�
float yawwant = 0.0f,pitchwant = 0.0f,rollwant = 0.0f;
//�IMU������ر���
float LegX_IMU_Control[4]={0};
#define AngleMargin 20.0f
float AngleDelta = 0;
//�ߵ���̬����PID�ṹ���ʼ��

void IMU_init()
{
    /****IMU��PID��ʼ��****///Pitch��Roll��PID��ʼ����ȫ����̬���ƣ������ֱ�ӿ��Ƶ��ת�ǣ�
    IMU_AUTO_PID_SET(0.5,0.01,2.0,3600);
}

void IMU_AUTO_PID_SET(float kp,float ki,float kd,float SumError_limit)
{
    PID_Set_KP_KI_KD(&Pitch_PID_Loop,kp,ki,kd);
    Pitch_PID_Loop.Output_limit = 180;
    Pitch_PID_Loop.SumError_limit = SumError_limit;
    memcpy(&Roll_PID_Loop,&Pitch_PID_Loop,sizeof(PIDTypeDef));
}

//�ر�IMUȫ����̬����
void Close_Global_IMU_Control(void)
{
    Global_IMU_Control=0;
    memset(LegX_IMU_Control,0,sizeof(float)*4);
}

//IMU���ȵ��޷����е�BUG��
void LegLenthLimit(uint8_t LegID)
{
    //���ȳ���������limit
    AngleDelta = AngleWant_MotorX[2*LegID]-AngleWant_MotorX[2*LegID+1];//�������Ŀ��Ƕ�֮���С
    //�¶�̬
    if(((AngleDelta>-180 && AngleDelta<0) || AngleDelta>180))
    {
        if(LegX_IMU_Control[LegID]<0)
        {
            if(abs(AngleDelta) <= AngleMargin) LegX_IMU_Control[LegID]=0;//������Ŀ��Ƕȷǳ���£���򲻽��е���
            else if((-2*LegX_IMU_Control[LegID]+AngleMargin) > abs(AngleDelta)) LegX_IMU_Control[LegID] = -(abs(AngleDelta)-AngleMargin)/2;//�����޷�
        }
    }
        //վ��̬
    else
    {
        if(LegX_IMU_Control[LegID]>0)
        {
            if(abs(AngleDelta) <= AngleMargin) LegX_IMU_Control[LegID]=0;//������Ŀ��Ƕȷǳ���£���򲻽��е���
            else if((2*LegX_IMU_Control[LegID]+AngleMargin) > abs(AngleDelta)) LegX_IMU_Control[LegID] = (abs(AngleDelta)-AngleMargin)/2;//�����޷�
        }
    }
}


//ȫ����̬����
void AttitudeControl_Global(float roll_set,float pitch_set)
{
    if(IMU_Control_Flag)
    {
        /*******IMU��PID���*******/
        //PIDĿ���趨��һ�㶼��0������Pitch��ʱҪ������һ���Ƕ�;���⻹�п�����Ϊ��΢��Yaw��
        SetPoint_IMU(&Pitch_PID_Loop,pitch_set);
        SetPoint_IMU(&Roll_PID_Loop,roll_set);
        //PID���㣨���õ��˴��ڽ������IMU_EulerAngle����λ��ʽPID���㣩
        PID_PosLocCalc(&Pitch_PID_Loop,IMU_EulerAngle.EulerAngle[Pitch]);//���бʱ��pitchΪ��
        PID_PosLocCalc(&Roll_PID_Loop,IMU_EulerAngle.EulerAngle[Roll]);//���Ҳ෭ʱ��rollΪ��
        //��������
        if((Pitch_PID_Loop.Out_put<0.5f) && (Pitch_PID_Loop.Out_put>-0.5f))   Pitch_PID_Loop.Out_put=0;
        if((Roll_PID_Loop.Out_put<0.5f) && (Roll_PID_Loop.Out_put>-0.5f)) 	Roll_PID_Loop.Out_put=0;
        /**********��̬����*********/
        //�Ⱥ�0123�ֱ��Ӧ��ǰ�������ǰ���Һ󣬼�1��2��Ӧ���ȣ�3��4��Ӧ���ȡ�ע�����öԣ�������������
        //���ȳ���IMU���ڣ����߼���
        LegX_IMU_Control[0] = ( Pitch_PID_Loop.Out_put + Roll_PID_Loop.Out_put);//���ϸ�ֵ������������С10�����нǶȻ�ԭ��
        LegX_IMU_Control[1] = (-Pitch_PID_Loop.Out_put + Roll_PID_Loop.Out_put);
        LegX_IMU_Control[2] = ( Pitch_PID_Loop.Out_put - Roll_PID_Loop.Out_put);
        LegX_IMU_Control[3] = (-Pitch_PID_Loop.Out_put - Roll_PID_Loop.Out_put);
        //���ȳ����޷�����LegX_IMU_Control[0]�Ƚ��д���
        LegLenthLimit(0);LegLenthLimit(2);LegLenthLimit(1);LegLenthLimit(3);
    }
}



//�ô������δ֪bug���ȴ�һ��������ˡ�
uint8_t IMU_WaitAngle(uint8_t angle_type, float takeoff_inclination, float imu_angle_half_range, uint8_t lock, uint8_t second_flag)
{
    static float last_times=0;
    static uint8_t SeconTime=0;      //�ж��Ƿ��ǵڶ��ε���
    static uint8_t k=0;
    if(lock) return 0;//ͨ��lock�Ըú�������������ʧȥ�жϹ��ܣ�ֻ����0��
    else if(k==0)
    {
        //last_times=times;//��¼��ǰʱ��,λ�ڶ�ʱ��
        k=1;
    }
    if(second_flag==1 && angle_type==Pitch)
    {
        //һֱ���ǶȺ��ʣ�Ȼ���л�����һ����̬��
        if(SeconTime==0 && IMU_EulerAngle.EulerAngle[angle_type]> -(takeoff_inclination+imu_angle_half_range) && IMU_EulerAngle.EulerAngle[angle_type]< -(takeoff_inclination-imu_angle_half_range) )
        {
            SeconTime=2;
        }
        else if(SeconTime==2 && IMU_EulerAngle.EulerAngle[angle_type]> -90 && IMU_EulerAngle.EulerAngle[angle_type]< -85 ) SeconTime=1;
        else if(SeconTime==1 && IMU_EulerAngle.EulerAngle[angle_type]> -(takeoff_inclination+imu_angle_half_range) && IMU_EulerAngle.EulerAngle[angle_type]< -(takeoff_inclination-imu_angle_half_range) )
        {
            //times=last_times;//����֮ǰ��timesֵ��
            last_times=0;
            SeconTime=0;
            k=0;
            return 0;//������־
        }
    }
    return 1;
}

void IMU_Data_Process(uint16_t rx_len)
{
    static uint8_t SystematicErrorFlag = 0;
    static float Yaw_ErrorCorrection = 0;
    static float Pitch_ErrorCorrection = 0;
    static float Roll_ErrorCorrection = 0;
    //���͵�ŷ���������У�ǰ0-5���ֽ���header�����ǿ��Բ��ܣ�6-8�ֽ�ΪPID����������жϡ�
    if(IMU_RX_BUF[6]==0x01 && IMU_RX_BUF[7]==0xb0 && IMU_RX_BUF[8]==0x10)//����ŷ���ǣ�PID��0xB001��������LSB���䣬��ʵ�����յ�01�����յ�B0��PID֮����PL�����غɴ�С��0x10��ʾ16�ֽڵ�����
    {
        //���������ݷŵ�ŷ������������
        for(uint8_t i=9;i<21;i++) IMU_EulerAngle.raw_data[i-9] = IMU_RX_BUF[i];//�Ƕ����ݲ�����0 1 2λ�õ����ݣ���Ϊ����֡ͷ����������3-14��4��Ԫ�أ�*3���飩���ݡ�
        //����ʼ�⵽��ֵ��Ϊ�������ֵ
        switch(SystematicErrorFlag)//����switcg-case����ʵ�ַǳ�����Ч�˷�������ʱ
        {
            case 10:
            {
                //ϵͳ�������ֵ����
                Yaw_ErrorCorrection = IMU_EulerAngle.EulerAngle[Yaw];
                Pitch_ErrorCorrection = IMU_EulerAngle.EulerAngle[Pitch];
                Roll_ErrorCorrection = IMU_EulerAngle.EulerAngle[Roll];
                SystematicErrorFlag++;
            }
            case 11:break;
            default:
                SystematicErrorFlag++;
                break;
        }
        //���������Ƕ�ֵ
        IMU_EulerAngle.EulerAngle[Yaw]  -= Yaw_ErrorCorrection;
        IMU_EulerAngle.EulerAngle[Pitch]-= Pitch_ErrorCorrection;
        IMU_EulerAngle.EulerAngle[Roll] -=Roll_ErrorCorrection;
    }
}

void Control_Flag(uint8_t IMU_Flag,uint8_t Visual_flag)
{
    if(IMU_Flag == 1)
        IMU_Control_Flag = 1;
    else if(Visual_flag == 1)
        visual_control_flag = 1;
}