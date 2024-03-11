//
// Created by Joerange on 2023/11/18.
//
#include <string.h>
#include "imu.h"
//定义用于接收不同IMU信息的联合体
//imu_measure IMU_EulerAngle={0};
imu_measure IMU_LinearAcc={0};
imu_measure IMU_Distance={0};
imu_measure IMU_Heave={0};
imu_measure IMU_RawAcc={0};
IMU_EulerAngle_u IMU_EulerAngle={0};

//IMU串口接收BUF
uint8_t IMU_RX_BUF[IMU_REC_LEN]={0};
//是否开启IMU控制
uint8_t IMU_Control_Flag = 0;
//IMU的PID的Kp值
float IMU_Kp_Intensity = 4.0f;
//期望角度
float yawwant = 0.0f,pitchwant = 0.0f,rollwant = 0.0f;
//差动IMU调节相关变量
float LegX_IMU_Control[4]={0};
#define AngleMargin 20.0f
float AngleDelta = 0;
//惯导姿态控制PID结构体初始化

void IMU_init()
{
    /****IMU的PID初始化****///Pitch和Roll的PID初始化（全局姿态控制，其输出直接控制电机转角）
    IMU_AUTO_PID_SET(0.5,0.01,2.0,3600);
}

void IMU_AUTO_PID_SET(float kp,float ki,float kd,float SumError_limit)
{
    PID_Set_KP_KI_KD(&Pitch_PID_Loop,kp,ki,kd);
    Pitch_PID_Loop.Output_limit = 180;
    Pitch_PID_Loop.SumError_limit = SumError_limit;
    memcpy(&Roll_PID_Loop,&Pitch_PID_Loop,sizeof(PIDTypeDef));
}

//关闭IMU全局姿态控制
void Close_Global_IMU_Control(void)
{
    Global_IMU_Control=0;
    memset(LegX_IMU_Control,0,sizeof(float)*4);
}

//IMU自稳的限幅（有点BUG）
void LegLenthLimit(uint8_t LegID)
{
    //狗腿长度上下限limit
    AngleDelta = AngleWant_MotorX[2*LegID]-AngleWant_MotorX[2*LegID+1];//求解两腿目标角度之差大小
    //下蹲态
    if(((AngleDelta>-180 && AngleDelta<0) || AngleDelta>180))
    {
        if(LegX_IMU_Control[LegID]<0)
        {
            if(abs(AngleDelta) <= AngleMargin) LegX_IMU_Control[LegID]=0;//若两腿目标角度非常靠拢，则不进行调节
            else if((-2*LegX_IMU_Control[LegID]+AngleMargin) > abs(AngleDelta)) LegX_IMU_Control[LegID] = -(abs(AngleDelta)-AngleMargin)/2;//调节限幅
        }
    }
        //站立态
    else
    {
        if(LegX_IMU_Control[LegID]>0)
        {
            if(abs(AngleDelta) <= AngleMargin) LegX_IMU_Control[LegID]=0;//若两腿目标角度非常靠拢，则不进行调节
            else if((2*LegX_IMU_Control[LegID]+AngleMargin) > abs(AngleDelta)) LegX_IMU_Control[LegID] = (abs(AngleDelta)-AngleMargin)/2;//调节限幅
        }
    }
}


//全局姿态控制
void AttitudeControl_Global(float roll_set,float pitch_set)
{
    if(IMU_Control_Flag)
    {
        /*******IMUのPID相关*******/
        //PID目标设定（一般都是0，除了Pitch有时要求它是一定角度;另外还有可能是为了微调Yaw）
        SetPoint_IMU(&Pitch_PID_Loop,pitch_set);
        SetPoint_IMU(&Roll_PID_Loop,roll_set);
        //PID计算（利用到了串口解算出的IMU_EulerAngle进行位置式PID计算）
        PID_PosLocCalc(&Pitch_PID_Loop,IMU_EulerAngle.EulerAngle[Pitch]);//向后斜时，pitch为负
        PID_PosLocCalc(&Roll_PID_Loop,IMU_EulerAngle.EulerAngle[Roll]);//向右侧翻时，roll为负
        //死区设置
        if((Pitch_PID_Loop.Out_put<0.5f) && (Pitch_PID_Loop.Out_put>-0.5f))   Pitch_PID_Loop.Out_put=0;
        if((Roll_PID_Loop.Out_put<0.5f) && (Roll_PID_Loop.Out_put>-0.5f)) 	Roll_PID_Loop.Out_put=0;
        /**********步态控制*********/
        //腿号0123分别对应左前、左后、右前、右后，即1、2对应左腿，3、4对应右腿。注意配置对，否则正反馈。
        //狗腿长度IMU调节（正逻辑）
        LegX_IMU_Control[0] = ( Pitch_PID_Loop.Out_put + Roll_PID_Loop.Out_put);//加上负值进行削减，缩小10倍进行角度还原。
        LegX_IMU_Control[1] = (-Pitch_PID_Loop.Out_put + Roll_PID_Loop.Out_put);
        LegX_IMU_Control[2] = ( Pitch_PID_Loop.Out_put - Roll_PID_Loop.Out_put);
        LegX_IMU_Control[3] = (-Pitch_PID_Loop.Out_put - Roll_PID_Loop.Out_put);
        //狗腿长度限幅（对LegX_IMU_Control[0]等进行处理）
        LegLenthLimit(0);LegLenthLimit(2);LegLenthLimit(1);LegLenthLimit(3);
    }
}



//该代码存在未知bug，等待一个解决的人。
uint8_t IMU_WaitAngle(uint8_t angle_type, float takeoff_inclination, float imu_angle_half_range, uint8_t lock, uint8_t second_flag)
{
    static float last_times=0;
    static uint8_t SeconTime=0;      //判断是否是第二次到达
    static uint8_t k=0;
    if(lock) return 0;//通过lock对该函数进行上锁，失去判断功能，只返回0。
    else if(k==0)
    {
        //last_times=times;//记录当前时间,位于定时器
        k=1;
    }
    if(second_flag==1 && angle_type==Pitch)
    {
        //一直到角度合适，然后切换到下一个步态。
        if(SeconTime==0 && IMU_EulerAngle.EulerAngle[angle_type]> -(takeoff_inclination+imu_angle_half_range) && IMU_EulerAngle.EulerAngle[angle_type]< -(takeoff_inclination-imu_angle_half_range) )
        {
            SeconTime=2;
        }
        else if(SeconTime==2 && IMU_EulerAngle.EulerAngle[angle_type]> -90 && IMU_EulerAngle.EulerAngle[angle_type]< -85 ) SeconTime=1;
        else if(SeconTime==1 && IMU_EulerAngle.EulerAngle[angle_type]> -(takeoff_inclination+imu_angle_half_range) && IMU_EulerAngle.EulerAngle[angle_type]< -(takeoff_inclination-imu_angle_half_range) )
        {
            //times=last_times;//返回之前的times值。
            last_times=0;
            SeconTime=0;
            k=0;
            return 0;//结束标志
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
    //发送的欧拉角数据中，前0-5个字节是header，我们可以不管；6-8字节为PID，用其进行判断。
    if(IMU_RX_BUF[6]==0x01 && IMU_RX_BUF[7]==0xb0 && IMU_RX_BUF[8]==0x10)//对于欧拉角，PID是0xB001，由于是LSB传输，故实际先收到01，后收到B0。PID之后是PL，即载荷大小，0x10表示16字节的数据
    {
        //将串口数据放到欧拉角联合体中
        for(uint8_t i=9;i<21;i++) IMU_EulerAngle.raw_data[i-9] = IMU_RX_BUF[i];//角度数据不包括0 1 2位置的内容（作为三个帧头），而包括3-14共4（元素）*3（组）数据。
        //将初始测到的值作为误差修正值
        switch(SystematicErrorFlag)//利用switcg-case可以实现非常简洁高效了非阻塞延时
        {
            case 10:
            {
                //系统误差修正值计算
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
        //误差修正后角度值
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