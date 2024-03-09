//
// Created by 1 on 2023-11-07.
//
#include "Attitude_Slove.h"

float AngleWant_MotorX[9] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
float Leg1_Delay = 0;
float Leg2_Delay = 0.5;
float Leg3_Delay = 0.5;
float Leg4_Delay = 0;
float step_angle[4] = {0};
float times = 0.0f;
float x,y;
uint8_t reverse_move_flag = 0;
float steplen = 0;

//用于复制上方状态数组作为永恒基准。
DetachedParam StateDetachedParams_Copy[StatesMaxNum] = {0};
//调试时用来改变生成的轨迹参数
void Change_SinStateDetachedParams(DetachedParam *State,int8_t id,int8_t legid,float stance_height,float step_length,
                                float up_amp,float down_amp,float flight_percent,float freq)
{
    State[id].GaitID = id;

    switch (legid) {
        case 1:
            State[id].detached_params_0.stance_height = stance_height;
            State[id].detached_params_0.step_length = step_length;
            State[id].detached_params_0.up_amp = up_amp;
            State[id].detached_params_0.down_amp = down_amp;
            State[id].detached_params_0.flight_percent = flight_percent;
            State[id].detached_params_0.freq = freq;

            break;
        case 2:
            State[id].detached_params_1.stance_height = stance_height;
            State[id].detached_params_1.step_length = step_length;
            State[id].detached_params_1.up_amp = up_amp;
            State[id].detached_params_1.down_amp = down_amp;
            State[id].detached_params_1.flight_percent = flight_percent;
            State[id].detached_params_1.freq = freq;

            break;
        case 3:
            State[id].detached_params_2.stance_height = stance_height;
            State[id].detached_params_2.step_length = step_length;
            State[id].detached_params_2.up_amp = up_amp;
            State[id].detached_params_2.down_amp = down_amp;
            State[id].detached_params_2.flight_percent = flight_percent;
            State[id].detached_params_2.freq = freq;

            break;
        case 4:
            State[id].detached_params_3.stance_height = stance_height;
            State[id].detached_params_3.step_length = step_length;
            State[id].detached_params_3.up_amp = up_amp;
            State[id].detached_params_3.down_amp = down_amp;
            State[id].detached_params_3.flight_percent = flight_percent;
            State[id].detached_params_3.freq = freq;

            break;
        default:
            break;
    }

}

void SetCoupledThetaPositionAll(void)
{
    SetCoupledThetaPosition(0);
    SetCoupledThetaPosition(1);
    SetCoupledThetaPosition(2);
    SetCoupledThetaPosition(3);
}

void SetCoupledThetaPosition(int LegId)
{
    switch(LegId)
    {
        case 0:
            AngleWant_MotorX[1]=-TargetAngle2+offset_front_1;
            AngleWant_MotorX[2]=-TargetAngle1+offset_front_0;//+10.0f
            break;
        case 1:
            AngleWant_MotorX[3]=-TargetAngle2+offset_back_1;//+5.0f
            AngleWant_MotorX[4]=-TargetAngle1+offset_back_0;
            break;
        case 2:
            AngleWant_MotorX[5]=TargetAngle1-offset_front_0;//-4.0f
            AngleWant_MotorX[6]=TargetAngle2-offset_front_1;
            break;
        case 3:
            AngleWant_MotorX[7]=TargetAngle1-offset_back_0-0.2f;
            AngleWant_MotorX[8]=TargetAngle2-offset_back_1;
            break;
        default:
            break;
    }

    //注意角度赋值根据不同的电机顺序和正负不同，同时也受机械结果安装的影响。若有改动，则这里的角度的对应关系也要变。
    /*
    根据如下角度配置信息，我们可以得到如下结论：
    01和67号电机(电机设置一致）、75和23号电机（电机设置一致且与前述电机方向相反）分别对应两条对角线。

                                     头
            左前腿 /~**********************************~\右前腿
                    theta0 12 theta1  -theta1 56 -theta0
                     leftleg(usart1)	rightleg(usart2)
                   theta0 34 theta1  -theta1 78 -theta0
            左后腿 \~**********************************~/右后腿
                                     尾

    如上图所示：
        2和3是前腿内侧、5和8是后退外侧，他们均对应theta0；又2和3电机是对称的而非一致的，因此角度方向要反向一下。
        其它电机同理。
    */
}
/*
* NAME: void CartesianToThetaGamma(void)
* FUNCTION: 笛卡尔坐标（足端直角坐标系）转换到角度坐标（电机角度坐标）， 也就是将xy转换成theta，运动学逆过程（核心函数）
* 数据流：改变theta0和theta1的值
* 隐藏入口参数（即利用全局变量）：x，y目标坐标
*/
void CartesianToTheta(void)
{
    float L=0,N=0;
    double M=0;
    float A1=0,A2=0;
    //所需腿长计算及腿长限位
    L=sqrt(pow(x,2) + pow(y,2));
    if(L<LegLenthMin) L=LegLenthMin;
    else if(L>LegLenthExtremeMax) L=LegLenthExtremeMax;
    //根据腿长计算“中间角度”N和M。总体角度范围-180°~180°。
    N = asin(y / L) * 180.0 / PI;////角度范围为-90°~90°。
    if((x < 0)&&(y > 0)) N = 180 - N;////角度范围为90°~180°
    else if((x < 0)&&(y < 0)) N =-180-N;////角度范围为-180°~-90°
    M=acos(	( pow(L,2)+pow(L1,2)-pow(L2,2) )/(2*L1*L) )*180.0/PI;////绝对的角度大小，角度范围为0°~90°。
    //坐标转换（先不考虑offset，在最终赋值再进行调整）。
    A1=M+N-90;
    A2=90-(N-M);
    //最终确定电机角度。角度范围分别为0°~360°和-360°~0°。
    TargetAngle1=-(A1-90);
    TargetAngle2=-(A2-270);//
    //
    if(reverse_move_flag == 1)//运动反向控制
    {
        TargetAngle1-=360;
        TargetAngle2+=360;
    }

    TargetAngle1 = TargetAngle1 / 180 * 3.1415926535;
    TargetAngle2 = TargetAngle2 / 180 * 3.1415926535;
}
/*
* NAME: SinTrajectory (float t,GaitParams params, float gaitOffset)
* FUNCTION : 正弦轨迹生成器（核心函数），用在CoupledMoveLeg函数中。
* 入口参数：
			t：心跳控制变量，用来体现时间的流逝。我们在上层参数输入时，输入的实际上是：tt=times*5/1000;即tt约每5ms变化5/1000即，0.005。
			GaitParams：步态控制
			gaitOffset：相位差，用于构成不同步态的核心参数！！
			leg_diretion：代表腿的前进或后退
			angle：
* 待优化：初始相位的控制比较重要，决定了起步的时候是否平稳！
*/
void SinTrajectory (float t,GaitParams params, float gaitOffset,float leg_diretion,float angle,int LegId)
{
//t=times*5/1000，即每1s变化1
    //获取正弦函数的所要配置的参数
    float stanceHeight = params.stance_height;////狗底盘离地高度
    float downAMP = params.down_amp;////负峰值
    float upAMP = params.up_amp;////正峰值
    float flightPercent = params.flight_percent;////摆动相占比
    float stepLength = params.step_length ;////步长
    float FREQ = params.freq;////频率
    if(leg_diretion<0) stepLength = -stepLength;////方向控制
    //原始坐标初始化
    float x0=0,y0=0;
    /******相位（时间、周期循环）控制******/
    //相位时间累计(要想实现不同腿不同频率，就不能共用一个这个，而应该将其变为腿部参数特征)。
    //由于t每次进入函数变化至少0.005，因此FREQ理论上要小于200。否则，p的变化量将大于等于1，从而导致运动出错。
    //例如当FREQ=1时，每经过1s，t变化1，而p刚好变化1，故此时频率为1Hz，当FREQ=n时，频率显然就为nHz。故频率最大为200Hz。
    //建议频率不要过大，因为频率越大意味着采样点数越少。而实际上我们不需要那么高频率，应将频率限制在0-5开区间范围内。
    static float p = 0,prev_t = 0;//频率*时间变化量即为相位变化量。p每次变化所经历的时间是固定的5ms，
    // 但我们可以通过改变每次变化的大小来间接代替变化频率。FREQ越大，单次变化的就越大。
    p += FREQ * (t - prev_t);//
    float gp = fmod((p+gaitOffset),1.0);////该函数返回 x/y 的余数，除1.0表明取小数部分，即将gp限制在0-1范围内。
    prev_t = t;////将当前t值保存下来。
    /******正弦轨迹生成******/
    //足尖摆动相
    if (gp <= flightPercent) // //gp将从gaitOffset开始，因此当gaitOffset大于flightPercent时，将直接转到支撑相。
    {
        x0 = (gp/flightPercent)*stepLength - stepLength/2.0f;////从-stepLength/2到+stepLength/2，移动时间不随stepLength改变，故stepLength越大实际移动速度越快。
        y0 = -upAMP*sin(PI*gp/flightPercent) + stanceHeight;////围绕stanceHeight为基础进行正弦波动。同样是upAMP越大移动速度越快。
    }
        //足尖支撑相
    else ////摆动总是从正弦轨迹的起始位置处执行。
    {
        float percentBack = (gp-flightPercent)/(1.0f-flightPercent);//percentBack与(gp/flightPercent)是一个道理
        x0 = -percentBack*stepLength + stepLength/2.0f;////一般来说，首次进入时总是从stepLength/2开始，然后之后就向后运动。
        y0 = downAMP*sin(PI*percentBack) + stanceHeight;//
    }
    ////经过坐标系转换后得到最终结果(angle目前都是0，从而x=x0，y=y0)
    x =  cos(angle*PI/180)*x0 + sin(angle*PI/180)*y0;
    y = -sin(angle*PI/180)*x0 + cos(angle*PI/180)*y0;

}
/*
* NAME: CoupledMoveLeg
* FUNCTION :狗腿运动的耦合控制，用在gait_detached函数中
* 入口参数:
	GaitParams 一只腿的参数
	gait_offset 姿态修正量
	leg_diretion 腿的前后方向，1为前进方向，-1为后退方向
	LegId 腿号
	angle 角度
*/
void CoupledMoveLeg(float t, GaitParams params,float gait_offset, float leg_direction, int LegId, float angle)
{
    SinTrajectory(t,params,gait_offset,leg_direction,angle,LegId);//足端正弦轨迹生成器
    CartesianToTheta();//笛卡尔坐标转换到角度坐标
    SetCoupledThetaPosition(LegId);//发送数据给电机驱动函数
}
/*
* NAME: gait_detached
* FUNCTION : 四腿分离的腿部控制函数
* 入口参数：
	DetachedParam 所要实现的步态信息，该结构体包含每个退的步态信息
	legx_offset   每条腿的相位延时，如trot步态2、3腿有0.5的延时，而walk步态则
	legx_direction 走的方向
*/
void gait_detached(	DetachedParam d_params,
                       float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset,
                       float leg0_direction, float leg1_direction,float leg2_direction, float leg3_direction)
{
    float tt=times*5/1000;
    CoupledMoveLeg(tt,d_params.detached_params_0,leg0_offset,leg0_direction,0,step_angle[0]);
    CoupledMoveLeg(tt,d_params.detached_params_1,leg1_offset,leg1_direction,1,step_angle[1]);
    CoupledMoveLeg(tt,d_params.detached_params_2,leg2_offset,leg2_direction,2,step_angle[2]);
    CoupledMoveLeg(tt,d_params.detached_params_3,leg3_offset,leg3_direction,3,step_angle[3]);
}
//调试老狗使用3508电机需要做的减速比转换，宇树电机用不到
/*
void Output_Angle(void)
{
    for(int i=0;i<8;i++)
    {
        Angle_Output[i] = AngleWant_MotorX[i] / 360 * 8192 * 19;
    }
}
*/
//赋目标速度值
void Get_Target(int theta1,int theta2)
{
    TargetAngle1 = theta1;
    TargetAngle2 = theta2;
}


/*
 * 为了不影响代码阅读，将正弦生成曲线特征结构体数组放置在最后，通过Change_SinStateDetachedParams(DetachedParam *State,int8_t id,float stance_height,float step_length,
 * float up_amp,float down_amp,float flight_percent,float freq);
 * 函数对特定使用的正弦生成曲线特征量进行更改，可以像修改pid的函数直接进入步态任务中，
 * 下方数组的初始化将更多地作为一个调好的存储/暂存区域，可以省去git的步骤
 */
DetachedParam state_detached_params[StatesMaxNum] = {


        {
                0,//转弯（在转弯函数中会调整该步态以实现转弯）
                {18.0f, 6.25f, 2.0f, 1.5f, 0.4f, 4.0f},
                {18.0f, 6.25f, 2.0f, 1.5f, 0.4f, 4.0f},
                {18.0f, 6.25f, 2.0f, 1.5f, 0.4f, 4.0f},// 6个参数变量为stance_height; step_length; up_amp; down_amp; flight_percent; freq
                {18.0f, 6.25f, 2.0f, 1.5f, 0.4f, 4.0f}
//                0,//转弯（在转弯函数中会调整该步态以实现转弯）
//                {18.0f, 6.25f, 1.0f, 1.0f, 0.25f, 4.0f},
//                {18.0f, 6.25f, 1.0f, 1.0f, 0.25f, 4.0f},
//                {18.0f, 6.25f, 1.0f, 1.0f, 0.25f, 4.0f},// 6个参数变量为stance_height; step_length; up_amp; down_amp; flight_percent; freq
//                {18.0f, 6.25f, 1.0f, 1.0f, 0.25f, 4.0f}
        },
        {

            1,//大步Trot（快速）,现在最高点y轴坐标应该大于15，最大不超过32
            {17.0f, 20.0f,  2.5f, 0.2f, 0.35f, 3.0f},
            {17.0f, 20.0f,  2.5f, 0.2f, 0.35f, 3.0f},
            {17.0f, 20.0f,  2.5f, 0.2f, 0.35f, 3.0f},
            {17.0f, 20.0f,  2.5f, 0.2f, 0.35f, 3.0f}



        },
        {
            2,//原地踏步//出现多种步态基高差距过大是会失效
            {16.0f, 0.0f,  0.8f, 10.0f, 0.25f, 3.0f},
            {16.0f, 0.0f,  0.8f, 10.0f, 0.25f, 3.0f},
            {16.0f, 0.0f,  0.8f, 10.0f, 0.25f, 3.0f},
            {16.0f, 0.0f,  0.8f, 10.0f, 0.25f, 3.0f}
        },
        {
            3,//Walk步态（没有调好）
            {15.0f, 20.0f,  5.0f, 5.0f, 0.18f, 2.1f},
            {15.0f, 20.0f,  5.0f, 5.0f, 0.18f, 2.1f},
            {15.0f, 20.0f,  5.0f, 5.0f, 0.18f, 2.1f},
            {15.0f, 20.0f,  5.0f, 5.0f, 0.18f, 2.1f}
        },
        {
            4,//小步Trot（稳速）
            {20.0f, 15.0f,  1.5f, 1.0f, 0.18f, 2.0f},
            {20.0f, 15.0f,  1.5f, 1.0f, 0.18f, 2.0f},
            {20.0f, 15.0f,  1.5f, 1.0f, 0.18f, 2.0f},
            {20.0f, 15.0f,  1.5f, 1.0f, 0.18f, 2.0f}

        },
        {

                5,//大步Trot（快速）,现在最高点y轴坐标应该大于15，最大不超过32
                {20.0f, 22.5f,  2.5f, 1.2f, 0.3f, 5.5f},
                {20.0f, 22.5f,  2.5f, 1.2f, 0.3f, 5.5f},
                {20.0f, 22.5f,  2.5f, 1.2f, 0.3f, 5.5f},
                {20.0f, 22.5f,  2.5f, 1.2f, 0.3f, 5.5f}



        },
};


//新版IMU控制基于一个复制的基准进行IMU调节，避免了各种非归零造成的BUG。
void YawControl(float yaw_set,DetachedParam *State_Detached_Params,int direction)
{
    float normal_step_left = 0,normal_step_right = 0;
    float f_left = 0,f_right = 0;
    if(IMU_Control_Flag)
    {
        /*******IMUのPID相关*******/
        //PID目标设定（一般都是0，除了Pitch有时要求它是一定角度）
        SetPoint_IMU(&Yaw_PID_Loop,yaw_set);
        PID_PosLocCalc(&Yaw_PID_Loop,IMU_EulerAngle.EulerAngle[Yaw]);
        if(direction != 1) Yaw_PID_Loop.Out_put = -Yaw_PID_Loop.Out_put;
        /**********步态控制*********/
        //Yaw输出给步长参数
        normal_step_left  = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.step_length - Yaw_PID_Loop.Out_put;//左腿步长增加
        normal_step_right = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.step_length + Yaw_PID_Loop.Out_put;//右腿步长减小
        f_left = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.freq - Yaw_PID_Loop.Out_put;//左腿步长增加
        f_right = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.freq + Yaw_PID_Loop.Out_put;//左腿步长增加
        //步长限幅
        if(normal_step_right > StepLenthMax)
            normal_step_right = StepLenthMax;
        else if(normal_step_right < StepLenthMin)
            normal_step_right = StepLenthMin;

        if(normal_step_left > StepLenthMax)
            normal_step_left = StepLenthMax;
        else if(normal_step_left < StepLenthMin)
            normal_step_left = StepLenthMin;

        if(f_right > freMAX)
            f_right = freMAX;
        else if(f_right < freMIN)
            f_right = freMIN;
//
        if(f_left > freMAX)
            f_left = freMAX;
        else if(f_left < freMIN)
            f_left = freMIN;

        //最终赋值（前面的步长限幅保证了步长参数总是在合理的范围内而不会疯掉，从根本上解决了出现IMU控制坏掉BUG的可能性）
        State_Detached_Params->detached_params_0.step_length = normal_step_left;
        State_Detached_Params->detached_params_1.step_length = normal_step_left;

        State_Detached_Params->detached_params_0.freq = f_left;
        State_Detached_Params->detached_params_1.freq = f_left;

        State_Detached_Params->detached_params_2.step_length = normal_step_right;
        State_Detached_Params->detached_params_3.step_length = normal_step_right;

        State_Detached_Params->detached_params_2.freq = f_right;
        State_Detached_Params->detached_params_3.freq = f_right;
    }
    else if(visual_control_flag)
    {
        /*******IMUのPID相关*******/
        //PID目标设定（一般都是0，除了Pitch有时要求它是一定角度）
        SetPoint_IMU(&Yaw_PID_Loop,yaw_set);
        SetPoint_Visual(&VisualLoop,MidPoint);
        PID_PosLocCalc(&Yaw_PID_Loop,IMU_EulerAngle.EulerAngle[Yaw]);
        PID_PosLocCalc(&VisualLoop,visual.offset);
        if(direction != 1) Yaw_PID_Loop.Out_put = -Yaw_PID_Loop.Out_put;
        /**********步态控制*********/
        //Yaw输出给步长参数
        normal_step_left  = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.step_length - Yaw_PID_Loop.Out_put - VisualLoop.Out_put;//左腿步长增加
        normal_step_right = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.step_length + Yaw_PID_Loop.Out_put + VisualLoop.Out_put;//右腿步长减小
        f_left = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.freq - Yaw_PID_Loop.Out_put - VisualLoop.Out_put;//左腿步长增加
        f_right = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.freq + Yaw_PID_Loop.Out_put + VisualLoop.Out_put;//左腿步长增加
        //步长限幅
        if(normal_step_right > StepLenthMax)
            normal_step_right = StepLenthMax;
        else if(normal_step_right < StepLenthMin)
            normal_step_right = StepLenthMin;

        if(normal_step_left > StepLenthMax)
            normal_step_left = StepLenthMax;
        else if(normal_step_left < StepLenthMin)
            normal_step_left = StepLenthMin;

        if(f_right > freMAX)
            f_right = freMAX;
        else if(f_right < freMIN)
            f_right = freMIN;
//
        if(f_left > freMAX)
            f_left = freMAX;
        else if(f_left < freMIN)
            f_left = freMIN;

        //最终赋值（前面的步长限幅保证了步长参数总是在合理的范围内而不会疯掉，从根本上解决了出现IMU控制坏掉BUG的可能性）
        State_Detached_Params->detached_params_0.step_length = normal_step_left;
        State_Detached_Params->detached_params_1.step_length = normal_step_left;

        State_Detached_Params->detached_params_0.freq = f_left;
        State_Detached_Params->detached_params_1.freq = f_left;

        State_Detached_Params->detached_params_2.step_length = normal_step_right;
        State_Detached_Params->detached_params_3.step_length = normal_step_right;

        State_Detached_Params->detached_params_2.freq = f_right;
        State_Detached_Params->detached_params_3.freq = f_right;
    }
}

//直接设置所需x，y位置进行电机控制
void SetCoupledCartesianPosition(int LegId,float x_want,float y_want)
{
    x=x_want;
    y=y_want;
    CartesianToTheta();
    SetCoupledThetaPosition(LegId);
}

//所有腿的直角坐标控制
void SetCartesianPositionAll_Delay(float x_want,float y_want,uint16_t delaytime)
{
    SetCoupledCartesianPosition(0,x_want,y_want);
    SetCoupledCartesianPosition(1,x_want,y_want);
    SetCoupledCartesianPosition(2,x_want,y_want);
    SetCoupledCartesianPosition(3,x_want,y_want);
    osDelay(delaytime);
}

//所有腿的极坐标控制
void SetPolarPositionAll_Delay(float polar_angle,float polar_diameter,uint16_t delaytime)
{
    float x_want,y_want;
    if(polar_angle>=0)
    {
        x_want = -polar_diameter*cos(polar_angle*PI/180);
        y_want =  polar_diameter*sin(polar_angle*PI/180);
    }
    else
    {
        x_want =  polar_diameter*cos(polar_angle*PI/180);
        y_want = -polar_diameter*sin(polar_angle*PI/180);
    }
    SetCartesianPositionAll_Delay(x_want,y_want,delaytime);
}
void ReverseMoveOpen(void)
{
    reverse_move_flag=1;
}
//关闭运动反向
void ReverseMoveClose(void)
{
    reverse_move_flag=0;
}