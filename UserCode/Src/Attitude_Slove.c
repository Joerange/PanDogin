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

//���ڸ����Ϸ�״̬������Ϊ�����׼��
DetachedParam StateDetachedParams_Copy[StatesMaxNum] = {0};
//����ʱ�����ı����ɵĹ켣����
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
            AngleWant_MotorX[1]=TargetAngle1-offset_front_0;
            AngleWant_MotorX[2]=TargetAngle2-offset_front_1;//+10.0f
            break;
        case 1:
            AngleWant_MotorX[3]=TargetAngle1-offset_back_0;//+5.0f
            AngleWant_MotorX[4]=TargetAngle2-offset_back_1;
            break;
        case 2:
            AngleWant_MotorX[5]=-TargetAngle2+offset_front_1;//-4.0f
            AngleWant_MotorX[6]=-TargetAngle1+offset_front_0;
            break;
        case 3:
            AngleWant_MotorX[7]=-TargetAngle2+offset_back_1;
            AngleWant_MotorX[8]=-TargetAngle1+offset_back_0;
            break;
        default:
            break;
    }

    //ע��Ƕȸ�ֵ���ݲ�ͬ�ĵ��˳���������ͬ��ͬʱҲ�ܻ�е�����װ��Ӱ�졣���иĶ���������ĽǶȵĶ�Ӧ��ϵҲҪ�䡣
    /*
    �������½Ƕ�������Ϣ�����ǿ��Եõ����½��ۣ�
    01��67�ŵ��(�������һ�£���75��23�ŵ�����������һ������ǰ����������෴���ֱ��Ӧ�����Խ��ߡ�

                                     ͷ
            ��ǰ�� /~**********************************~\��ǰ��
                    theta0 12 theta1  -theta1 56 -theta0
                     leftleg(usart1)	rightleg(usart2)
                   -theta1 34 -theta0  theta0 78 theta1
            ����� \~**********************************~/�Һ���
                                     β

    ����ͼ��ʾ��
        2��3��ǰ���ڲࡢ5��8�Ǻ�����࣬���Ǿ���Ӧtheta0����2��3����ǶԳƵĶ���һ�µģ���˽Ƕȷ���Ҫ����һ�¡�
        �������ͬ��
    */
}
/*
* NAME: void CartesianToThetaGamma(void)
* FUNCTION: �ѿ������꣨���ֱ������ϵ��ת�����Ƕ����꣨����Ƕ����꣩�� Ҳ���ǽ�xyת����theta���˶�ѧ����̣����ĺ�����
* ���������ı�theta0��theta1��ֵ
* ������ڲ�����������ȫ�ֱ�������x��yĿ������
*/
void CartesianToTheta(void)
{
    float L=0,N=0;
    double M=0;
    float A1=0,A2=0;
    //�����ȳ����㼰�ȳ���λ
    L=sqrt(pow(x,2) + pow(y,2));
    if(L<LegLenthMin) L=LegLenthMin;
    else if(L>LegLenthExtremeMax) L=LegLenthExtremeMax;
    //�����ȳ����㡰�м�Ƕȡ�N��M������Ƕȷ�Χ-180��~180�㡣
    N = asin(y / L) * 180.0 / PI;////�Ƕȷ�ΧΪ-90��~90�㡣
    if((x < 0)&&(y > 0)) N = 180 - N;////�Ƕȷ�ΧΪ90��~180��
    else if((x < 0)&&(y < 0)) N =-180-N;////�Ƕȷ�ΧΪ-180��~-90��
    M=acos(	( pow(L,2)+pow(L1,2)-pow(L2,2) )/(2*L1*L) )*180.0/PI;////���ԵĽǶȴ�С���Ƕȷ�ΧΪ0��~90�㡣
    //����ת�����Ȳ�����offset�������ո�ֵ�ٽ��е�������
    A1=M+N-90;
    A2=90-(N-M);
    //����ȷ������Ƕȡ��Ƕȷ�Χ�ֱ�Ϊ0��~360���-360��~0�㡣
    TargetAngle1=-(A1-90);
    TargetAngle2=-(A2-270);//
    //
    if(reverse_move_flag == 1)//�˶��������
    {

        TargetAngle1-=360;
        TargetAngle2+=360;
    }

    TargetAngle1 = TargetAngle1 / 180 * 3.1415926535;
    TargetAngle2 = TargetAngle2 / 180 * 3.1415926535;
}
/*
* NAME: SinTrajectory (float t,GaitParams params, float gaitOffset)
* FUNCTION : ���ҹ켣�����������ĺ�����������CoupledMoveLeg�����С�
* ��ڲ�����
			t���������Ʊ�������������ʱ������š��������ϲ��������ʱ�������ʵ�����ǣ�tt=times*5/1000;��ttԼÿ5ms�仯5/1000����0.005��
			GaitParams����̬����
			gaitOffset����λ����ڹ��ɲ�ͬ��̬�ĺ��Ĳ�������
			leg_diretion�������ȵ�ǰ�������
			angle��
* ���Ż�����ʼ��λ�Ŀ��ƱȽ���Ҫ���������𲽵�ʱ���Ƿ�ƽ�ȣ�
*/
void SinTrajectory (float t,GaitParams params, float gaitOffset,float leg_diretion,float angle,int LegId)
{
    //t=times*5/1000����ÿ1s�仯1
    //��ȡ���Һ�������Ҫ���õĲ���
    float stanceHeight = params.stance_height;////��������ظ߶�
    float downAMP = params.down_amp;////����ֵ
    float upAMP = params.up_amp;////����ֵ
    float flightPercent = params.flight_percent;////�ڶ���ռ��
    float stepLength = params.step_length ;////����
    float FREQ = params.freq;////Ƶ��
    if(leg_diretion<0) stepLength = -stepLength;////�������
    //ԭʼ�����ʼ��
    float x0=0,y0=0;
    /******��λ��ʱ�䡢����ѭ��������******/
    //��λʱ���ۼ�(Ҫ��ʵ�ֲ�ͬ�Ȳ�ͬƵ�ʣ��Ͳ��ܹ���һ���������Ӧ�ý����Ϊ�Ȳ���������)��
    //����tÿ�ν��뺯���仯����0.005�����FREQ������ҪС��200������p�ı仯�������ڵ���1���Ӷ������˶�����
    //���統FREQ=1ʱ��ÿ����1s��t�仯1����p�պñ仯1���ʴ�ʱƵ��Ϊ1Hz����FREQ=nʱ��Ƶ����Ȼ��ΪnHz����Ƶ�����Ϊ200Hz��
    //����Ƶ�ʲ�Ҫ������ΪƵ��Խ����ζ�Ų�������Խ�١���ʵ�������ǲ���Ҫ��ô��Ƶ�ʣ�Ӧ��Ƶ��������0-5�����䷶Χ�ڡ�
    static float p = 0,prev_t = 0;//Ƶ��*ʱ��仯����Ϊ��λ�仯����pÿ�α仯��������ʱ���ǹ̶���5ms��
    // �����ǿ���ͨ���ı�ÿ�α仯�Ĵ�С����Ӵ���仯Ƶ�ʡ�FREQԽ�󣬵��α仯�ľ�Խ��
    p += FREQ * (t - prev_t);//
    float gp = fmod((p+gaitOffset),1.0);////�ú������� x/y ����������1.0����ȡС�����֣�����gp������0-1��Χ�ڡ�
    prev_t = t;////����ǰtֵ����������
    /******���ҹ켣����******/
    //���ڶ���
    if (gp <= flightPercent) // //gp����gaitOffset��ʼ����˵�gaitOffset����flightPercentʱ����ֱ��ת��֧���ࡣ
    {
        x0 = (gp/flightPercent)*stepLength - stepLength/2.0f;////��-stepLength/2��+stepLength/2���ƶ�ʱ�䲻��stepLength�ı䣬��stepLengthԽ��ʵ���ƶ��ٶ�Խ�졣
        y0 = -upAMP*sin(PI*gp/flightPercent) + stanceHeight;////Χ��stanceHeightΪ�����������Ҳ�����ͬ����upAMPԽ���ƶ��ٶ�Խ�졣
    }
    //���֧����
    else ////�ڶ����Ǵ����ҹ켣����ʼλ�ô�ִ�С�
    {
        float percentBack = (gp-flightPercent)/(1.0f-flightPercent);//percentBack��(gp/flightPercent)��һ������
        x0 = -percentBack*stepLength + stepLength/2.0f;////һ����˵���״ν���ʱ���Ǵ�stepLength/2��ʼ��Ȼ��֮�������˶���
        y0 = downAMP*sin(PI*percentBack) + stanceHeight;//
    }
    ////��������ϵת����õ����ս��(angleĿǰ����0���Ӷ�x=x0��y=y0)
    x =  cos(angle*PI/180)*x0 + sin(angle*PI/180)*y0;
    y = -sin(angle*PI/180)*x0 + cos(angle*PI/180)*y0;
}
/*
* NAME: CoupledMoveLeg
* FUNCTION :�����˶�����Ͽ��ƣ�����gait_detached������
* ��ڲ���:
	GaitParams һֻ�ȵĲ���
	gait_offset ��̬������
	leg_diretion �ȵ�ǰ����1Ϊǰ������-1Ϊ���˷���
	LegId �Ⱥ�
	angle �Ƕ�
*/
void CoupledMoveLeg(float t, GaitParams params,float gait_offset, float leg_direction, int LegId, float angle)
{
    SinTrajectory(t,params,gait_offset,leg_direction,angle,LegId);//������ҹ켣������
    CartesianToTheta();//�ѿ�������ת�����Ƕ�����
    SetCoupledThetaPosition(LegId);//�������ݸ������������
}
/*
* NAME: gait_detached
* FUNCTION : ���ȷ�����Ȳ����ƺ���
* ��ڲ�����
	DetachedParam ��Ҫʵ�ֵĲ�̬��Ϣ���ýṹ�����ÿ���˵Ĳ�̬��Ϣ
	legx_offset   ÿ���ȵ���λ��ʱ����trot��̬2��3����0.5����ʱ����walk��̬��
	legx_direction �ߵķ���
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
//�����Ϲ�ʹ��3508�����Ҫ���ļ��ٱ�ת������������ò���
/*
void Output_Angle(void)
{
    for(int i=0;i<8;i++)
    {
        Angle_Output[i] = AngleWant_MotorX[i] / 360 * 8192 * 19;
    }
}
*/
//��Ŀ���ٶ�ֵ
void Get_Target(int theta1,int theta2)
{
    TargetAngle1 = theta1;
    TargetAngle2 = theta2;
}


/*
 * Ϊ�˲�Ӱ������Ķ����������������������ṹ��������������ͨ��Change_SinStateDetachedParams(DetachedParam *State,int8_t id,float stance_height,float step_length,
 * float up_amp,float down_amp,float flight_percent,float freq);
 * �������ض�ʹ�õ����������������������и��ģ��������޸�pid�ĺ���ֱ�ӽ��벽̬�����У�
 * �·�����ĳ�ʼ�����������Ϊһ�����õĴ洢/�ݴ����򣬿���ʡȥgit�Ĳ���
 */
DetachedParam state_detached_params[StatesMaxNum] = {

        {
                0,//��MARCH
                //2022.7.14��̬
                {14.9, 14.29, 2.5, 1.30, 0.17, 2.0},
                {14.9, 14.29, 2.5, 1.30, 0.17, 2.0},
                {14.9, 14.29, 2.5, 1.30, 0.17, 2.0},// 6����������Ϊstance_height; step_length; up_amp; down_amp; flight_percent; freq
                {14.9, 14.29, 2.5, 1.30, 0.17, 2.0}
        },

        {
                1,//WALK
                {15.0, 14.00, 2.0, 1.30, 0.125, 1.4},//walk��̬����Ҫ��flight_percent�����ڶ��ࣩ��0.25-0��Χ���ڣ�����ȡ0.2���ɳ���ȡ0.125���ԡ�
                {15.0, 14.00, 2.0, 1.30, 0.125, 1.4},
                {15.0, 14.00, 2.0, 1.30, 0.125, 1.4},
                {15.0, 14.00, 2.0, 1.30, 0.125, 1.4}
        },

        {
                2,//���ΰ�   û��ǿ�ƣ����ǿ�PID����
                {14.0, 14.0, 2.00, 2.00, 0.25, 0.5}, //С���ӿ�������ȥ
                {14.0, 14.0, 2.00, 2.00, 0.25, 0.5},
                {14.0, 14.0, 2.00, 2.00, 0.25, 0.5},// 6����������Ϊstance_height; step_length; up_amp; down_amp; flight_percent; freq
                {14.0, 14.0, 2.00, 2.00, 0.25, 0.5}
        },

        {   //���ΰ�  �������֮ǰ��������ʱ�ǿ�������ȥ�ģ�ǿ����ǰ���ȵĸ߶Ȳ�һ����������PID��
                3,
                {14.0, 14.0, 3.00, 2.00, 0.25, 1.0}, //С���ӿ�������ȥ
                {19.0, 14.0, 3.00, 2.00, 0.25, 1.0},
                {14.0, 14.0, 3.00, 2.00, 0.25, 1.0},// 6����������Ϊstance_height; step_length; up_amp; down_amp; flight_percent; freq
                {19.0, 14.0, 3.00, 2.00, 0.25, 1.0}
        },

        {
                4, //��ʯ·
                {16.0, 14.0, 2.50, 2.00, 0.25, 2.0},
                {16.0, 14.0, 2.50, 2.00, 0.25, 2.0},
                {16.0, 14.0, 2.50, 2.00, 0.25, 2.0},// 6����������Ϊstance_height; step_length; up_amp; down_amp; flight_percent; freq
                {16.0, 14.0, 2.50, 2.00, 0.25, 2.0}
        },

        {
                5,//����
//		/*ʹ��Trot��̬ʱ�Ĳ���*/
//        {18.0, 14.0, 3.00, 1.50, 0.25, 2.0},
//        {18.0, 14.0, 3.00, 1.50, 0.25, 2.0},
//        {18.0, 14.0, 3.00, 1.50, 0.25, 2.0},// 6����������Ϊstance_height; step_length; up_amp; down_amp; flight_percent; freq
//        {18.0, 14.0, 3.00, 1.50, 0.25, 2.0}
                /*ʹ��Gallop��̬ʱ�Ĳ���*/
                {18.0, 14.0, 3.00, 1.50, 0.25, 2.0},
                {18.0, 14.0, 3.00, 1.50, 0.25, 2.0},
                {18.0, 14.0, 3.00, 1.50, 0.25, 2.0},// 6����������Ϊstance_height; step_length; up_amp; down_amp; flight_percent; freq
                {18.0, 14.0, 3.00, 1.50, 0.25, 2.0}
        },

        {
                6,//�ⲽ
                {18.0, 0.0, 3.00, 1.50, 0.25, 2.0},
                {18.0, 0.0, 3.00, 1.50, 0.25, 2.0},
                {18.0, 0.0, 3.00, 1.50, 0.25, 2.0},// 6����������Ϊstance_height; step_length; up_amp; down_amp; flight_percent; freq
                {18.0, 0.0, 3.00, 1.50, 0.25, 2.0}
        },

        {
                7,//�����߶�ľ�ŵĲ�̬
//        {14.9, 6.25, 3.7, 1.30, 0.35, 3.4},
//        {14.9, 6.25, 3.7, 1.30, 0.35, 3.4},
//        {14.9, 6.25, 3.7, 1.30, 0.35, 3.4},// 6����������Ϊstance_height; step_length; up_amp; down_amp; flight_percent; freq
//        {14.9, 6.25, 3.7, 1.30, 0.35, 3.4}
//        {18, 25, 6.7, 1.30, 0.35, 1.5},
//        {18, 25, 6.7, 1.30, 0.35, 1.5},
//        {18, 25, 6.7, 1.30, 0.35, 1.5},// 6����������Ϊstance_height; step_length; up_amp; down_amp; flight_percent; freq
//        {18, 25, 6.7, 1.30, 0.35, 1.5}
//        {14, 15, 3.7, 1.30, 0.35, 1.5},//�״γ��Կ�ͨ����ľ�ţ�����Ҫ���ٶȱȽ�����
//        {14, 15, 3.7, 1.30, 0.35, 1.5},
//        {14, 15, 3.7, 1.30, 0.35, 1.5},// 6����������Ϊstance_height; step_length; up_amp; down_amp; flight_percent; freq
//        {14, 15, 3.7, 1.30, 0.35, 1.5}
//        {14, 15, 2.7, 1.30, 0.35, 0.8},//�״γ��Կ�ͨ����ľ�ţ�����Ҫ���ٶȱȽ�����
//        {14, 15, 2.7, 1.30, 0.35, 0.8},
//        {14, 15, 2.7, 1.30, 0.35, 0.8},// 6����������Ϊstance_height; step_length; up_amp; down_amp; flight_percent; freq
//        {14, 15, 2.7, 1.30, 0.35, 0.8}
                {12, 13, 1.3, 1.30, 0.32, 0.8},//�״γ��Կ�ͨ����ľ�ţ�����Ҫ���ٶȱȽ�����
                {12, 13, 1.3, 1.30, 0.32, 0.8},
                {12, 13, 1.3, 1.30, 0.32, 0.8},// 6����������Ϊstance_height; step_length; up_amp; down_amp; flight_percent; freq
                {12, 13, 1.3, 1.30, 0.32, 0.8}
        },

        {
                8,//ת�䣨��ת�亯���л�����ò�̬��ʵ��ת�䣩
                {15.0, 6.25, 8.5, 8.0, 0.25, 6.5},
                {15.0, 6.25, 8.5, 8.0, 0.25, 6.5},
                {15.0, 6.25, 8.5, 8.0, 0.25, 6.5},// 6����������Ϊstance_height; step_length; up_amp; down_amp; flight_percent; freq
                {15.0, 6.25, 8.5, 8.0, 0.25, 6.5}
        },

        {
                9,//ң���������û�׼��̬������Trot��
                {15.0, 14.0, 2.50, 1.5, 0.35, 2.0}, // 15.0, 14.0, 2.50, 1.50, 0.35, 2.0(�����ȣ�
                {15.0, 14.0, 2.50, 1.5, 0.35, 2.0}, // 6����������Ϊstance_height; step_length; up_amp; down_amp; flight_percent; freq
                {15.0, 14.0, 2.50, 1.5, 0.35, 2.0},
                {15.0, 14.0, 2.50, 1.5, 0.35, 2.0}
        },

        {
                10,//ң���������ò�̬������walk
                {16.0, 17.00, 3.0, 3.00, 0.20, 0.4},//walk��̬����Ҫ��flight_percent�����ڶ��ࣩ��0.25-0��Χ���ڣ�����ȡ0.2���ɳ���ȡ0.125���ԡ�
                {16.0, 17.00, 3.0, 3.00, 0.20, 0.4},
                {16.0, 17.00, 3.0, 3.00, 0.20, 0.4},
                {16.0, 17.00, 3.0, 3.00, 0.20, 0.4}
        },

        {
                11,//С��Tro
                {15.0, 5.0,  4.0, 3.0, 0.2, 2.0},
                {15.0, 5.0,  4.0, 3.0, 0.2, 2.0},
                {15.0, 5.0,  4.0, 3.0, 0.2, 2.0},
                {15.0, 5.0,  4.0, 3.0, 0.2, 2.0}
        }
};

/*
	���ܣ�����IMU�Ƕ����ݵ���̬����
	float roll_set,float pitch_set,float yaw_set��ŷ���ǣ������������ˮƽ����Ŀ��ֵ��
	GaitParams params������state_detached_params[i].detached_params_j����ĳ�ֲ�̬��ĳ���ȵ���̬���ò�����Ϊһ��״̬��׼��jȡ0���ɣ�ӦΪ�����Ȳ�������һ���ġ�
	paramID����������״̬��׼�����ı����²�̬��Ϣ������state_detached_params[paramID]��ֵ��paramIDӦ����һ���е�iһ����
	���⣺�ú�����ֱ�¶��������Ŀǰ�Ŀ��Ƶ�Ƿȱ����������̬�У������ȵĲ������ö���һ���ġ�
*/
void AttitudeControl(float roll_set,float pitch_set,float yaw_set,DetachedParam *State_Detached_Params,int direction)
{
    float normal_step_left,normal_step_right;
    float normal_stance_0,normal_stance_1,normal_stance_2,normal_stance_3;
    if(IMU_Control_Flag)
    {
        /*******IMU��PID���*******/
        //PIDĿ���趨��һ�㶼��0������Pitch��ʱҪ������һ���Ƕ�;���⻹�п�����Ϊ��΢��Yaw��
        SetPoint_IMU(&Yaw_PID_Loop,yaw_set);
        SetPoint_IMU(&Pitch_PID_Loop,pitch_set);
        SetPoint_IMU(&Roll_PID_Loop,roll_set);
        //PID���㣨���õ��˴��ڽ������IMU_EulerAngle����λ��ʽPID���㣩
        PID_PosLocCalc(&Pitch_PID_Loop,IMU_EulerAngle.EulerAngle[Pitch]);
        PID_PosLocCalc(&Roll_PID_Loop,IMU_EulerAngle.EulerAngle[Roll]);
        PID_PosLocCalc(&Yaw_PID_Loop,IMU_EulerAngle.EulerAngle[Yaw]);
        if(direction != 1)
        {
            Yaw_PID_Loop.Out_put = -Yaw_PID_Loop.Out_put;
        }
        //��������
        if((Yaw_PID_Loop.Out_put<1.0f) && (Yaw_PID_Loop.Out_put>-1.0f)) 	Yaw_PID_Loop.Out_put=0;
        if((Pitch_PID_Loop.Out_put<1.0f) && (Pitch_PID_Loop.Out_put>-1.0f)) Pitch_PID_Loop.Out_put=0;
        if((Roll_PID_Loop.Out_put<1.0f) && (Roll_PID_Loop.Out_put>-1.0f)) 	Roll_PID_Loop.Out_put=0;
        /**********��̬����*********/
        //Yaw�������������
        normal_step_left  = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.step_length - Yaw_PID_Loop.Out_put;//���Ȳ�����С
        normal_step_right = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.step_length + Yaw_PID_Loop.Out_put ;//���Ȳ�������
        //�����޷�
        normal_step_left  = ((normal_step_left>StepLenthMax - 2) ? StepLenthMax- 2 : normal_step_left);
        normal_step_right = ((normal_step_right>StepLenthMax -2) ? StepLenthMax- 2 : normal_step_right);
        normal_step_left  = ((normal_step_left<StepLenthMin - 2) ? StepLenthMin- 2 : normal_step_left);
        normal_step_right = ((normal_step_right<StepLenthMin- 2) ? StepLenthMin- 2 : normal_step_right);
        //���ո�ֵ
        State_Detached_Params->detached_params_0.step_length = normal_step_left;
        State_Detached_Params->detached_params_1.step_length = normal_step_left;
        State_Detached_Params->detached_params_2.step_length = normal_step_right;
        State_Detached_Params->detached_params_3.step_length = normal_step_right;
        //�Ⱥ�0123�ֱ��Ӧ��ǰ�������ǰ���Һ󣬼�1��2��Ӧ���ȣ�3��4��Ӧ���ȡ�ע�����öԣ�������������
        //���ȳ��ȿ��ƣ�����һ��˼·��ֻ������������֤�������������������Ա������̫������ӵ��ڸ�����
        normal_stance_0 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.stance_height + Pitch_PID_Loop.Out_put - Roll_PID_Loop.Out_put;//��+�󣭣���Ϊ������ǰ����ҷ���
        normal_stance_1 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_1.stance_height - Pitch_PID_Loop.Out_put - Roll_PID_Loop.Out_put;
        normal_stance_2 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_2.stance_height + Pitch_PID_Loop.Out_put + Roll_PID_Loop.Out_put;
        normal_stance_3 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_3.stance_height - Pitch_PID_Loop.Out_put + Roll_PID_Loop.Out_put;
        //���ȳ������޿���
        normal_stance_0  = ( (normal_stance_0>=LegLenthMax) ? LegLenthMax : normal_stance_0 );
        normal_stance_1  = ( (normal_stance_1>=LegLenthMax) ? LegLenthMax : normal_stance_1 );
        normal_stance_2  = ( (normal_stance_2>=LegLenthMax) ? LegLenthMax : normal_stance_2 );
        normal_stance_3  = ( (normal_stance_3>=LegLenthMax) ? LegLenthMax : normal_stance_3 );
        //���ȳ������޿���
        normal_stance_0  = ( (normal_stance_0<=LegLenthMin) ? LegLenthMin : normal_stance_0 );
        normal_stance_1  = ( (normal_stance_1<=LegLenthMin) ? LegLenthMin : normal_stance_1 );
        normal_stance_2  = ( (normal_stance_2<=LegLenthMin) ? LegLenthMin : normal_stance_2 );
        normal_stance_3  = ( (normal_stance_3<=LegLenthMin) ? LegLenthMin : normal_stance_3 );
        //Pitch��Roll�����stance_height����
        State_Detached_Params->detached_params_0.stance_height = normal_stance_0;
        State_Detached_Params->detached_params_1.stance_height = normal_stance_1;
        State_Detached_Params->detached_params_2.stance_height = normal_stance_2;
        State_Detached_Params->detached_params_3.stance_height = normal_stance_3;
    }
}

//�°�IMU���ƻ���һ�����ƵĻ�׼����IMU���ڣ������˸��ַǹ�����ɵ�BUG��
void YawControl(float yaw_set,DetachedParam *State_Detached_Params,int direction)
{
    float normal_step_left,normal_step_right;
    if(IMU_Control_Flag)
    {
        /*******IMU��PID���*******/
        //PIDĿ���趨��һ�㶼��0������Pitch��ʱҪ������һ���Ƕȣ�
        SetPoint_IMU(&Yaw_PID_Loop,yaw_set);
        PID_PosLocCalc(&Yaw_PID_Loop,IMU_EulerAngle.EulerAngle[Yaw]);
        if(direction != 1) Yaw_PID_Loop.Out_put = -Yaw_PID_Loop.Out_put;
        /**********��̬����*********/
        //Yaw�������������
        normal_step_left  = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.step_length - Yaw_PID_Loop.Out_put * 200;//���Ȳ�����С
        normal_step_right = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.step_length + Yaw_PID_Loop.Out_put * 200;//���Ȳ�������
        //�����޷�
        normal_step_left  = ((normal_step_left>StepLenthMax_Half)  ? StepLenthMax_Half : normal_step_left);
        normal_step_right = ((normal_step_right>StepLenthMax_Half) ? StepLenthMax_Half : normal_step_right);
        normal_step_left  = ((normal_step_left<StepLenthMin)  ? StepLenthMin : normal_step_left);
        normal_step_right = ((normal_step_right<StepLenthMin) ? StepLenthMin : normal_step_right);
        //���ո�ֵ��ǰ��Ĳ����޷���֤�˲������������ں���ķ�Χ�ڶ����������Ӹ����Ͻ���˳���IMU���ƻ���BUG�Ŀ����ԣ�
        State_Detached_Params->detached_params_0.step_length = normal_step_left;
        State_Detached_Params->detached_params_1.step_length = normal_step_left;
        State_Detached_Params->detached_params_2.step_length = normal_step_right;
        State_Detached_Params->detached_params_3.step_length = normal_step_right;
    }
}