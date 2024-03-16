//
// Created by 1 on 2023-11-07.
//

#ifndef MY_SCUDOG_ATTITUDE_TASK_H
#define MY_SCUDOG_ATTITUDE_TASK_H

#include "Attitude_Slove.h"
#include "pid.h"

//ǰ�����˶���
#define Forward 1
#define Backward -1
//״̬����������
#define StatesMaxNum 20

#define position 1
#define Zero 0
/*******ͨ�ã�GP����̬�����*******/
enum GPStates // ��Ŵ�С��Ӧ����253��254����״̬��������������״̬������IUM��PID���BUG��255�������ˣ���һλ���ã�
{
    //�������0-19��
    STOP=0,              //�ڵ�ǰλ��ֹͣ�����ģ�
    HALT =1,             //���������ã�
    KNEEL  =2,			 //���£���������
    END  =3,             //��ʼ����̬,װ�������ã�
    ROTAT_LEFT=6,      	 //ԭ����ת�����ã�
    ROTAT_RIGHT=7,     	 //ԭ����ת�����ã�
    TURN_LEFT= 8,        //���
    TURN_RIGHT=  9,      //�ҹ�
    MARCH =10,			 //��ǰ�н�����
    MARCH_BACK=11,       //����н�����
    //������19Ϊ������̬����
    //��Ծ���20-29��
    Jump_Standard=20,        //��׼��������,�߶Ⱥ�Զ�����У�
    Jump_High = 21,	//���ߣ�Զ�Ⱥ�С
    Jump_Far = 22,  //��Զ���߶Ⱥ�С�����״�
    Jump_Step = 23, //���߲�Զ���Ƚ�����
    Jump_Leap = 24, //��Խ����������
    BACKFLIP = 25,  //ǰ�շ�����ɽԽ�룬һ����ǰ
    //������29Ϊ������Ծ����
    //�������30-49��
    SQUAT = 30,         //���£���̬����
    SHAKEHAND = 31,     //���֣�������ò
    STRETCH = 32,       //��չ�������
    MARKINGTIME = 33,   //̤���������˶�
    LIEDOWN = 34,       //��ƽ��������
    SWAY = 35,          //ҡ�ڣ����һζ�
    PLANE = 36,         //�ٵأ����鼱��
    HOMOLATERAL = 37,   //˳�գ��ճ�����
    REPOSITION  = 39,   //��λ���ع�����
    NIGHTVISION = 40,   //ҹ�ӣ�����ͷ������ҹ������ͷ��Ȼ����û�У�
    WARN = 41,   		//���䣬��������
    //�������100-254��
    //������
    NONECMD = 254,      //��״̬����
};
/*******���⣨GP����̬�����*******/
enum DPStates
{
    //ר�����50-99��
    //��������
    RACING=50,          //����״̬
    PEBBLEROAD = 51,	//����ʯ·״̬
    SEESAW = 52,		//�����ΰ�״̬
    BRIDGE = 53,		//��˫Ŀ��״̬
    STAIR = 54,         //������״̬
    HIGHHURDLES=55,     //������״̬
    CALANDRIA = 56,		//���Ź�״̬
    CHECK = 57,		    //��¼״̬
    //��������
    TEST = 59,          //���Բ�̬
    //��̬��������
    TROT=60,            //�첽
    WALK=61,			//����
    PACE=62,            //�ⲽ
    CANTER = 63,		//�ܲ�
    GALLOP = 64,        //�ɳ�
    SMALLSTEPS = 65,    //�鲽
    //��ר������
    NONE = 254,
};
//��������
enum LegNames
{
    Leg_Front_Left = 0,
    Leg_Front_Right = 2,
    Leg_Back_Left = 1,
    Leg_Back_Right = 3,
    Leg_Front = 5,
    Leg_Back = 6,
};
extern enum GPStates gpstate;
extern enum DPStates dpstate;
extern float TargetAngle1,TargetAngle2;
extern int Global_IMU_Control;
extern float Direct;
extern float TargetAngle;

void MarkingTime(void);
void StandUp_Posture(void);
void Test_Move(void);
void Trot(float direction,int8_t kind);
void Walk(float direction,uint8_t speed);
void LieDown_Posture(void);
void Dog_Posture(void);
void EndPosture(void);
void Turn(int state_flag,int speed_flag);
void Handshake(void);
void StretchPosture(void);
void SquatPosture(void);
void FBwAaLitAir(void);
void Race_Competition(void);

#endif //MY_SCUDOG_ATTITUDE_TASK_H
