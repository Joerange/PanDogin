//
// Created by 1 on 2023-11-07.
//

#ifndef MY_SCUDOG_ATTITUDE_TASK_H
#define MY_SCUDOG_ATTITUDE_TASK_H

#include "Attitude_Slove.h"
#include "pid.h"

//前进后退定义
#define Forward 1
#define Backward -1
//状态数上限配置
#define StatesMaxNum 20

#define position 1
#define Zero 0
/*******通用（GP）姿态命令表*******/
enum GPStates // 序号大小不应超过253（254是无状态，不可用于其它状态，否则IUM的PID会出BUG。255到极限了，这一位不用）
{
    //基本命令（0-19）
    STOP=0,              //在当前位置停止（核心）
    HALT =1,             //立定（常用）
    KNEEL  =2,			 //坐下，四腿收缩
    END  =3,             //初始结束态,装死（常用）
    ROTAT_LEFT=6,      	 //原地左转（常用）
    ROTAT_RIGHT=7,     	 //原地右转（常用）
    TURN_LEFT= 8,        //左拐
    TURN_RIGHT=  9,      //右拐
    MARCH =10,			 //向前行进命令
    MARCH_BACK=11,       //向后行进命令
    //保留至19为基本步态命令
    //跳跃命令（20-29）
    Jump_Standard=20,        //标准跳（常用,高度和远度适中）
    Jump_High = 21,	//跳高，远度很小
    Jump_Far = 22,  //跳远，高度很小，容易打滑
    Jump_Step = 23, //不高不远，比较折中
    Jump_Leap = 24, //翻越，跳起蜷腿
    BACKFLIP = 25,  //前空翻，翻山越岭，一往无前
    //保留至29为各种跳跃命令
    //补充命令（30-49）
    SQUAT = 30,         //蹲下，憨态可掬
    SHAKEHAND = 31,     //握手，文明礼貌
    STRETCH = 32,       //伸展，舒活筋骨
    MARKINGTIME = 33,   //踏步，热身运动
    LIEDOWN = 34,       //躺平，侧身翻到
    SWAY = 35,          //摇摆，左右晃动
    PLANE = 36,         //刨地，心情急切
    HOMOLATERAL = 37,   //顺拐，日常智障
    REPOSITION  = 39,   //复位，回归正道
    NIGHTVISION = 40,   //夜视（开启头部红外夜视摄像头，然而并没有）
    WARN = 41,   		//警戒，弓下身体
    //其它命令（100-254）
    //无命令
    NONECMD = 254,      //无状态命令
};
/*******特殊（GP）姿态命令表*******/
enum DPStates
{
    //专用命令（50-99）
    //比赛命令
    RACING=50,          //竞速状态
    PEBBLEROAD = 51,	//过卵石路状态
    SEESAW = 52,		//过跷跷板状态
    BRIDGE = 53,		//过双目桥状态
    STAIR = 54,         //过阶梯状态
    HIGHHURDLES=55,     //过高栏状态
    CALANDRIA = 56,		//过排管状态
    CHECK = 57,		    //检录状态
    //调试命令
    TEST = 59,          //调试步态
    //步态调整命令
    TROT=60,            //快步
    WALK=61,			//慢步
    PACE=62,            //踱步
    CANTER = 63,		//跑步
    GALLOP = 64,        //飞驰
    SMALLSTEPS = 65,    //碎步
    //无专用命令
    NONE = 254,
};
//腿名定义
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
