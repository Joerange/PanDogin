//
// Created by Joerange on 2023/11/18.
//
#include "Attitude_Slove.h"
#include "Attitude_Task.h"
#include "remote.h"
#include "imu.h"
#include "stm32g474xx.h"

//重定义fputc函数
//int fputc(int ch, FILE *f)
//{
//    while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
//    USART1->DR = (uint8_t) ch;
//    return ch;
//}

//串口1中断服务程序
uint8_t USART_RX_BUF[USART_REC_LEN];//接收缓冲,最大USART_REC_LEN个字节
//接收状态
//bit15，	接收完成标志（0x0a）
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
uint16_t USART_RX_STA=0;//接收状态标记

/*遥控器调参相关控制变量*/
uint8_t REMOTE_RX_BUF[REMOTE_REC_LEN];
uint8_t TestFlag = 0;
uint8_t RestartFlag = 0;
uint8_t DebugData = 0;

//遥控快速调节步态参数以实现最佳步态的确定
float Leg1Delay = 0;
float Leg2Delay = 0.5;
float Leg3Delay = 0.5;
float Leg4Delay = 0;

//运动速度控制
uint32_t MoveSpeed = SpeedMode_EXTREME;
//运动强度控制
uint16_t MoveIntensity_PosKp = 70;
float MoveIntensity_SpdKi = 0.26;
//跳跃角度控制
uint8_t Jump_Angle_Remote = 60;

//关闭遥控串口接收中断
void CloseRemote(void)
{
    // 关闭串口5中断
    HAL_NVIC_DisableIRQ(UART5_IRQn);
}
//开启遥控串口接收中断
void OpenRemote(void)
{
    HAL_NVIC_EnableIRQ(UART5_IRQn);
}

void Remote_Controller(void)
{
    switch (gpstate) {
        case 1:
            StandUp_Posture();
            break;
        case 3:
            LieDown_Posture();
            break;
        case 6:
            Turn('r');
            break;
        case 7:
            Turn('l');
            break;
        case 10:
            Trot(Backward,1);
            break;
        case 11:
            Trot(Forward,1);
            break;
        case 33:
            MarkingTime();
            break;
        default:
            break;
    }
}

//控制函数
void RemoteCtrl(uint16_t rx_len)
{
    if(TestFlag!=0 && rx_len ==1) //DMA特殊情况下非一次性接收时所采取的程序方式
    {
        //有IMU控制和没有大不相同，因为实际上有IMU控制时，步态的参数基准本质上由copy决定，故无法通过调节原参数改变实际步态。
        if(IMU_Control_Flag==0)
        {
            switch(TestFlag)
            {
                //步态参数调整
                case 1:
                    state_detached_params[9].detached_params_0.stance_height =
                    state_detached_params[9].detached_params_1.stance_height =
                    state_detached_params[9].detached_params_2.stance_height =
                    state_detached_params[9].detached_params_3.stance_height = REMOTE_RX_BUF[0] / 10.0f;
                    break;
                case 2:
                    state_detached_params[9].detached_params_0.step_length =
                    state_detached_params[9].detached_params_1.step_length =
                    state_detached_params[9].detached_params_2.step_length =
                    state_detached_params[9].detached_params_3.step_length = REMOTE_RX_BUF[0] / 5.6f;
                    break;
                case 3:
                    state_detached_params[9].detached_params_0.up_amp =
                    state_detached_params[9].detached_params_1.up_amp =
                    state_detached_params[9].detached_params_2.up_amp =
                    state_detached_params[9].detached_params_3.up_amp = REMOTE_RX_BUF[0] / 10.0f;
                    break;
                case 4:
                    state_detached_params[9].detached_params_0.down_amp =
                    state_detached_params[9].detached_params_1.down_amp =
                    state_detached_params[9].detached_params_2.down_amp =
                    state_detached_params[9].detached_params_3.down_amp = REMOTE_RX_BUF[0] / 10.0f;
                    break;
                case 5:
                    state_detached_params[9].detached_params_0.flight_percent =
                    state_detached_params[9].detached_params_1.flight_percent =
                    state_detached_params[9].detached_params_2.flight_percent =
                    state_detached_params[9].detached_params_3.flight_percent = REMOTE_RX_BUF[0] / 100.0f;
                    break;
                case 6:
                    state_detached_params[9].detached_params_0.freq =
                    state_detached_params[9].detached_params_1.freq =
                    state_detached_params[9].detached_params_2.freq =
                    state_detached_params[9].detached_params_3.freq = REMOTE_RX_BUF[0] / 10.0f;
                    break;
                    //相位延时调整
                case 7:
                    Leg1Delay = (float)REMOTE_RX_BUF[0] / 100;
                    break;
                case 8:
                    Leg2Delay = (float)REMOTE_RX_BUF[0] / 100;
                    break;
                case 9:
                    Leg3Delay = (float)REMOTE_RX_BUF[0] / 100;
                    break;
                case 10:
                    Leg4Delay = (float)REMOTE_RX_BUF[0] / 100;
                    break;
                    //步速调整
                case 11:
                    MoveSpeed = (int)(REMOTE_RX_BUF[0] * 34.9);
                    break;
                    //IMU强度调整
                case 12:;
                    break;
                    //新的心跳
                case 13:
                    //NewHeartbeat = REMOTE_RX_BUF[0];//在定时器当中产生的心跳
                    break;
                    //跳跃角度调整
                case 14:
                    Jump_Angle_Remote = REMOTE_RX_BUF[0];
                    break;
                    //步态强度调整
                case 15:
                    MoveIntensity_PosKp = REMOTE_RX_BUF[0];
                    break;
                default:
                    break;
            }
        }
        else
        {
            switch(TestFlag)
            {
                //步态参数调整
                case 1:
                    StateDetachedParams_Copy[9].detached_params_0.stance_height=
                    StateDetachedParams_Copy[9].detached_params_1.stance_height=
                    StateDetachedParams_Copy[9].detached_params_2.stance_height=
                    StateDetachedParams_Copy[9].detached_params_3.stance_height=REMOTE_RX_BUF[0]/10.0f;
                    break;
                case 2:
                    StateDetachedParams_Copy[9].detached_params_0.step_length=
                    StateDetachedParams_Copy[9].detached_params_1.step_length=
                    StateDetachedParams_Copy[9].detached_params_2.step_length=
                    StateDetachedParams_Copy[9].detached_params_3.step_length= REMOTE_RX_BUF[0]/5.6f;
                    break;
                case 3:
                    StateDetachedParams_Copy[9].detached_params_0.up_amp=
                    StateDetachedParams_Copy[9].detached_params_1.up_amp=
                    StateDetachedParams_Copy[9].detached_params_2.up_amp=
                    StateDetachedParams_Copy[9].detached_params_3.up_amp=REMOTE_RX_BUF[0]/10.0f;
                    break;
                case 4:
                    StateDetachedParams_Copy[9].detached_params_0.down_amp=
                    StateDetachedParams_Copy[9].detached_params_1.down_amp=
                    StateDetachedParams_Copy[9].detached_params_2.down_amp=
                    StateDetachedParams_Copy[9].detached_params_3.down_amp=REMOTE_RX_BUF[0]/10.0f;
                    break;
                case 5:
                    StateDetachedParams_Copy[9].detached_params_0.flight_percent=
                    StateDetachedParams_Copy[9].detached_params_1.flight_percent=
                    StateDetachedParams_Copy[9].detached_params_2.flight_percent=
                    StateDetachedParams_Copy[9].detached_params_3.flight_percent=REMOTE_RX_BUF[0]/100.0f;
                    break;
                case 6:
                    StateDetachedParams_Copy[9].detached_params_0.freq=
                    StateDetachedParams_Copy[9].detached_params_1.freq=
                    StateDetachedParams_Copy[9].detached_params_2.freq=
                    StateDetachedParams_Copy[9].detached_params_3.freq=REMOTE_RX_BUF[0]/10.0f;
                    break;
                    //相位延时调整
                case 7:
                    Leg1Delay = (float)REMOTE_RX_BUF[0]/100;
                    break;
                case 8:
                    Leg2Delay = (float)REMOTE_RX_BUF[0]/100;
                    break;
                case 9:
                    Leg3Delay = (float)REMOTE_RX_BUF[0]/100;
                    break;
                case 10:
                    Leg4Delay = (float)REMOTE_RX_BUF[0]/100;
                    break;
                    //步速调整
                case 11:
                    MoveSpeed = (int)(REMOTE_RX_BUF[0]*34.9);
                    break;
                    //IMU强度调整
                case 12:;
                    break;
                    //新的心跳
                case 13:
                    //NewHeartbeat = REMOTE_RX_BUF[0];
                    break;
                    //跳跃角度调整
                case 14:
                    Jump_Angle_Remote = REMOTE_RX_BUF[0];
                    break;
                    //步态强度调整
                case 15:
                    MoveIntensity_PosKp = REMOTE_RX_BUF[0];
                    break;
                case 16:
                    MoveIntensity_SpdKi = (float)REMOTE_RX_BUF[0]/50;
                    break;
                default:
                    break;
            }
        }
        TestFlag = 0;
    }
    else if(rx_len ==3 && REMOTE_RX_BUF[rx_len-1]=='C' && REMOTE_RX_BUF[rx_len-2]=='R')//三位控制指令
    {
        //遥控器协议与命令表对应关系转换
        switch(REMOTE_RX_BUF[0])
        {
            /*大写字母命令（通用命令）*/
            case 'W':gpstate = MARCH;break;
            case 'S':gpstate = MARCH_BACK;break;
            case 'A':gpstate = ROTAT_LEFT;break;
            case 'D':gpstate = ROTAT_RIGHT;break;
            case 'Q':gpstate = SQUAT;break;
            case 'Z':gpstate = END;break;
            case 'X':gpstate = HALT;break;
            case 'E':gpstate = KNEEL;break;
            case 'C':gpstate = SHAKEHAND;break;
            case 'R':gpstate = STRETCH;break;
            case 'V':gpstate = PLANE;break;
            case 'T':gpstate = Jump_Leap;break;
            case 'G':gpstate = Jump_Standard;break;
            case 'Y':gpstate = Jump_High;break;
            case 'F':gpstate = Jump_Far;break;
            case 'H':gpstate = WARN;break;
            case 'N':gpstate = SWAY;break;
            case 'U':gpstate = MARKINGTIME;break;
            case 'J':gpstate = HOMOLATERAL;break;
            case 'B':gpstate = NIGHTVISION;break;
            case 'I':yawwant=IMU_EulerAngle.EulerAngle[Yaw];break;//修正IMU目标值为当前值
            case 'O':gpstate = REPOSITION;break;//自动复位回中
                /*特殊字符命令（专用命令）*/
            case '>':dpstate = RACING;break;	        //竞速
            case '~':dpstate = HIGHHURDLES;break;      //高栏
            case '#':dpstate = PEBBLEROAD;break;       //卵石路
            case '^':dpstate = SEESAW;break;	        //跷跷板
            case '|':dpstate = CALANDRIA;break;	    //排管
            case '=':dpstate = BRIDGE;break;	        //双木桥
            case '-':dpstate = STAIR;break;		    //阶梯
            case '*':dpstate = TEST;break;             //调试
            case '?':dpstate = NONE;break;             //退出专用命令
                /*小写字母及数字命令（特殊命令，如调参、自动重启，步态切换等）*/
                //系统参数修正命令
            case 'l':yawwant+=2;break;//目标角度增加意味想要向左进行调整
            case 'r':yawwant-=2;break;//目标角度减小意味想要向右
            case 'o':yawwant=0;break; //无偏，目标角度回到0
            case 'p':RestartFlag=1;break;//通过控制一个变量由0到1，从而使系统卡死在while循环中，在循环中执行“卧倒态”，从而无法喂狗，同时双闪停止，在1s后触发系统重启。
            case 'i':IMU_Control_Flag=!IMU_Control_Flag;break;//IMU姿态调节的控制开关命令
            case 'w':break;
                /******调试界面使用******/
                //步态参数调整命令兼具步态切换命令
            case 'h':if(dpstate==TEST)TestFlag=1;else dpstate=TROT;break;
            case 's':if(dpstate==TEST)TestFlag=2;else dpstate=WALK;break;
            case 'a':if(dpstate==TEST)TestFlag=3;else dpstate=PACE;break;
            case 'd':if(dpstate==TEST)TestFlag=4;else dpstate=CANTER;break;
            case 'b':if(dpstate==TEST)TestFlag=5;else dpstate=GALLOP;break;
            case 'f':if(dpstate==TEST)TestFlag=6;else dpstate=SMALLSTEPS;break;
            case 'm':TestFlag=11;break;
            case 'k':TestFlag=12;break;
            case 'u':TestFlag=15;break;
            case 'n':TestFlag=16;break;
                //步态相位延时调整
            case '1':TestFlag=7;break;
            case '2':TestFlag=8;break;
            case '3':TestFlag=9;break;
            case '4':TestFlag=10;break;
                //新的心跳
            case 'x':TestFlag=13;break;
                //跳跃角度控制
            case 'g':if(dpstate==TEST) TestFlag=14;else dpstate=CHECK;break;
        }
    }

}




