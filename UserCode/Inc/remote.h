//
// Created by Joerange on 2023/11/18.
//

#ifndef MY_SCUDOG_REMOTE_H
#define MY_SCUDOG_REMOTE_H

#include "stdio.h"
#include "printf.h"
#include "LED.h"

#define REMOTE_REC_LEN 50
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

extern uint8_t REMOTE_RX_BUF[REMOTE_REC_LEN];

extern uint8_t USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符
extern uint16_t USART_RX_STA;         		//接收状态标记
extern uint8_t DebugData;
extern int ReceiveCMD;
extern uint8_t RestartFlag;

extern float Leg1Delay;
extern float Leg2Delay;
extern float Leg3Delay;
extern float Leg4Delay;

extern uint32_t MoveSpeed;
extern uint16_t MoveIntensity_PosKp;
extern float MoveIntensity_SpdKi;
extern uint8_t Jump_Angle_Remote;
extern int16_t RemoteHeartBeat;

void DebugUSART_Init(int bound);//串口1调试无线下载器远程控制用
void RemoteUSART_Init(int bound);//蓝牙无线远程控制用
void RemoteCtrl(uint16_t rx_len);//遥控接收处理
void CloseRemote(void);//关闭遥控串口
void OpenRemote(void);//开启遥控串口
void Remote_Controller(void);

#endif //MY_SCUDOG_REMOTE_H
