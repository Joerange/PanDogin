//
// Created by Joerange on 2023/11/18.
//

#ifndef MY_SCUDOG_REMOTE_H
#define MY_SCUDOG_REMOTE_H

#include "stdio.h"
#include "printf.h"
#include "LED.h"

#define REMOTE_REC_LEN 50
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

extern uint8_t REMOTE_RX_BUF[REMOTE_REC_LEN];

extern uint8_t USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern uint16_t USART_RX_STA;         		//����״̬���
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

void DebugUSART_Init(int bound);//����1��������������Զ�̿�����
void RemoteUSART_Init(int bound);//��������Զ�̿�����
void RemoteCtrl(uint16_t rx_len);//ң�ؽ��մ���
void CloseRemote(void);//�ر�ң�ش���
void OpenRemote(void);//����ң�ش���
void Remote_Controller(void);

#endif //MY_SCUDOG_REMOTE_H
