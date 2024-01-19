//
// Created by 1 on 2023-11-06.
//

#ifndef MY_SCUDOG_MYINIT_H
#define MY_SCUDOG_MYINIT_H

#include "CanMotor.h"
#include "stm32g4xx_hal_tim.h"
#include "retarget.h"
#include "usart.h"
#include "motor.h"
#include "24l01.h"

extern int8_t BlueTeeth_flag,NRFStart_flag;
extern TIM_HandleTypeDef htim2;

void Myinit(void);
void RemoteControl_Init(int8_t Blue_flag,int8_t NRF_flag);

#endif //MY_SCUDOG_MYINIT_H
