//
// Created by 1 on 2023-11-06.
//
#include "Myinit.h"

int8_t BlueTeeth_flag = 0,NRFStart_flag = 0;

void Myinit(void)
{
    MyFDCan1_config();
    MyFDCan2_config();
    MyFDCan3_config();
    RetargetInit(&huart3);
//    RetargetInit(&huart2);
    HAL_TIM_Base_Start_IT(&htim2);
}

void RemoteControl_Init(int8_t Blue_flag,int8_t NRF_flag)
{
    if(NRF_flag == 1)
    {
        while (NRF24L01_Check()) {
            NRF24L01_RX_Mode();
            usart_printf("NRF READY!\r\n");
        }
    }

    BlueTeeth_flag = Blue_flag;
}
