//
// Created by 1 on 2023-11-06.
//
#include "Myinit.h"
#include "string.h"

int8_t BlueTeeth_flag = 0,NRFStart_flag = 0;

void Myinit(void)
{
    MyFDCan1_config();
    MyFDCan2_config();
    MyFDCan3_config();
    RetargetInit(&huart3);
    HAL_TIM_Base_Start_IT(&htim2);

    memcpy(StateDetachedParams_Copy,state_detached_params,100*StatesMaxNum);//state_detached_params每个元素（DetachedParam型,即每种步态，外加一个u8的ID号）。
    //实际占据的字节数为4*6*4+4=96+4=100（＋4而不是加1是因为要4字节对齐）。
    //设定StatesMaxNum，则拷贝的上限为100*StateMaxNum，不要少拷贝，可以多拷贝，但多拷贝的不要用。
    //该复制操作不要在任务中进行，而要在操作系统初始化之前进行，否则将给操作系统的运行造成奇怪的问题。
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
