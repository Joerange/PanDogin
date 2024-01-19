//
// Created by 1 on 2023-11-04.
//
#include "Key.h"

void Key_Scan(void)
{
    if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10) == GPIO_PIN_RESET)
    {
        osDelay(20);
        while(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10) == GPIO_PIN_RESET);
        Key_Task();
        osDelay(20);
    }
}

void Key_Task(void)
{

}