/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu.h"
#include "crc16.h"
#include "visual.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LEG_MINCOUNT 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int NewHearbet = 0;
extern float times;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim3;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
    static uint8_t legcount = LEG_MINCOUNT;
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
    if(legcount-- == 0)
    {
        legcount = LEG_MINCOUNT + NewHearbet;
        times++;
    }
//    usart_printf("%f\r\n",times);
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
    uint8_t id;                          //电机id
    uint8_t mode;                        //电机模式
    if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)==SET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);//清除空闲中断接受标志wei
        HAL_UART_DMAStop(&huart1);//关闭DMA接受
//        usart_printf("%d,%d,%d,%d,%d,%d\r\n",LeftLeg_ReceiverBuffer[0],LeftLeg_ReceiverBuffer[1],LeftLeg_ReceiverBuffer[2],LeftLeg_ReceiverBuffer[3],LeftLeg_ReceiverBuffer[4]);
        uart1_rec_crc = CRC16_CCITT(LeftLeg_ReceiverBuffer, 14);
        if(uart1_rec_crc == (LeftLeg_ReceiverBuffer[14] | (LeftLeg_ReceiverBuffer[15] << 8)))           //接收到正确的反馈报文
        {
            id = (LeftLeg_ReceiverBuffer[2] & 0x0F);
            mode = (LeftLeg_ReceiverBuffer[2] >> 4) & 0x07;
            if (mode == 0)
            {
                began_pos[id] =
                        LeftLeg_ReceiverBuffer[7] | (LeftLeg_ReceiverBuffer[8] << 8) | (LeftLeg_ReceiverBuffer[9] << 16) | (LeftLeg_ReceiverBuffer[10] << 24);
                end_pos[id] = began_pos[id];       //把began_pos当成第一次PID计算的反馈zhi

            }
            else if (mode == 1)
            {
//                usart_printf("%d\r\n",end_pos[id]);
                end_pos[id] = LeftLeg_ReceiverBuffer[7] | (LeftLeg_ReceiverBuffer[8] << 8) | (LeftLeg_ReceiverBuffer[9] << 16) | (LeftLeg_ReceiverBuffer[10] << 24);
                real_speed[id] = (int16_t)(LeftLeg_ReceiverBuffer[5] | (LeftLeg_ReceiverBuffer[6] << 8));
                temperature[id] = (int8_t)LeftLeg_ReceiverBuffer[11];
                merror[id] = (LeftLeg_ReceiverBuffer[12] & 0x07);
//                usart_printf("%d\r\n",real_speed[5]);
            }

        }
        //memset(UART6_DMA_REbuffer,0x00,sizeof(UART6_DMA_REbuffer));//清除数组接受到的数据
    }
    HAL_UART_Receive_DMA(&huart1, (uint8_t *)&LeftLeg_ReceiverBuffer, MOTOR_RECEIVE_SIZE);//重新使能DMA接受，继续下1轮recieve

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
    uint32_t tmp_flag = 0;
    uint32_t temp;
    uint16_t rx_len;
    tmp_flag =__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE); //获取IDLE标志位
//    usart_printf("1\r\n");

    __HAL_UART_CLEAR_IDLEFLAG(&huart2);//清除标志位
    HAL_UART_DMAStop(&huart2); //

    if((tmp_flag != RESET) && BlueTeeth_flag == 1)//idle标志被置位
    {
        temp = huart2.Instance->ISR;  //清除状态寄存器SR,读取SR寄存器可以实现清除SR寄存器的功能
        temp = huart2.Instance->RDR; //读取数据寄存器中的数据
        temp = hdma_usart2_rx.Instance->CNDTR;// 获取DMA中未传输的数据个数，NDTR寄存器分析见下面
        rx_len =  REMOTE_REC_LEN - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
        RemoteCtrl(rx_len);	//中断中的数据处理去改变相应控制变量的值
    }//https://www.cnblogs.com/zjx123/p/11949052.html->该知识网址
    HAL_UART_Receive_DMA(&huart2,(uint8_t *)&REMOTE_RX_BUF,REMOTE_REC_LEN);//使能串口5 DMA接受
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt / UART4 wake-up interrupt through EXTI line 34.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
    uint32_t tmp_flag = 0;
    uint32_t temp;
    uint16_t rx_len;
    tmp_flag =__HAL_UART_GET_FLAG(&huart4,UART_FLAG_IDLE); //获取IDLE标志位
    __HAL_UART_CLEAR_IDLEFLAG(&huart4);//清除标志位
    HAL_UART_DMAStop(&huart4); //

    if((tmp_flag != RESET))//idle标志被置位
    {

        temp = huart4.Instance->ISR;  //清除状态寄存器SR,读取SR寄存器可以实现清除SR寄存器的功能
        temp = huart4.Instance->RDR; //读取数据寄存器中的数据

        temp = hdma_uart4_rx.Instance->CNDTR;// 获取DMA中未传输的数据个数，NDTR寄存器分析见下面
        rx_len =  IMU_REC_LEN - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
        //进行数据处理
//        RemoteCtrl(rx_len);	//中断中的数据处理去改变相应控制变量的值
        IMU_Data_Process(1);	//中断中的数据处理去改变相应控制变量的值
    }//https://www.cnblogs.com/zjx123/p/11949052.html->该知识网址
    HAL_UART_Receive_DMA(&huart4,(uint8_t *)&IMU_RX_BUF,IMU_REC_LEN);//使能串口5 DMA接受
  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt / UART5 wake-up interrupt through EXTI line 35.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */
    if(__HAL_UART_GET_FLAG(&huart5,UART_FLAG_IDLE)==SET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);//清除空闲中断接受标志wei
        HAL_UART_DMAStop(&huart1);//关闭DMA接受

        visual_process();

//        memset(VISUAL_REC,0x00,sizeof VISUAL_REC);
    }
    HAL_UART_Receive_DMA(&huart5,(uint8_t *)&VISUAL_REC,Length_of_visual);//使能串口5 DMA接受
    // /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 channel2 global interrupt.
  */
void DMA2_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel2_IRQn 0 */

  /* USER CODE END DMA2_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Channel2_IRQn 1 */

  /* USER CODE END DMA2_Channel2_IRQn 1 */
}

/**
  * @brief This function handles FDCAN2 interrupt 0.
  */
void FDCAN2_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN2_IT0_IRQn 0 */

  /* USER CODE END FDCAN2_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan2);
  /* USER CODE BEGIN FDCAN2_IT0_IRQn 1 */

  /* USER CODE END FDCAN2_IT0_IRQn 1 */
}

/**
  * @brief This function handles FDCAN3 interrupt 0.
  */
void FDCAN3_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN3_IT0_IRQn 0 */

  /* USER CODE END FDCAN3_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan3);
  /* USER CODE BEGIN FDCAN3_IT0_IRQn 1 */

  /* USER CODE END FDCAN3_IT0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel8 global interrupt.
  */
void DMA1_Channel8_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel8_IRQn 0 */

  /* USER CODE END DMA1_Channel8_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel8_IRQn 1 */

  /* USER CODE END DMA1_Channel8_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
