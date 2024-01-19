/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId StartTaskHandle;
osThreadId BlueteethTaskHandle;
osThreadId NRFTaskHandle;
osThreadId Posture_TaskHandle;
osThreadId LegControl_LeftHandle;
osThreadId LegControl_righHandle;
osThreadId GO1Init_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDebug(void const * argument);
void BlueTeeth_RemoteControl(void const * argument);
void NRF_RemoteControl(void const * argument);
void Posture(void const * argument);
void legControl_Left(void const * argument);
void legcontrol_right(void const * argument);
void GO1Init(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of StartTask */
  osThreadDef(StartTask, StartDebug, osPriorityLow, 0, 128);
  StartTaskHandle = osThreadCreate(osThread(StartTask), NULL);

  /* definition and creation of BlueteethTask */
  osThreadDef(BlueteethTask, BlueTeeth_RemoteControl, osPriorityAboveNormal, 0, 256);
  BlueteethTaskHandle = osThreadCreate(osThread(BlueteethTask), NULL);

  /* definition and creation of NRFTask */
  osThreadDef(NRFTask, NRF_RemoteControl, osPriorityAboveNormal, 0, 256);
  NRFTaskHandle = osThreadCreate(osThread(NRFTask), NULL);

  /* definition and creation of Posture_Task */
  osThreadDef(Posture_Task, Posture, osPriorityNormal, 0, 512);
  Posture_TaskHandle = osThreadCreate(osThread(Posture_Task), NULL);

  /* definition and creation of LegControl_Left */
  osThreadDef(LegControl_Left, legControl_Left, osPriorityNormal, 0, 512);
  LegControl_LeftHandle = osThreadCreate(osThread(LegControl_Left), NULL);

  /* definition and creation of LegControl_righ */
  osThreadDef(LegControl_righ, legcontrol_right, osPriorityNormal, 0, 512);
  LegControl_righHandle = osThreadCreate(osThread(LegControl_righ), NULL);

  /* definition and creation of GO1Init_Task */
  osThreadDef(GO1Init_Task, GO1Init, osPriorityLow, 0, 128);
  GO1Init_TaskHandle = osThreadCreate(osThread(GO1Init_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
//    vTaskResume(StartTaskHandle);
    vTaskSuspend(GO1Init_TaskHandle);
    vTaskSuspend(BlueteethTaskHandle);
    vTaskSuspend(NRFTaskHandle);
    vTaskSuspend(Posture_TaskHandle);
    vTaskSuspend(LegControl_LeftHandle);
    vTaskSuspend(LegControl_righHandle);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDebug */
/**
  * @brief  Function implementing the StartTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDebug */
void StartDebug(void const * argument)
{
  /* USER CODE BEGIN StartDebug */
    Myinit();
    RemoteControl_Init(1,0);
    printf("Init_Ready\n");
    osDelay(3);

    osDelay(3000);

    vTaskResume(GO1Init_TaskHandle);

  /* Infinite loop */
  for(;;)
  {
      LED1_Flash;
      LED2_Flash;

      osDelay(500);
  }

  /* USER CODE END StartDebug */
}

/* USER CODE BEGIN Header_BlueTeeth_RemoteControl */
/**
* @brief Function implementing the BlueteethTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BlueTeeth_RemoteControl */
void BlueTeeth_RemoteControl(void const * argument)
{
  /* USER CODE BEGIN BlueTeeth_RemoteControl */
  /* Infinite loop */
  for(;;)
  {
      Remote_Controller();

    osDelay(5);
  }
  /* USER CODE END BlueTeeth_RemoteControl */
}

/* USER CODE BEGIN Header_NRF_RemoteControl */
/**
* @brief Function implementing the NRFTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NRF_RemoteControl */
void NRF_RemoteControl(void const * argument)
{
  /* USER CODE BEGIN NRF_RemoteControl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END NRF_RemoteControl */
}

/* USER CODE BEGIN Header_Posture */
/**
* @brief Function implementing the Posture_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Posture */
void Posture(void const * argument)
{
  /* USER CODE BEGIN Posture */
  /* Infinite loop */
  for(;;)
  {
    osDelay(5);
  }
  /* USER CODE END Posture */
}

/* USER CODE BEGIN Header_legControl_Left */
/**
* @brief Function implementing the LegControl_Left thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_legControl_Left */
void legControl_Left(void const * argument)
{
  /* USER CODE BEGIN legControl_Left */

  /* Infinite loop */
  for(;;)
  {
      leg_pos_controll02();
      leg_pos_controll();
    osDelay(2);
  }
  /* USER CODE END legControl_Left */
}

/* USER CODE BEGIN Header_legcontrol_right */
/**
* @brief Function implementing the LegControl_righ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_legcontrol_right */
void legcontrol_right(void const * argument)
{
  /* USER CODE BEGIN legcontrol_right */
  /* Infinite loop */
  for(;;)
  {
      Speed_Controll();

    osDelay(2);
  }
  /* USER CODE END legcontrol_right */
}

/* USER CODE BEGIN Header_GO1Init */
/**
* @brief Function implementing the GO1Init_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GO1Init */
void GO1Init(void const * argument)
{
  /* USER CODE BEGIN GO1Init */
    MOTOR_Send_Init(); //初始化电机发送帧头
    Eight_PID_Init();//八个电机PID结构体初始化
    ChangeGainOfPID(3.0f,0,0.03f,0.05f);//初始化pid

    Get_motor_began_pos();       //获得各个电机的初始位
    EndPosture();                //锁住电机

    PID_Init(&Yaw_PID_Loop);
    Yaw_PID_Loop.P = 2.0f;
    Yaw_PID_Loop.D = 0.2f;
    Yaw_PID_Loop.SumError_limit = 2500;
    Yaw_PID_Loop.Output_limit = 15;//陀螺仪结构体初始化

    printf("GO1 Init Ready\n");
    osDelay(3);

    vTaskResume(LegControl_LeftHandle);
//    vTaskResume(LegControl_righHandle);
    vTaskResume(BlueteethTaskHandle);
    vTaskSuspend(NULL);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GO1Init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

