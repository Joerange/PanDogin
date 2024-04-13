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
#include "fdcan.h"
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
osThreadId GO1Init_TaskHandle;
osThreadId VisualHandle;
osThreadId NRFTaskHandle;
osThreadId TripodHeadHandle;
osThreadId GO_OutputLeftHandle;
osThreadId GO_OutputrightHandle;
osMessageQId VisialHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDebug(void const * argument);
void BlueTeeth_RemoteControl(void const * argument);
void GO1Init(void const * argument);
void VisualTask(void const * argument);
void NRF(void const * argument);
void TripodHeadTask(void const * argument);
void GO_OutputLeftTask(void const * argument);
void GO_OutputrightTask(void const * argument);

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

  /* Create the queue(s) */
  /* definition and creation of Visial */
  osMessageQDef(Visial, 1, uint8_t);
  VisialHandle = osMessageCreate(osMessageQ(Visial), NULL);

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

  /* definition and creation of GO1Init_Task */
  osThreadDef(GO1Init_Task, GO1Init, osPriorityLow, 0, 128);
  GO1Init_TaskHandle = osThreadCreate(osThread(GO1Init_Task), NULL);

  /* definition and creation of Visual */
  osThreadDef(Visual, VisualTask, osPriorityHigh, 0, 256);
  VisualHandle = osThreadCreate(osThread(Visual), NULL);

  /* definition and creation of NRFTask */
  osThreadDef(NRFTask, NRF, osPriorityLow, 0, 512);
  NRFTaskHandle = osThreadCreate(osThread(NRFTask), NULL);

  /* definition and creation of TripodHead */
  osThreadDef(TripodHead, TripodHeadTask, osPriorityNormal, 0, 256);
  TripodHeadHandle = osThreadCreate(osThread(TripodHead), NULL);

  /* definition and creation of GO_OutputLeft */
  osThreadDef(GO_OutputLeft, GO_OutputLeftTask, osPriorityHigh, 0, 256);
  GO_OutputLeftHandle = osThreadCreate(osThread(GO_OutputLeft), NULL);

  /* definition and creation of GO_Outputright */
  osThreadDef(GO_Outputright, GO_OutputrightTask, osPriorityLow, 0, 256);
  GO_OutputrightHandle = osThreadCreate(osThread(GO_Outputright), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
    vTaskResume(StartTaskHandle);
    vTaskSuspend(TripodHeadHandle);
    vTaskSuspend(GO1Init_TaskHandle);
    vTaskSuspend(BlueteethTaskHandle);
    vTaskSuspend(GO_OutputLeftHandle);
    vTaskSuspend(GO_OutputrightHandle);
    vTaskSuspend(VisualHandle);
    vTaskSuspend(NRFTaskHandle);
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
    RemoteControl_Init(1,0); //选择要使用的远程控制模式
    Control_Flag(1,0);//选择是否开启陀螺仪与视觉纠偏开关
    IMU_Slove(1);//是否开启障碍时腿时刻保持竖直

    printf("Init_Ready\n");
    osDelay(3);

    osDelay(1000); //在调试的时候延迟3秒用来打开急停开关

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
//      usart_printf("%d,%d\n",gpstate,dpstate);
//      usart_printf("%f,%f,%f,%f,%f,%f\n",IMU_EulerAngle.EulerAngle[Yaw],visual.offset,state_detached_params[1].detached_params_0.step_length,
//                   state_detached_params[1].detached_params_0.freq,state_detached_params[1].detached_params_2.step_length,state_detached_params[1].detached_params_2.freq);
//      usart_printf("%f,%f\n",visual.distance,visual.offset);
//      usart_printf("%f,%f,%f.%f\n", AngleLoop[1].Out_put,AngleLoop[2].Out_put,AngleLoop[3].Out_put,AngleLoop[4].Out_put);
//      usart_printf("%f,%f,%d,%f,%f,%d,%f,%f\n",IMU_EulerAngle.EulerAngle[Yaw],Yaw_PID_Loop.Out_put,Race_count,visual.distance,visual.offset,gpstate,x,y);
//      usart_printf("%f,%f,%f,%f,%f,%f\n",Yaw_PID_Loop.Setpoint,IMU_EulerAngle.EulerAngle[Yaw],Yaw_PID_Loop.Out_put,state_detached_params[1].detached_params_0.step_length,state_detached_params[1].detached_params_2.step_length,visual.offset);
//      usart_printf("%f,%f,%f\n", IMU_EulerAngle.EulerAngle[Yaw],IMU_EulerAngle.EulerAngle[Pitch],IMU_EulerAngle.EulerAngle[Roll]);
    osDelay(1);
  }
  /* USER CODE END BlueTeeth_RemoteControl */
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
    ChangeGainOfPID(6.0f,1.0f,0.03f,0.05f);//初始化pid

    Get_motor_began_pos();       //获得各个电机的初始位
    EndPosture();                //锁住电机

    visual.offset = 100;

    PID_Init(&Yaw_PID_Loop);
    ChangeYawOfPID(0.03f,0.02f,4000.0f,15.0f);//陀螺仪PID初始化

    PID_Init(&Roll_PID_Loop);
    Roll_PID_Loop.P = 0.2f;
    Roll_PID_Loop.D = 0.02f;
    Roll_PID_Loop.SumError = 4000.0f;
    Roll_PID_Loop.Output_limit = 15.0f;

    PID_Init(&VisualLoop);
    VisualLoop.P = 0.2f;
    VisualLoop.D = 0.03f;
    VisualLoop.SumError = 4000.0f;
    VisualLoop.Output_limit = 15.0f;

    printf("GO1 Init Ready\n");
    osDelay(3);

    vTaskResume(GO_OutputLeftHandle);
//    vTaskResume(GO_OutputrightHandle);
    vTaskResume(BlueteethTaskHandle);
//    vTaskResume(NRFTaskHandle);
    vTaskResume(VisualHandle);
    vTaskResume(TripodHeadHandle);
    vTaskSuspend(NULL); //电机初始化任务完成后自挂捏
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GO1Init */
}

/* USER CODE BEGIN Header_VisualTask */
/**
* @brief Function implemaenting the Visual thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VisualTask */
void VisualTask(void const * argument)
{
  /* USER CODE BEGIN VisualTask */

  /* Infinite loop */
  for(;;)
  {
//        visual_process();
      usart_printf("%d,%d,%d,%d,%d,%d\n",visual.data_8[0],visual.data_8[1],visual.data_8[2]
              ,visual.data_8[3],visual.data_8[4],visual.data_8[5]);
//      if(visual.data_8[1] == 1 && gpstate != 0 && gpstate != 3 && gpstate != 1)
//          MarkingTime();

      osDelay(1);
  }
  /* USER CODE END VisualTask */
}

/* USER CODE BEGIN Header_NRF */
/**
* @brief Function implementing the NRFTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NRF */
void NRF(void const * argument)
{
  /* USER CODE BEGIN NRF */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END NRF */
}

/* USER CODE BEGIN Header_TripodHeadTask */
/**
* @brief Function implementing the TripodHead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TripodHeadTask */
void TripodHeadTask(void const * argument)
{
  /* USER CODE BEGIN TripodHeadTask */
    PID_Init(&M2006_Speed);
    PID_Init(&M2006_Position);

    PID_Set_KP_KI_KD(&M2006_Speed,5.0f,0.05f,0.0f);//2006电机速度环初始化
    PID_Set_KP_KI_KD(&M2006_Position,0.4f,0.0f,0.9f);//2006电机位置环初始化

    M2006_Speed.Output_limit = 4000;
    M2006_Position.Output_limit = 10000;
  /* Infinite loop */
  for(;;)
  {
      SetPoint_IMU(&Roll_PID_Loop,0);
      PID_PosLocM2006(&Roll_PID_Loop,IMU_EulerAngle.EulerAngle[Roll]);

      SetPoint_IMU(&M2006_Position, AngleChange(TargetAngle));
      PID_PosLocM2006(&M2006_Position,struct_debug1[0].total_angle);

      SetPoint_IMU(&M2006_Speed, M2006_Position.Out_put);
      PID_PosLocM2006(&M2006_Speed,struct_debug1[0].speed);

      set_current(&hfdcan2,0x200,M2006_Speed.Out_put,0,0,0);

    osDelay(5);
  }
  /* USER CODE END TripodHeadTask */
}

/* USER CODE BEGIN Header_GO_OutputLeftTask */
/**
* @brief Function implementing the GO_OutputLeft thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GO_OutputLeftTask */
void GO_OutputLeftTask(void const * argument)
{
  /* USER CODE BEGIN GO_OutputLeftTask */
  /* Infinite loop */
  for(;;)
  {
      leg_pos_controll();
      leg_pos_controll02();
     osDelay(1);
  }
  /* USER CODE END GO_OutputLeftTask */
}

/* USER CODE BEGIN Header_GO_OutputrightTask */
/**
* @brief Function implementing the GO_Outputright thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GO_OutputrightTask */
void GO_OutputrightTask(void const * argument)
{
  /* USER CODE BEGIN GO_OutputrightTask */
  /* Infinite loop */
  for(;;)
  {
      leg_pos_controll02();
    osDelay(5);
  }
  /* USER CODE END GO_OutputrightTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

