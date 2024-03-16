//
// Created by 19734 on 2022/12/26.
//
#include "motor.h"
#include "cmsis_os.h"
#include "crc16.h"
#include "usart.h"
#include "tim.h"
#include "pid.h"
#include "Attitude_Task.h"
#include "Attitude_Slove.h"

ControlData_t motor_s;
ControlData_t Motor_s[MOTOR_ID_NUM];
int8_t MOTOR_Send[MOTOR_ID_NUM][MOTOR_SEND_LENGTH];
int8_t motor_send[MOTOR_SEND_LENGTH] = {0};  //发送数组
int32_t began_pos[MOTOR_ID_NUM];                   //电机运动初始位置
int32_t end_pos[MOTOR_ID_NUM];                     //电机运动结束位置
int16_t real_speed[MOTOR_ID_NUM];                  //电机实时速度
int8_t temperature[MOTOR_ID_NUM];                  //电机温度
uint8_t merror[MOTOR_ID_NUM];
uint16_t rec_crc;                    //反馈报文的CRC校验码
uint16_t uart1_rec_crc;
float K_P=0.0f,K_W=0.0f;
float speed_kp = 0.1f;

uint8_t LeftLeg_ReceiverBuffer[MOTOR_RECEIVE_SIZE] = {0};
uint8_t RightLeg_ReceiverBuffer[MOTOR_RECEIVE_SIZE] = {0};
/**
 * 功能：通过串口给电机发送命令报文
 * @param id 电机id
 */
void UART_SendMessage(uint8_t id)
{
//    if(id<5)
//    {
        //1号腿和2号腿挂在串口1的总线上
        HAL_UART_Transmit_DMA(&huart1,(uint8_t *) MOTOR_Send[id],MOTOR_SEND_LENGTH);
//    }
//    else
//    {
//        //3号腿和4号腿挂在串口6的总线上
//        HAL_UART_Transmit_DMA(&huart2,(uint8_t *) MOTOR_Send[id],MOTOR_SEND_LENGTH);
//    }
}


void modify_data(void)
{
    motor_send[0] = motor_s.head[0];
    motor_send[1] = motor_s.head[1];
    motor_send[2] = ((motor_s.mode.none<<7)|(motor_s.mode.status<<4)|(motor_s.mode.id));
    motor_send[3] = (motor_s.comd.tor_des*256);         //低8位
    motor_send[4] = (int)(motor_s.comd.tor_des*256)>>8;      //高8位
    motor_send[5] = (motor_s.comd.spd_des*256/(2*pi));
    motor_send[6] = (int)(motor_s.comd.spd_des*256/(2*pi))>>8;
    motor_send[7] = (int)(motor_s.comd.pos_des*32768/(2*pi));
    motor_send[8] = (int)(motor_s.comd.pos_des*32768/(2*pi))>>8;
    motor_send[9] = (int)(motor_s.comd.pos_des*32768/(2*pi))>>16;
    motor_send[10] = (int)(motor_s.comd.pos_des*32768/(2*pi))>>24;
    motor_send[11] = (int)(motor_s.comd.k_p*1280);
    motor_send[12] = (int)(motor_s.comd.k_p*1280)>>8;
    motor_send[13] = (int)(motor_s.comd.k_w*1280);
    motor_send[14] = (int)(motor_s.comd.k_w*1280)>>8;
    motor_s.CRC16 = CRC16_CCITT(motor_send,15);
    motor_send[15] = motor_s.CRC16;
    motor_send[16] = motor_s.CRC16>>8;


}
/**
 * 功能：给电机发送报文各个字节赋值
 * @param id
 */
void modify_id_data(uint8_t id)
{
    MOTOR_Send[id][2] = ((Motor_s[id].mode.none<<7)|(Motor_s[id].mode.status<<4)|(Motor_s[id].mode.id));
    MOTOR_Send[id][3] = (Motor_s[id].comd.tor_des*256);         //低8位
    MOTOR_Send[id][4] = (int)(Motor_s[id].comd.tor_des*256)>>8;         //高8位
    MOTOR_Send[id][5] = (Motor_s[id].comd.spd_des*256/(2*pi));
    MOTOR_Send[id][6] = (int)(Motor_s[id].comd.spd_des*256/(2*pi))>>8;
    MOTOR_Send[id][7] = (int)(Motor_s[id].comd.pos_des*32768/(2*pi));
    MOTOR_Send[id][8] = (int)(Motor_s[id].comd.pos_des*32768/(2*pi))>>8;
    MOTOR_Send[id][9] = (int)(Motor_s[id].comd.pos_des*32768/(2*pi))>>16;
    MOTOR_Send[id][10] = (int)(Motor_s[id].comd.pos_des*32768/(2*pi))>>24;
    MOTOR_Send[id][11] = (int)(Motor_s[id].comd.k_p*1280);
    MOTOR_Send[id][12] = (int)(Motor_s[id].comd.k_p*1280)>>8;
    MOTOR_Send[id][13] = (int)(Motor_s[id].comd.k_w*1280);
    MOTOR_Send[id][14] = (int)(Motor_s[id].comd.k_w*1280)>>8;
    Motor_s[id].CRC16 = CRC16_CCITT(MOTOR_Send[id],15);
    MOTOR_Send[id][15] = Motor_s[id].CRC16;
    MOTOR_Send[id][16] = Motor_s[id].CRC16>>8;
}

void motor_stop()          //给电机发送停止命令
{
    motor_send[0] = 0xFE;
    motor_send[1] = 0xEE;
    motor_send[2] = 0x00;
    motor_s.CRC16 = CRC16_CCITT(motor_send,15);
    motor_send[15] = motor_s.CRC16;
    motor_send[16] = motor_s.CRC16>>8;

    HAL_UART_Transmit_DMA(&huart2,(uint8_t *) motor_send,MOTOR_SEND_LENGTH);
}


void motor_test(void)
{
    int i;
    motor_s.head[0] = 0xFE;
    motor_s.head[1] = 0xEE;
    motor_s.mode.id = 0;
    motor_s.mode.status = 1;      //FOC闭环模式
    motor_s.comd.tor_des = 0;              //T
    motor_s.comd.spd_des = 0;      //W
    //设置目标位置时一定要加上上电后的初始位置（began_pos）,比如初始位置为0.3pi,而你设置的目标位置是pi,你要是不加上初始位置，
    //那么电机只会从0.3pi->pi，实际转过0.7pi;所以要将目标位置设置为pi+0.3pi=1.3pi，让电机从0.3pi->1.3pi，实际转过pi
    motor_s.comd.pos_des = pi*6.33+(began_pos[0]*2*pi)/32768;
    motor_s.comd.k_p = 0.25f;
    motor_s.comd.k_w = 0.015f;



    for(i=0;i<100;i++)
    {
        modify_data();
        HAL_UART_Transmit_DMA(&huart2,(uint8_t *) motor_send,MOTOR_SEND_LENGTH);
        osDelay(1000);

    }
}

/**功能：设置指定电机的模式
 *
 * @param id
 * @param mode 0:锁定
 *             1：FOC闭环
 *             2：编码器校准（等待5s，期间不可以给电机发送任何数据包）
 */
void Set_Motor_Mode(uint8_t id,uint8_t mode)
{
    Motor_s[id].mode.status = mode;
}

/**功能：初始化所有电机的发送报文的头帧为0xFE,0xEE
 * 参数：无
 * 返回值：无
 */
void MOTOR_Send_Init(void)
{
    for (int i = 0; i < 12; i++)
    {
        MOTOR_Send[i][0] = 0xFE;
        MOTOR_Send[i][1] = 0xEE;
    }
}


/**功能：控制指定id的电机转动指定的角度
 *
 * 参数：id:电机的id
 *      rad:期望电机转动的角度，注意是弧度制。逆时针为正，顺时针为负
 *      motor_mode:可选闭环模式中的位置模式（Position）和零力矩模式（Zero_torque）
 *
 * 返回值：无
 */
void motor_pos_controll(uint8_t id,float rad,uint8_t motor_mode)
{
    Set_Motor_Mode(id,1);      //FOC闭环模式
    Motor_s[id].mode.id = id;

    switch (motor_mode)
    {
        //位置模式
        case position:
        Motor_s[id].comd.tor_des = 0.0f;     //T
        Motor_s[id].comd.spd_des = 0.0f;     //W
        Motor_s[id].comd.pos_des = rad * 6.33f + (began_pos[id] * 2 * pi) / 32768;
        Motor_s[id].comd.k_p = K_P;       //0.25
        Motor_s[id].comd.k_w = K_W;       //0.015
        break;

        //零力矩模式
        case Zero_torque:
        Motor_s[id].comd.tor_des = 0.0f;     //T
        Motor_s[id].comd.spd_des = 0.0f;     //W
        Motor_s[id].comd.pos_des = 0.0f;
        Motor_s[id].comd.k_p = 0.0f;
        Motor_s[id].comd.k_w = 0.0f;
        break;

        default:
            break;
    }


    modify_id_data(id);
//    HAL_UART_Transmit_DMA(&huart6,(uint8_t *) MOTOR_Send[id],MOTOR_SEND_LENGTH);
    UART_SendMessage(id);
}


/*!
 * 对电机的速度进行控制
 * @param id
 * @param w     电机输出轴目标角速度
 * @param kw
 */
void motor_speed_controll_with_kw(uint8_t id,float w,float kw)
{
    Set_Motor_Mode(id,1);      //FOC闭环模式
    Motor_s[id].mode.id = id;

    Motor_s[id].comd.tor_des = 0.0f;     //T
    Motor_s[id].comd.spd_des = w*6.33f;     //W
    Motor_s[id].comd.pos_des = 0.0f;     //Pos
    Motor_s[id].comd.k_p = 0;
    Motor_s[id].comd.k_w = kw;

    modify_id_data(id);
    //向电机发送命令
    UART_SendMessage(id);
}

void motor_torque_controll(uint8_t id, float torque)
{
    Set_Motor_Mode(id,1);      //FOC闭环模式
    Motor_s[id].mode.id = id;

    Motor_s[id].comd.tor_des = torque/6.33f;     //T
    Motor_s[id].comd.spd_des = 0.0f;     //W
    Motor_s[id].comd.pos_des = 0.0f;     //Pos
    Motor_s[id].comd.k_p = 0;
    Motor_s[id].comd.k_w = 0;

    modify_id_data(id);
    //向电机发送命令
    UART_SendMessage(id);
}

/*！
 * 编码器校准，没有特殊情况请不要校准编码器，出厂前电机已校准过
 * 使用完此函数后一定要延时5s,在这5s期间不可与电机通信，否则会标定失败
 * 校准的例子：
 *       Encoder_calibration(0);       //对0号电机进行编码器校正
 *       HAL_Delay(5000);   //为了保证标定效果，切换到编码器校准模式后，需要等待 5s 再进行通信
 */
void Encoder_calibration(uint8_t id)
{
    Set_Motor_Mode(id,2);      //编码器校准模式
    Motor_s[id].mode.id = id;


    modify_id_data(id);
    HAL_UART_Transmit_DMA(&huart2,(uint8_t *) MOTOR_Send[id],MOTOR_SEND_LENGTH);
}

/*!
 * 与motor_pos_controll()的区别是提供了修改kp and kw的入口，并且目标位置由弧度制变成了控制报文的原始数据（0~2PI对应0~32768）——
 * @param id          motor id
 * @param theat_set   0~32768 corresponding 0~2PI
 * @param motor_mode  Position or Zero_torque
 * @param KP          kp
 * @param KW          kw
 */
void Pos_Controll(uint8_t id,int32_t theat_set,uint8_t motor_mode,float KP,float KW)
{
    Set_Motor_Mode(id,1);      //FOC闭环模式
    Motor_s[id].mode.id = id;

    switch (motor_mode)
    {
        //位置模式
        case position:
            Motor_s[id].comd.tor_des = 0.0f;     //T
            Motor_s[id].comd.spd_des = 0.0f;     //W
            Motor_s[id].comd.pos_des = (theat_set*6.33f+began_pos[id] )* 2 * pi/ 32768;
            Motor_s[id].comd.k_p = KP;
            Motor_s[id].comd.k_w = KW;
            break;

            //零力矩模式
        case Zero_torque:
            Motor_s[id].comd.tor_des = 0.0f;     //T
            Motor_s[id].comd.spd_des = 0.0f;     //W
            Motor_s[id].comd.pos_des = 0.0f;
            Motor_s[id].comd.k_p = 0.0f;
            Motor_s[id].comd.k_w = 0.0f;
            break;

        default:
            break;
    }

    modify_id_data(id);
    HAL_UART_Transmit_DMA(&huart2,(uint8_t *) MOTOR_Send[id],MOTOR_SEND_LENGTH);
}

/**功能：在上电时给指定的电机发送锁定命令来获得电机的初始位置
 *
 * @param id
 */
void Motor_stop(uint8_t id)
{
    Motor_s[id].mode.id = id;
    Set_Motor_Mode(id,0);     //锁定模式
    MOTOR_Send[id][2] = ((Motor_s[id].mode.none<<7)|(Motor_s[id].mode.status<<4)|(Motor_s[id].mode.id));
    Motor_s[id].CRC16 = CRC16_CCITT(MOTOR_Send[id],15);
    MOTOR_Send[id][15] = Motor_s[id].CRC16;
    MOTOR_Send[id][16] = Motor_s[id].CRC16>>8;

//    HAL_UART_Transmit_DMA(&huart6,(uint8_t *) MOTOR_Send[id],MOTOR_SEND_LENGTH);
    UART_SendMessage(id);

}

/*
 * 功能：获得各个电机的上电初始位置
 */
void Get_motor_began_pos(void)
{
    Motor_stop(0);           //获得0号电机的初始位置
    HAL_Delay(10);       //1s延时用于初始化，不发起任务调度
    Motor_stop(1);           //获得1号电机的初始位置
    HAL_Delay(10);       //1s延时用于初始化，不发起任务调度
    Motor_stop(2);           //获得2号电机的初始位置
    HAL_Delay(10);       //1s延时用于初始化，不发起任务调度
    Motor_stop(3);
    HAL_Delay(10);
    Motor_stop(4);
    HAL_Delay(10);
    Motor_stop(5);
    HAL_Delay(10);
    Motor_stop(6);
    HAL_Delay(10);
    Motor_stop(7);
    HAL_Delay(10);
    Motor_stop(8);
    HAL_Delay(10);
    Motor_stop(9);
    HAL_Delay(10);
    Motor_stop(10);
    HAL_Delay(10);
    Motor_stop(11);
    HAL_Delay(10);
}

void Get_motor_began_pos1(void)
{
    Motor_stop(0);           //获得0号电机的初始位置
//    user_delaynus_tim(100);
    HAL_Delay(10);       //1s延时用于初始化，不发起任务调度
    Motor_stop(1);           //获得1号电机的初始位置
//    user_delaynus_tim(100);
    HAL_Delay(10);       //1s延时用于初始化，不发起任务调度
    Motor_stop(2);           //获得2号电机的初始位置
//    user_delaynus_tim(100);
    HAL_Delay(10);       //1s延时用于初始化，不发起任务调度
}

void Get_motor_began_pos2(void)
{
    Motor_stop(3);
    HAL_Delay(10);
    Motor_stop(4);
    HAL_Delay(10);
    Motor_stop(5);
    HAL_Delay(10);
}
void Get_motor_began_pos3(void)
{
    Motor_stop(6);
    HAL_Delay(10);
    Motor_stop(7);
    HAL_Delay(10);
    Motor_stop(8);
    HAL_Delay(10);
}
void Get_motor_began_pos4(void)
{
    Motor_stop(9);
    HAL_Delay(10);
    Motor_stop(10);
    HAL_Delay(10);
    Motor_stop(11);
    HAL_Delay(10);
}
/*
 * 功能：修改电机的PID参数
 * k_p:比例系数P
 * k_w:微分系数D
 */
void Change_motor_PID(float k_p,float k_w)
{
    K_P = k_p;
    K_W = k_w;
}

//对伪8自由度的8个电机的速度同时限制
void AllLegsSpeedLimit(float speedlimit)
{
    //no.1 leg
    AngleLoop[1].Output_limit = speedlimit;
    AngleLoop[2].Output_limit = speedlimit;

    //no.2 leg
    AngleLoop[3].Output_limit = speedlimit;
    AngleLoop[4].Output_limit = speedlimit;

    //no.3 leg
    AngleLoop[5].Output_limit = speedlimit;
    AngleLoop[6].Output_limit = speedlimit;

    //no.4 leg
    AngleLoop[7].Output_limit = speedlimit;
    AngleLoop[8].Output_limit = speedlimit;
}

//每个腿的速度限制
void LegSpeedLimit(uint8_t LegId, float speedlimit)
{
    AngleLoop[LegId].Output_limit = AngleLoop[LegId].Output_limit = speedlimit;
}

//前后腿的速度限制
void FBLegsSpeedLimit(uint8_t Leg_FB, float speedlimit)
{
    if(Leg_FB==Leg_Front) {
        //no.1 leg
        AngleLoop[1].Output_limit = speedlimit;
        AngleLoop[2].Output_limit = speedlimit;
        //no.3 leg
        AngleLoop[5].Output_limit = speedlimit;
        AngleLoop[6].Output_limit = speedlimit;
    }
    else {
        //no.2 leg
        AngleLoop[3].Output_limit = speedlimit;
        AngleLoop[4].Output_limit = speedlimit;
        //no.4 leg
        AngleLoop[7].Output_limit = speedlimit;
        AngleLoop[8].Output_limit = speedlimit;
    }
}

/**
 * 对1号腿和2号腿的速度进行限制
 * @param speedlimit 速度最大值
 */
void no1_2_LegsSpeedLimit(float speedlimit)
{
    //no.1 leg
    AngleLoop[1].Output_limit = speedlimit;
    AngleLoop[2].Output_limit = speedlimit;

    //no.2 leg
    AngleLoop[4].Output_limit = speedlimit;
    AngleLoop[5].Output_limit = speedlimit;
}

/**
 * 对3号腿和4号腿的速度进行限制
 * @param speedlimit 速度最大值
 */
void no3_4_LegsSpeedLimit(float speedlimit)
{
    //no.3 leg
    AngleLoop[7].Output_limit = speedlimit;
    AngleLoop[8].Output_limit = speedlimit;

    //no.4 leg
    AngleLoop[10].Output_limit = speedlimit;
    AngleLoop[11].Output_limit = speedlimit;
}

//对伪8自由度的8个电机的力矩同时限制
void AllLegsTorqueLimit(float torquelimit)
{
    for(uint8_t i=1;i<9;i++)
    {
        SpeedLoop[i].Output_limit = torquelimit;
    }
}

void leg_pos_controll(void )
{
    //将目标角度放入各个电机的角度环并进行角度环PID计算

    //no3.leg
    SetPoint(&AngleLoop[5],AngleWant_MotorX[5],5);
    SetPoint(&AngleLoop[6],AngleWant_MotorX[6],6);
    PID_PosLocCalc(&AngleLoop[5],end_pos[5]);
    PID_PosLocCalc(&AngleLoop[6],end_pos[6]);

    //no4.leg
    SetPoint(&AngleLoop[7],AngleWant_MotorX[7],7);
    SetPoint(&AngleLoop[8],AngleWant_MotorX[8],8);
    PID_PosLocCalc(&AngleLoop[7],end_pos[7]);
    PID_PosLocCalc(&AngleLoop[8],end_pos[8]);

//    if(gpstate == 10 || gpstate == 11)
//        usart_printf("%f\n",AngleLoop[1].Out_put);


    //如果只使用位置环，则需要直接将位置环的输出送给电机
    if(OnlyPosLoop)
    {
        //输出
        motor_speed_controll_with_kw(5,AngleLoop[5].Out_put,speed_kp);
        osDelay(1);
        motor_speed_controll_with_kw(6,AngleLoop[6].Out_put,speed_kp);
        osDelay(1);
        motor_speed_controll_with_kw(7,AngleLoop[7].Out_put,speed_kp);
        osDelay(1);
        motor_speed_controll_with_kw(8,AngleLoop[8].Out_put,speed_kp);
        osDelay(1);
    }
}

void leg_pos_controll02(void )
{
    //位置式PID函数PID_PosLocCalc（）对1和2号腿做了特殊处理，让1号腿和2号腿跳跃的时候kp大一些，因为跳跃的时候重心靠前，前腿不容易起来
    //将目标角度放入各个电机的角度环并进行角度环PID计算
    //no.1 leg
    SetPoint(&AngleLoop[1],AngleWant_MotorX[1],1);
    SetPoint(&AngleLoop[2],AngleWant_MotorX[2],2);
    PID_PosLocCalc(&AngleLoop[1],end_pos[1]);
    PID_PosLocCalc(&AngleLoop[2],end_pos[2]);

    //no.2 leg
    SetPoint(&AngleLoop[3],AngleWant_MotorX[3],3);
    SetPoint(&AngleLoop[4],AngleWant_MotorX[4],4);
    PID_PosLocCalc(&AngleLoop[3],end_pos[3]);
    PID_PosLocCalc(&AngleLoop[4],end_pos[4]);

//    usart_printf("%d\r\n",AngleLoop[1].Out_put);

    //如果只使用位置环，则需要直接将位置环的输出送给电机
    if(OnlyPosLoop)
    {
        //输出
        motor_speed_controll_with_kw(1,AngleLoop[1].Out_put,speed_kp);
        osDelay(1);
        motor_speed_controll_with_kw(2,AngleLoop[2].Out_put,speed_kp);
        osDelay(1);
        motor_speed_controll_with_kw(3,AngleLoop[3].Out_put,speed_kp);
        osDelay(1);
        motor_speed_controll_with_kw(4,AngleLoop[4].Out_put,speed_kp);
        osDelay(1);
    }
}

/*！
 * 速度环控制
 */
void Speed_Controll(void )
{
    if(!OnlyPosLoop)        //如果只启用位置环，则速度环函数名存实亡。
    {
        //PID计算
        for(uint8_t j = 7;j < 12;j++)
        {
            if(j == 9)
            {
                //伪8自由度不对0号，3号，6号，9号电机进行控制。for循环从1开始，所以不用对0号电机判断
            } else{
//                SetPoint_Speed(&SpeedLoop[j],AngleLoop[j].Out_put);
                PID_PosLocCalc(&SpeedLoop[j],real_speed[j]);
            }
        }

        //输出
        for (uint8_t i = 7; i < 12; i++)
        {
            if (i == 9)
            {
                //伪8自由度不对0号，3号，6号，9号电机进行控制。for循环从1开始，所以不用对0号电机判断
            } else {
                motor_torque_controll(i, SpeedLoop[i].Out_put);
                osDelay(1);
            }
        }
//        for (uint8_t i = 1; i < 12; i++)
//        {
//            if (i == 3 || i == 6 || i == 9)
//            {
//                //伪8自由度不对0号，3号，6号，9号电机进行控制。for循环从1开始，所以不用对0号电机判断
//            } else {
//                motor_torque_controll(i, SpeedLoop[i].Out_put);
//                user_delaynus_tim(200);
//            }
//        }
    }
}

void Speed_Controll02(void )
{
    if(!OnlyPosLoop)        //如果只启用位置环，则速度环函数名存实亡。
    {
        //PID计算
        for(uint8_t j = 1;j < 9;j++)
        {
//            SetPoint_Speed(&SpeedLoop[j],AngleLoop[j].Out_put);
            PID_PosLocCalc(&SpeedLoop[j],real_speed[j]);
        }

        //输出
        for (uint8_t i = 1; i < 9; i++)
        {
            motor_torque_controll(i, SpeedLoop[i].Out_put);
            osDelay(1);
        }
    }
}

