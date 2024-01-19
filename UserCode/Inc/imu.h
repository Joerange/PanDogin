//
// Created by Joerange on 2023/11/18.
//

#ifndef MY_SCUDOG_IMU_H
#define MY_SCUDOG_IMU_H

#include <stdint-gcc.h>
#include "Attitude_Slove.h"

enum AngleTypes
{
    Roll=0,
    Pitch=1,
    Yaw=2,
};
enum RawAccTypes
{
    RawAcc_x=0,
    RawAcc_y=1,
    RawAcc_z=2,
};
enum LinearAccTypes
{
    LinearAcc_x=0,
    LinearAcc_y=1,
    LinearAcc_z=2,
};
enum DistanceTypes
{
    Dis_x=0,
    Dis_y=1,
    Dis_z=2,
};

#define IMU_REC_LEN 50

typedef union
{
    uint8_t raw_data[12];    //原始数据
    float EulerAngle[3];//32位变量
    float RawAcc[3];    //xyz方向原始加速度
    float LinearAcc[3]; //xyz方向线性加速度
    float Velocity[3];  //xyz方向速度
    float Distance[3];  //xyz方向位移
    float Heave;
}imu_measure;

typedef union
{
    uint8_t raw_data[12];
    float EulerAngle[3];//32位变量
}IMU_EulerAngle_u;

extern float yawwant;
extern float pitchwant;
extern float rollwant;
extern IMU_EulerAngle_u IMU_EulerAngle;
extern uint8_t IMU_Control_Flag;
extern uint8_t IMU_RX_BUF[IMU_REC_LEN];

void IMU_Init(void);
void Close_Global_IMU_Control(void);
void IMU_AUTO_PID_SET(float kp,float ki,float kd,float SumError_limit);
void IMU_Data_Process(uint16_t rx_len);
void uart_rx_angle_init(uint32_t bound);
void AttitudeControl_Global(float roll_set,float pitch_set);
uint8_t IMU_WaitAngle(uint8_t angle_type, float takeoff_inclination, float imu_angle_half_range, uint8_t lock, uint8_t second_flag);
#endif //MY_SCUDOG_IMU_H
