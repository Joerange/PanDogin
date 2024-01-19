//
// Created by hyz on 2023/11/25.
//

#ifndef MASTER_G4_DEMO_CANMOTOR_H
#define MASTER_G4_DEMO_CANMOTOR_H
#include "stm32g4xx_hal.h"
typedef struct
{
    int16_t set_voltage;          //电压
    uint16_t angle;          //转子角度 abs angle_now range:[0,8191]
    int16_t speed;          //转子速度
    int16_t torque_current;       //扭矩（以电流值为单位）
    uint8_t temperate;                 //电机温度
    int32_t total_angle;          //转子转过的总角度
    int16_t total_cnt;            //转子转过的总圈数
    uint16_t offset_angle;        //上电时的转子位置（初始位置）
    uint16_t last_angle;          //abs angle_now range:[0,8191]
    uint32_t msg_cnt;              //消息计数值，收到一次就+1
}moto_info_t;
extern moto_info_t struct_debug1[8];
extern void motor_info_record(moto_info_t *ptr, uint8_t *data);
extern void MyFDCan1_config(void);
extern void MyFDCan2_config(void);
extern void MyFDCan3_config(void);
extern void set_current(FDCAN_HandleTypeDef *_hcan, int16_t id_range, int16_t current1, int16_t current2, int16_t current3, int16_t current4);

#endif //MASTER_G4_DEMO_CANMOTOR_H
