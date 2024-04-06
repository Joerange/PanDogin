//
// Created by Joerange on 2024/1/18.
//

#ifndef ROBOMASTER_C_DEMO_JUMP_TASK_H
#define ROBOMASTER_C_DEMO_JUMP_TASK_H

#include "cmsis_os.h"

#define SpeedMode_JUMPPEDAL 8900 //go1的极限速度
enum JumpTypes
{
    Standard_Jump = 0,
    High_Jump =     1,
    Far_Jump =      2,
    Step_Jump =     3,
    Leap_Jump =     4,
    Test_Jump =     5,
};

extern uint8_t Jump_flag;

int ExecuteJump(uint8_t JumpType,float JumpAngle);
void StairJump(uint8_t stage);
void SeesawJump(uint8_t stage);
void FrontFlipJump(uint8_t mode);
void Bridge_Jump(uint8_t stage);
#endif //ROBOMASTER_C_DEMO_JUMP_TASK_H
