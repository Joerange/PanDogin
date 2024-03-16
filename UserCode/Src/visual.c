//
// Created by 1 on 2024-01-19.
//
#include "visual.h"

uint8_t VISUAL_REC[Length_of_visual] = {0};
Visial_data visual;
uint8_t visual_control_flag = 0;
float distance[5] = {200,200,200,200,200};
float Distance = 0;

void visual_process(void)
{
    static uint8_t count = 0;
    if(visual_control_flag == 1)
    {
        visual.distance = (visual.data_8[0]-48) * 1000.0f + (visual.data_8[1]-48) * 100.0f + (visual.data_8[2]-48) * 10.0f + (visual.data_8[3]-48) * 1.0f ;
        visual.offset = (visual.data_8[4]-48) * 100.0f + (visual.data_8[5]-48) * 10.0f + (visual.data_8[6]-48) * 1.0f ;

        distance[count] = visual.distance;
        if(count++ == 4)
            count = 0;

        Distance = (distance[0] + distance[1] + distance[2] + distance[3] + distance[4]) / 5;

        if(visual.distance < -3000)
            visual.distance = 200;
        if(visual.offset < -300)
            visual.offset = 0;
    }
}