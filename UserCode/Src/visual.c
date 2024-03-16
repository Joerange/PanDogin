//
// Created by 1 on 2024-01-19.
//
#include "visual.h"

uint8_t VISUAL_REC[Length_of_visual] = {0};
Visial_data visual;
uint8_t visual_control_flag = 0;

void visual_process(void)
{
    if(visual_control_flag == 1)
    {
        visual.distance = (visual.data_8[0]-48) * 1000.0f + (visual.data_8[1]-48) * 100.0f + (visual.data_8[2]-48) * 10.0f + (visual.data_8[3]-48) * 1.0f ;
        visual.offset = (visual.data_8[4]-48) * 100.0f + (visual.data_8[5]-48) * 10.0f + (visual.data_8[6]-48) * 1.0f ;
    }
}