//
// Created by 1 on 2024-01-19.
//
#include "visual.h"

uint8_t VISUAL_REC[Length_of_visual] = {0};
Visial_data data;

void visual_process(void)
{
    if(VISUAL_REC[0] == 'W')
    {
        data.distance =(short) ( (VISUAL_REC[1] << 8) | (VISUAL_REC[2]));

        xQueueOverwriteFromISR(VisialHandle,&data.distance,0);
    }
}