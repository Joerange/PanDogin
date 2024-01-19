//
// Created by 1 on 2024-01-19.
//
#include "visual.h"

uint8_t VISUAL_REC[Length_of_visual] = {0};
float distance = 0;

void visual_process(void)
{
    while(VISUAL_REC[0] == 'W')
    {
        distance = VISUAL_REC[1] * 10 + VISUAL_REC[2] + VISUAL_REC[4] * 0.1 + VISUAL_REC[5] * 0.01;
        if(VISUAL_REC[6] == 'C')
            break;
    }
}