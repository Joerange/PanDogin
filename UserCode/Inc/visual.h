//
// Created by 1 on 2024-01-19.
//

#ifndef PANDOGIN_DOG_VISUAL_H
#define PANDOGIN_DOG_VISUAL_H

#include "main.h"
#include "queue.h"
#include "cmsis_os.h"

#define Length_of_visual 4

typedef union
{
    uint8_t data_8[4];
    float distance;
}Visial_data;

extern osMessageQId VisialHandle;
extern uint8_t VISUAL_REC[Length_of_visual];
extern Visial_data visual;

void visual_process(void);

#endif //PANDOGIN_DOG_VISUAL_H
