//
// Created by 1 on 2024-01-19.
//

#ifndef PANDOGIN_DOG_VISUAL_H
#define PANDOGIN_DOG_VISUAL_H

#include "main.h"
#include "queue.h"
#include "cmsis_os.h"

#define Length_of_visual 7

typedef struct
{
    uint16_t distance;
}Visial_data;

extern osMessageQId VisialHandle;
extern uint8_t VISUAL_REC[Length_of_visual];
extern Visial_data data;

void visual_process(void);

#endif //PANDOGIN_DOG_VISUAL_H
