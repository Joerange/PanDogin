//
// Created by 19734 on 2022/12/26.
//
#include <stdint-gcc.h>
#include "crc16.h"

unsigned short CRC16_CCITT(const uint8_t *pdata, int len)
{    //正确的CRC16_CCITT计算程序
    unsigned short crc = 0x0000;//初始值
    int i, j;
    for (j = 0; j<len; j++)
    {
        crc ^= pdata[j];

        for (i = 0; i<8; i++) {
            if ((crc & 0x0001) >0) {
                crc = (crc >> 1) ^ 0x8408;//0x1021 翻转  0x8408
            }
            else
                crc >>= 1;
        }
    }
    return crc;
}

/**
 * 求浮点数a的绝对值
 * @param a
 * @return a的绝对值
 */
float my_abs(float a)
{
    if(a<0.0f)
        return -a;
    else
        return a;
}

