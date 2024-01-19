//
// Created by 1 on 2023-11-04.
//

#ifndef ROBOMASTER_A_LED_H
#define ROBOMASTER_A_LED_H

#include "gpio.h"

#define LED1_Flash HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_12)
#define LED2_Flash HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_13)

#endif //ROBOMASTER_A_LED_H
