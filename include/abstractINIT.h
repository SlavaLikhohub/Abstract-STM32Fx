/*
 * This file contains helper functions for initialization of abstractSTM32Fx library
 */
#ifndef _ABSTRACT_INIT_H_
#define _ABSTRACT_INIT_H_

#include "abstractSTM32.h"
#include <stdint.h>

void _abst_init_systick(uint32_t systick_fr, uint32_t anb);

void _abst_init_hard_pwm_tim1(uint32_t anb, uint32_t hard_pwm_freq);

uint8_t _abst_conv_mode(uint8_t mode, uint8_t speed);

uint8_t _abst_conv_cnf(uint8_t mode, uint8_t otype);

uint8_t _abst_conv_speed(uint8_t speed);

void _abst_init_pins(   uint8_t port, 
                        uint8_t mode, 
                        uint8_t speed, 
                        uint8_t otype, 
                        uint8_t pull_up_down,
                        uint8_t af,
                        uint16_t num);

#endif // _ABSTRACT_INIT_H_