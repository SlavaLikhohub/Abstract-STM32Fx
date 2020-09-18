/*
 * This file contains helper functions for initialization of abstractSTM32Fx library
 */
#ifndef _ABSTRACT_INIT_H_
#define _ABSTRACT_INIT_H_

#include "abstractSTM32.h"
#include <stdint.h>

void _abst_init_systick(uint32_t systick_fr, uint32_t anb);

void _abst_init_hard_pwm_tim1(uint32_t anb, uint32_t hard_pwm_freq);

enum abst_errors _abst_adc_init_single_conv(uint8_t adc_n, uint8_t prescale);

enum abst_errors _abst_init_adc_channel(const struct abst_pin *pin_ptr);

void _abst_init_pins(   uint8_t port, 
                        uint8_t mode, 
                        uint8_t speed, 
                        uint8_t otype, 
                        uint8_t pull_up_down,
                        uint8_t af,
                        uint8_t af_dir,
                        uint16_t num);

#endif // _ABSTRACT_INIT_H_