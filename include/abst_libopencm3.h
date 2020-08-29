/**
 * This file will cover all difference in function names in libopencm3 
 * across different microcontrollers
 */

 /*
  * Check list
  * 
  * + F... => done in specifed STM32F, - => different (TODO)
  * F... the same in specifed STM32F...
  *
  * abstractSTM32.c
  * 
  * systick_set_frequency       S  Systick common for all cortexM
  * systick_counter_enable      S
  * systick_counter_disable     S
  * systick_interrupt_enable    S
  * systick_clear               S
  * 
  * nvic_enable_irq             F1, 4
  * 
  * rcc_periph_clock_enable     F1, 4
  * 
  * gpio_mode_setup             - Too different to change separately
  * gpio_set_output_options     - Implementing by making different abst_init_gpio
  * 
  * gpio_set                    F1, 4
  * gpio_clear                  F1, 4
  * gpio_port_write             F1, 4
  * gpio_toggle                 F1, 4
  * gpio_get                    F1, 4
  * gpio_port_read              F1, 4
  *
  */

#ifndef _ABST_LIBOPENCM3_H_
#define _ABST_LIBOPENCM3_H_



#endif //_ABST_LIBOPENCM3_H_