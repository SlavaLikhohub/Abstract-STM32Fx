#ifndef _ABSTRACT_STM32_H_
#define _ABSTRACT_STM32_H_

#include <stdint.h>
#include <stdbool.h>

/** 
 * Error codes
 */
enum abst_errors {
    /** Ok */
    ABST_OK = 0, 
    
    /** Unknown error */
    ABST_UNKNOWN_ERROR = -1, 
    
    /** Wrong parameters */
    ABST_WRONG_PARAMS = -2, 
    
    /** Not implemented yet */
    ABST_NOT_IMPLEMENTED = -3
};

/**
 * Struct for storing data about pin
 */
struct abst_pin {  
    /** 
     * GPIO Port identifier :c:type:`abst_pin_port`.
     */
    uint8_t port : 4;
    
    /** 
     * GPIO Pin number. Can be 0..15
     */
    uint8_t num : 4;
    
    /** 
     * GPIO Pin mode :c:type:`abst_pin_mode`
     */
    uint8_t mode : 2;

    /**
     * GPIO Alternate function (0-15).
     */
    uint8_t af : 4;

    /**
     * GPIO Output Pins Driver Type :c:type:`abst_pin_otype`
     */
    uint8_t otype : 1;

    /**
     * GPIO Output Pin Speed :c:type:`abst_pin_speed`
     */
    uint8_t speed : 4;

    /** 
     * GPIO Output Pin Pullup :c:type:`abst_pull_up_down`
     */
    uint8_t pull_up_down : 2;

    /** 
     * Flag that specify if the pin is inversed. 
     * If true, output and input are inversed.
     */
    uint8_t is_inverse : 1;

    /*
     * Service variable: Must not be changed by user.
     * Used to save PWM value on the pin and used by :c:func:`abst_pwm_soft`
     */
    uint8_t __pwm_value;
};

/**
 * Struct for storing data about a pin group
 */
struct abst_pin_group {  
    /** GPIO Port ID. :c:type:`abst_pin_port`*/
    uint8_t port : 4;
    
    /** Pin identifiers. If multiple pins are to be set, use bitwise OR '|' to separate them. */
    uint16_t num;
    
    /** 
     * GPIO Pins direction :c:type:`abst_pin_mode`
     */
    uint8_t mode : 2;

    /**
     * GPIO Alternate function (0-15).
     */
    uint8_t af : 4;

    /**
     * GPIO Output Pins Driver Type :c:type:`abst_pin_otype`
     */
    uint8_t otype : 1;

    /**
     * GPIO Output Pins Speed :c:type:`abst_pin_speed`
     */
    uint8_t speed : 2;

    /** 
     * GPIO Output Pins Pullup :c:type:`abst_pull_up_down`
     */
    uint8_t pull_up_down : 2;

    /** 
     * Flags that specify if the pins are inversed. 
     * If true, abst_digital_write and abst_digital_read functions inverse the value at the pin.
     */
    uint16_t is_inverse;
};
/**
 * Port indentifier
 */
enum abst_pin_port {
    /** Port A */
    ABST_GPIOA = 0,
    /** Port B */
    ABST_GPIOB,
    /** Port C */
    ABST_GPIOC,
    /** Port D */
    ABST_GPIOD,
    /** Port E */
    ABST_GPIOE,
    /** Port F */
    ABST_GPIOF,
    /** Port G */
    ABST_GPIOG,
    /** Port H */
    ABST_GPIOH,
    /** Port I */
    ABST_GPIOI,
    /** Port J */
    ABST_GPIOJ,
    /** Port K */
    ABST_GPIOK
};

/** Modes of pins */
enum abst_pin_mode
{
    /** Input */
    ABST_MODE_INPUT = 0,
    /** Output */
    ABST_MODE_OUTPUT,
    /** Alternative function*/
    ABST_MODE_AF,
    /** Analog*/
    ABST_MODE_ANALOG
};

/** Pin's output driver type */
enum abst_pin_otype
{
    /** Open drain */
    ABST_OTYPE_PP  = 0,
    /** Push pull */
    ABST_OTYPE_OD 
};

/** Pin pullup/pulldown configuration */
enum abst_pull_up_down
{
    /** Float (no pull up or down) */
    ABST_PUPD_NONE = 0,
    /** Pull down */
    ABST_PUPD_PULLDOWN,
    /** Pull up */
    ABST_PUPD_PULLUP
};

/** 
 * Pin speed.
 * If specified speed not avaliable for current platform will be chosen closest, 
 * but not less than specified if this is possible. 
 * If a specified speed greater than maximun avaliable, the maximum avaliable will be chosen.
 */
enum abst_pin_speed
{
    /** 2 MHz max. Avaliable in STM32F1, STM32F4. */
    ABST_OSPEED_2MHZ = 0,
    /** 10 MHz max. Avaliable in STM32F1. */
    ABST_OSPEED_10MHZ,
    /** 25 MHz max. Avaliable in STM32F4.*/
    ABST_OSPEED_25MHZ,
    /** 50 MHz max. Avaliable in STM32F1, STM32F4.*/
    ABST_OSPEED_50MHZ,
    /** 100 MHz max. Avaliable in STM32F4. */
    ABST_OSPEED_100MHZ
};

void abst_init(uint32_t anb, uint32_t hard_pwm_freq);

void abst_gpio_init(const struct abst_pin *pin_ptr);

void abst_group_gpio_init(const struct abst_pin_group *pin_gr_ptr);

void abst_sys_tick_handler(void);

void abst_digital_write(const struct abst_pin *pin_ptr, bool value);

void abst_group_digital_write(const struct abst_pin_group *pin_gr_ptr, uint16_t values);

void abst_toggle(const struct abst_pin *pin_ptr);

bool abst_digital_read(const struct abst_pin *pin_ptr);

uint16_t abst_group_digital_read(const struct abst_pin_group *pin_gr_ptr);

void abst_pwm_soft(struct abst_pin *pin_ptr, uint8_t value);

bool abst_stop_pwm_soft(struct abst_pin *pin_ptr);

void abst_pwm_hard(struct abst_pin *pin_ptr, uint8_t value);

uint16_t abst_adc_read(struct abst_pin *pin_ptr);

void abst_delay_ms(uint32_t miliseconds);

void abst_delay_us(uint32_t microseconds);

void abst_sleep_wfi(void);

void abst_stop_sleep(void);

uint32_t abst_time_ms(void);

#endif //_ABSTRACT_STM32_H_