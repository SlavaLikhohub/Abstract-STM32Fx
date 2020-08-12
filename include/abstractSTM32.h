#include <stdint.h>
#include <stdbool.h>
/**
 * Struct for storing data about pin
 */
struct pin {
    /** GPIO Port ID. Can be GPIOA...GPIOK */
    uint8_t port : 4;
    
    /** GPIO Pin number. Can be 0..15*/
    uint8_t num : 4;
    
    /** 
     * GPIO Pin direction 
     * Can be GPIO_MODE_INPUT, GPIO_MODE_OUTPUT
     */
    uint8_t dir : 2;

    /** 
     * GPIO Pin Analog/Digital Mode 
     * Can be GPIO_MODE_AF, GPIO_MODE_ANALOG, 
     */
    uint8_t mode : 2;
    
    /**
     * GPIO Output Pin Driver Type
     * Can be GPIO_OTYPE_OD (open drain), GPIO_OTYPE_PP (push pull)
     */
    uint8_t otype : 1;

    /**
     * GPIO Output Pin Speed
     * Can be GPIO_OSPEED_2MHZ, GPIO_OSPEED_25MHZ, GPIO_OSPEED_50MHZ, GPIO_OSPEED_100MHZ
     */
    uint8_t speed : 2;

    /** 
     * GPIO Output Pin Pullup
     * Can be GPIO_PUPD_NONE, GPIO_PUPD_PULLDOWN, GPIO_PUPD_PULLUP
     */
    uint8_t pull_up_down : 2;


    /** 
     * Flag that specify if the pin is inversed. 
     * If true, abst_digital_write and abst_digital_read functions inverse the value at the pin.
     */
    uint8_t is_inverse : 1;
};

void abst_gpio_init(struct pin pin);

void abst_digital_write(struct pin pin, bool value);

bool abst_digital_read(struct pin pin);

void abst_pwm_write(struct pin pin, uint16_t value);

uint16_t abst_adc_read(struct pin pin);

void delay(uint64_t miliseconds);
void delayMicroseconds(uint64_t microseconds);