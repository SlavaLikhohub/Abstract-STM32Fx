#include <bits/stdint-uintn.h>
#include <stdint.h>
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

/**
 * Initialize the pin with the setting that specified in struct pin.
 * 
 * :param pin: pin struct with filled parameters.
 */
void abst_gpio_init(struct pin pin);

/**
 * Set value at the output of the pin.
 * 
 * :param pin: The pin struct with filled parameters.
 * :param value: The value to set. If is_inverse flag in struc pin is true value will be inversed.
 */
void abst_digital_write(struct pin pin, bool value);

/**
 * Read value from the pin via the input driver.
 * :param pin: The pin struct with filled parameters.
 * :return: Read value.
 */
bool abst_digital_read(struct pin pin);

/**
 * Set Pulse Wide Modulation at the pin.
 * :param pin: The pin struct with filled parameters.
 * :param value: the duty cycle: between 0 (always off) and 255 (always on).
 */
void abst_pwm_write(struct pin, uint8_t value);

/**
 * Read analog value from the pin via the Analog to Digital Converter (do not impelemented yet)
 * :param pin: The pin struct with filled parameters.
 * :return: Read value (12 bit)
 */
uint16_t abst_adc_read(struct pin);