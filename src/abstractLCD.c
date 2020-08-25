#include "abstractLCD.h"

/**
 * Check the lcd struct and connect to a LCD.
 *
 * :param lcd_ptr: Pointer to :c:type:`lcd` with filled parameters.
 * :return: Error code acording to the :c:type:`abst_errors`.
 *
 * If LCD with another connecting protocol is used redefine 
 * :c:func:_abst_lcd_connect_half_byte: or :c:func:_abst_lcd_connect_byte:.
 */
int abst_lcd_init(struct lcd *lcd_ptr)
{
    // Check number of DB ports
    int DB_cnt = 0;
    for (int i = 0; i < sizeof(lcd_ptr->DB), i++) {
        if ((lcd_ptr >> i) & 1)
            DB_cnt++;
    }
    if (lcd_ptr->is_half_byte && DB_cnt != 4)
        return ABST_WRONG_PARAMS;
    else if (!lcd_ptr->is_half_byte && DB_cnt != 8)
        return ABST_WRONG_PARAMS;
    
    // Configure DB pin group
    struct pin_group DB_group {
        .port = pin_ptr->port,
        .num = pin_ptr->DB,
        .dir = GPIO_MODE_OUTPUT,
        .mode = 0,
        .otype = GPIO_OTYPE_PP,
        .speed = GPIO_OSPEED_2MHZ,
        .pull_up_down = GPIO_PUPD_NONE,
        .is_inverse = false
    }
    lcd_ptr->_DB_group_ptr_ = malloc(sizeof pin_group);
    *(lcd_ptr->_DB_group_ptr_) = DB_group;
    
    abst_group_gpio_init(lcd_ptr->_DB_group_ptr_);

    // Connect to a LCD
    if (lcd_ptr->is_half_byte)
        return _abst_lcd_connect_half_byte(lcd_ptr);
    else
        return _abst_lcd_connect_byte(lcd_ptr);
}


/**
 * Connect to a LCD via half byte protocol. Must not be called by user.
 *
 * :param lcd_ptr: Pointer to :c:type:`lcd` with filled parameters.
 * :return: Error code acording to the :c:type:`abst_errors`.
 */
int __attribute__ ((weak)) _abst_lcd_connect_half_byte(struct lcd *lcd_ptr)
{
    
}