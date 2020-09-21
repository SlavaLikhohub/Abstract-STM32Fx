#include "abstractLCD.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>

const uint16_t ABST_LCD_E_HOLD = 1; // us. Datasheet asks for 140 ns.
const uint16_t ABST_LCD_DATA_HOLD = 1; // us. Datasheet asks for 40 ns.
const uint16_t ABDT_LCD_RET_HOME = 1700; // us. Datasheet asks for 1520 us.
const uint16_t ABST_LCD_COM_EXEC = 50; // us.  Datasheet asks for 37 (+4) us.
const uint16_t ABST_LCD_START_TIME = 5000; // us.

/*
 * Helper function that configure pin from data specified by the user.
 * Note: malloc is used.
 * 
 * :param port: Port used to connect LCD.
 * :param num: Number of pin to configure.
 */
static struct abst_pin *conf_pin_out(uint8_t port, int32_t num)
{
    if (num == -1) // if port is not specified
        return NULL;
    
    struct abst_pin *result = malloc(sizeof(struct abst_pin));
    *result = (struct abst_pin){
        .port = port,
        .num = num,
        .mode = ABST_MODE_OUTPUT,
        .otype = ABST_OTYPE_PP,
        .speed = ABST_OSPEED_2MHZ,
        .pull_up_down = ABST_PUPD_PULLUP,
        .is_inverse = false
    };
    abst_gpio_init(result);
    return result;
}

/*
 * Helper function that configure pin group from data specified by the user.
 * Note: malloc is used.
 * 
 * :param port: Port used to connect LCD.
 * :param num: Numbers of pins to configure in form of bitmap.
 */
static struct abst_pin_group *conf_pin_group_out(uint8_t port, uint16_t num)
{
    struct abst_pin_group *result = malloc(sizeof(struct abst_pin_group));
    *result = (struct abst_pin_group){
        .port = port,
        .num = num,
        .mode = ABST_MODE_OUTPUT,
        .otype = ABST_OTYPE_PP,
        .speed = ABST_OSPEED_2MHZ,
        .pull_up_down = ABST_PUPD_NONE,
        .is_inverse = false
    };
    abst_group_gpio_init(result);
    return result;
}

/*
 * Helper function that stop program for a specific time using functions specified by the user.
 *
 * :param lcd_ptr: Pointer to :c:type:`abst_lcd` with filled parameters.
 * :param us: Time to stop in microseconds.
 */
static void lcd_delay(const struct abst_lcd *lcd_ptr, uint64_t us)
{
    if (lcd_ptr->delay_ms == NULL) {
        (*lcd_ptr->delay_us)(us);
        return;
    }
    else if (lcd_ptr->delay_us == NULL) {
        (*lcd_ptr->delay_us)((us % 1000) ? (1 + us / 1000) : (us / 1000));
        return;
    }
    else {
        (*lcd_ptr->delay_ms)(us / 1000);
        (*lcd_ptr->delay_us)(us % 1000);
        return;
    }
}

/*
 * Helper function to control RS and RW pins
 *
 * :param lcd_ptr: Pointer to :c:type:`abst_lcd` with filled parameters.
 * :param RC: Value of the RC pin.
 * :param RW: Value of the RW pin. If pin have not been specified value is ignored.
 */
static void rc_rw_control(const struct abst_lcd *lcd_ptr, bool RC, bool RW)
{
    abst_digital_write(lcd_ptr->_RC_pin_ptr_, RC);

    if (lcd_ptr->_RW_pin_ptr_ != NULL)
        abst_digital_write(lcd_ptr->_RW_pin_ptr_, RW);
}

/*
 * Helper function to send a byte to LCD. Using in case of 8-bit interface
 *
 * :param lcd_ptr: Pointer to :c:type:`abst_lcd` with filled parameters that has been initialized by :c:func:`abst_lcd_init`.
 * :param data: A byte to be send.
 */
static void send_byte(const struct abst_lcd *lcd_ptr, uint8_t data)
{
    abst_digital_write(lcd_ptr->_E_pin_ptr_, 1);
    abst_group_digital_write(lcd_ptr->_DB_group_ptr_, data);
    lcd_delay(lcd_ptr, ABST_LCD_E_HOLD);
    abst_digital_write(lcd_ptr->_E_pin_ptr_, 0);
    lcd_delay(lcd_ptr, ABST_LCD_COM_EXEC);
}

/*
 * Helper function to send half of a byte to LCD. Using in case of 4-bit interface
 *
 * :param lcd_ptr: Pointer to :c:type:`abst_lcd` with filled parameters that has been initialized by :c:func:`abst_lcd_init`.
 * :param data: Half of a byte to be send.
 */
static void send_half_byte(const struct abst_lcd *lcd_ptr, uint8_t data)
{
    // For now they are the same, but we are left space for maneuver in the future 
    send_byte(lcd_ptr, data);
}

/*
 * Helper function to send 1 byte of data or command to lcd. 
 * This function does not control RS and RW pins, only DB pins.
 * 
 * :param lcd_ptr: Pointer to :c:type:`abst_lcd` with filled parameters that has been initialized by :c:func:`abst_lcd_init`.
 * :param data: Byte of data to be send.
 */
static void send_message(const struct abst_lcd *lcd_ptr, uint8_t data)
{
    if (lcd_ptr->is_half_byte) {
        send_half_byte(lcd_ptr, data >> 4);
        send_half_byte(lcd_ptr, data & 0x0f);
    }
    else {
        send_byte(lcd_ptr, data);
    }
}

/**
 * Check the :c:type:`abst_lcd` and connect to a LCD.
 *
 * :param lcd_ptr: Pointer to :c:type:`abst_lcd` with filled parameters.
 * :return: Error code acording to the :c:type:`abst_errors`.
 *
 * If LCD with another connecting protocol is used redefine 
 * :c:func:`_abst_lcd_connect_half_byte` or :c:func:`_abst_lcd_connect_byte`.
 */
int abst_lcd_init(struct abst_lcd *lcd_ptr)
{
    // Check number of DB ports
    int DB_cnt = 0;
    for (uint8_t i = 0; i < sizeof(lcd_ptr->DB) * 8; i++) {
        if ((lcd_ptr->DB >> i) & 1)
            DB_cnt++;
    }
    if (lcd_ptr->is_half_byte && DB_cnt != 4)
        return ABST_WRONG_PARAMS;
    else if (!lcd_ptr->is_half_byte && DB_cnt != 8)
        return ABST_WRONG_PARAMS;
    
    // Configure VO pin
    lcd_ptr->_VO_pin_ptr_ = conf_pin_out(lcd_ptr->port, lcd_ptr->VO);

    // Configure RC pin
    lcd_ptr->_RC_pin_ptr_ = conf_pin_out(lcd_ptr->port, lcd_ptr->RC);

    // Configure RW pin
    lcd_ptr->_RW_pin_ptr_ = conf_pin_out(lcd_ptr->port, lcd_ptr->RW);

    // Configure E pin
    lcd_ptr->_E_pin_ptr_ = conf_pin_out(lcd_ptr->port, lcd_ptr->E);

    // Configure LED pin
    lcd_ptr->_LED_pin_ptr_ = conf_pin_out(lcd_ptr->port, lcd_ptr->LED);

    // Configure DB pin group
    lcd_ptr->_DB_group_ptr_ = conf_pin_group_out(lcd_ptr->port, lcd_ptr->DB);
    
    if (lcd_ptr->delay_ms == NULL && lcd_ptr->delay_us == NULL) {
        lcd_ptr->delay_ms = &abst_delay_ms;
        lcd_ptr->delay_us = &abst_delay_us;
    } 

    // Connect to a LCD
    if (lcd_ptr->is_half_byte)
        _abst_lcd_connect_half_byte(lcd_ptr);
    else
        _abst_lcd_connect_byte(lcd_ptr);
    
    return ABST_OK;
}


/**
 * Connect to a LCD via half byte protocol. Must not be called by user.
 *
 * :param lcd_ptr: Pointer to :c:type:`abst_lcd` with filled parameters.
 */
void __attribute__ ((weak)) _abst_lcd_connect_half_byte(struct abst_lcd *lcd_ptr)
{
    while (abst_time_ms() < ABST_LCD_START_TIME / 1000)
        lcd_delay(lcd_ptr, 1e3);

    rc_rw_control(lcd_ptr, 0, 0);

    send_half_byte(lcd_ptr, 0b0011);
    abst_delay_ms(5);
    send_half_byte(lcd_ptr, 0b0011);
    abst_delay_us(200);
    send_half_byte(lcd_ptr, 0b0011);
    abst_delay_us(200);
    send_half_byte(lcd_ptr, 0b0010);

    // Function set
    uint8_t DL = 0; // 4-bit interface
    uint8_t N = 1; // Number of lines = 2
    uint8_t F = 0; // Font type = 5x8
    abst_lcd_function_set(lcd_ptr, DL, N, F);

    // Turn off cursor, blinking 
    // Will configure after initialization
    abst_lcd_disp_contr(lcd_ptr, true, false, false);

    // Clear display
    abst_lcd_clear_disp(lcd_ptr);
    
    // Set entery mode
    // Direction - increment, shift - off.
    abst_lcd_entery_mode_set(lcd_ptr, true, false);
}

/**
 * Connect to a LCD via half byte protocol. Must not be called by user.
 *
 * :param lcd_ptr: Pointer to :c:type:`abst_lcd` with filled parameters.
 */
void __attribute__ ((weak)) _abst_lcd_connect_byte(struct abst_lcd *lcd_ptr)
{
    return;
}

/**
 * Display disappears and the cursor or blinking goes to the left edge of the display 
 * (in the first line if 2 lines are displayed). It also sets I/D to 1 (increment mode)
 * in entry mode. S of entry mode does not change.
 *
 * :param lcd_ptr: Pointer to :c:type:`abst_lcd` with filled parameters that has been initialized by :c:func:`abst_lcd_init`.
 */
void abst_lcd_clear_disp(const struct abst_lcd *lcd_ptr)
{
    rc_rw_control(lcd_ptr, 0, 0);
    send_message(lcd_ptr, 0b00000001);
    lcd_delay(lcd_ptr, ABDT_LCD_RET_HOME);
}

/**
 * Display control
 * 
 * :param lcd_ptr: Pointer to :c:type:`abst_lcd` with filled parameters that has been initialized by :c:func:`abst_lcd_init`.
 * :param display: Display ON/OFF
 * :param cursor:  Cursor ON/OFF
 * :param blinking: Cursor blinking ON/OFF
 */
void abst_lcd_disp_contr(const struct abst_lcd *lcd_ptr, 
                         bool display, 
                         bool cursor, 
                         bool blinking)
{
    rc_rw_control(lcd_ptr, 0, 0);
    send_message(lcd_ptr, 1 << 3 | display << 2 | cursor << 1 | blinking);
}

/**
 * Entery mode set
 * 
 * :param lcd_ptr: Pointer to :c:type:`abst_lcd` with filled parameters that has been initialized by :c:func:`abst_lcd_init`.
 * :param curs_dir: Cursor direction (1 - increment, 0 - decrement)
 * :param disp_shift: Shifts the entire display either to the right (I/D = 0) 
 * or to the left (I/D = 1) when S is 1 while printing new chars if this parameter is 1.
 */
 void abst_lcd_entery_mode_set(const struct abst_lcd *lcd_ptr, bool curs_dir, bool disp_shift)
 {
    rc_rw_control(lcd_ptr, 0, 0);
    send_message(lcd_ptr, 1 << 2 | curs_dir << 1 | disp_shift);
 }

/**
 * Function set. Do not call this function outside lcd initialization.
 *
 * :param lcd_ptr: Pointer to :c:type:`abst_lcd` with filled parameters that has been initialized by :c:func:`abst_lcd_init`.
 * :param interface: Interface of comunication (0 - 8 bit, 1 - 4 bit ?check me?)
 * :param lines_num: Number of lines (0 - one line, 1 - 2 lines ?check me?). 
 * :param font_type: Font type (0 - 5x11, 1 - 5x8 ?check me?)
 */
void abst_lcd_function_set(const struct abst_lcd *lcd_ptr, 
                           bool interface, 
                           bool lines_num, 
                           bool font_type)
{
    rc_rw_control(lcd_ptr, 0, 0);
    send_message(lcd_ptr, 1 << 5 | interface << 4 | lines_num  << 3 | font_type << 2);
}

/**
 * Put char to LCD.
 *
 * :param lcd_ptr: Pointer to :c:type:`abst_lcd` with filled parameters that has been initialized by :c:func:`abst_lcd_init`.
 * :param ch: Char to put.
 */
void abst_lcd_put_char(const struct abst_lcd *lcd_ptr, const char ch)
{
    rc_rw_control(lcd_ptr, 1, 0);
    send_message(lcd_ptr, ch);
}

/**
 * Put string to LCD.
 *
 * :param lcd_ptr: Pointer to :c:type:`abst_lcd` with filled parameters that has been initialized by :c:func:`abst_lcd_init`.
 * :param ch: Null terminated string to put.
 */
void abst_lcd_put_str(const struct abst_lcd *lcd_ptr, char *str)
{
    abst_lcd_put_str_sm(lcd_ptr, str, 0);
}

/**
 * Put string to LCD using formatting (max 200 chars)
 *
 * :param lcd_ptr: Pointer to :c:type:`abst_lcd` with filled parameters that has been initialized by :c:func:`abst_lcd_init`.
 * :param ch: Null terminated fromat string to put.
 * :param ...: Variables to be substituted into the message according to the **format**.
 */
void abst_lcd_put_str_f(const struct abst_lcd *lcd_ptr, const char *format, ...)
{
    va_list arg;

    uint8_t N = 200;
    char buff[N];

    va_start(arg, format);
    vsnprintf(buff, N, format, arg);
    va_end(arg);
    
    abst_lcd_put_str(lcd_ptr, buff);
}

/**
 * Put string to LCD smootly (with delay between putting chars).
 *
 * :param lcd_ptr: Pointer to :c:type:`abst_lcd` with filled parameters that has been initialized by :c:func:`abst_lcd_init`.
 * :param ch: Null terminated string to put.
 * :param interval_ms: Interval between putting chars in miliseconds.
 */
void abst_lcd_put_str_sm(const struct abst_lcd *lcd_ptr, char *str, uint32_t interval_ms)
{
    while (*str != '\0') {
        abst_lcd_put_char(lcd_ptr, *str);
        str++;

        if (interval_ms != 0)
            lcd_delay(lcd_ptr, interval_ms * 1000);
    }
}

/**
 * Set cursor to the given position.
 * 
 * :param lcd_ptr: Pointer to :c:type:`abst_lcd` with filled parameters that has been initialized by :c:func:`abst_lcd_init`.
 * :param row: Row to set. Starts from 0.
 * :param column: Column to set. Starts from 0.
 */
void abst_lcd_set_cursor(const struct abst_lcd *lcd_ptr, uint8_t row, uint8_t column)
{
    rc_rw_control(lcd_ptr, 0, 0);

    column += 1; // To make count from 0

    // Address in hexadecimal
    uint8_t addr = row * 0x27 + column;
    send_message(lcd_ptr, 1 << 7 | addr);
}
/**
 * Set brightness of LCD's LED
 * 
 * :param lcd_ptr: Pointer to :c:type:`abst_lcd` with filled parameters that has been initialized by :c:func:`abst_lcd_init`.
 * :param lvl: Level of brightness (0-255). 
 * If pwm_setting == ABST_NO_PWM value will be converted to bool.
 */
void abst_lcd_set_led(const struct abst_lcd *lcd_ptr, uint8_t lvl)
{
    if (lcd_ptr->_LED_pin_ptr_ == NULL)
        return;
    
    if (lcd_ptr->pwm_setting == ABST_NO_PWM)
        abst_digital_write(lcd_ptr->_LED_pin_ptr_, (bool)lvl);
    else if (lcd_ptr->pwm_setting == ABST_SOFT_PWM)
        abst_pwm_soft(lcd_ptr->_LED_pin_ptr_, lvl);
    else if (lcd_ptr->pwm_setting == ABST_HARD_PWM)
        abst_pwm_hard(lcd_ptr->_LED_pin_ptr_, lvl);
}
