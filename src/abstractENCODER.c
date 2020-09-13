#include "abstractENCODER.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include <stdlib.h>

static uint32_t abst_opencm_adv_timer_conv(uint8_t timer)
{
    switch (timer) {
        case 1:
            return TIM1;
            break;
#ifdef STM32F4
        case 8:
            return TIM8;
            break;
#endif // STM32F4
        default:
            return NULL;
    }
}

static uint32_t abst_opencm_adv_timer_rcc_conv(uint8_t timer)
{
    switch (timer) {
        case 1:
            return RCC_TIM1;
            break;
#ifdef STM32F4
        case 8:
            return RCC_TIM8;
            break;
#endif // STM32F4
        default:
            return NULL;
    }
}

static uint32_t abst_opencm_timer_div_conv(enum abst_timer_div div)
{
    switch (div) {
        case ABST_TIM_DIV_1:
            return TIM_CR1_CKD_CK_INT;
            break;
        case ABST_TIM_DIV_2:
            return TIM_CR1_CKD_CK_INT_MUL_2;
            break;
        case ABST_TIM_DIV_4:
            return TIM_CR1_CKD_CK_INT_MUL_4;
            break;
        default:
            return NULL; // TIM_CR1_CKD_CK_INT
    }
}
/**
 * Initialize the timer to encoder mode
 * 
 * :param encoder: Pointer to :c:type:`abst_encoder`.
 * :param timer: An advanced-control timer that shoud be used for encoder.
 * :param divider: Divition ratio of timer :c:type:`abst_timer_div`.
 */
enum abst_errors abst_encoder_init(struct abst_encoder *encoder, 
                                   uint8_t timer, 
                                   enum abst_timer_div divider)
{
    uint32_t opencm_timer = abst_opencm_adv_timer_conv(timer);
    uint32_t opencm_timer_rcc = abst_opencm_adv_timer_rcc_conv(timer);
    uint32_t opencm_timer_div = abst_opencm_timer_div_conv(divider);

    if (!opencm_timer || !opencm_timer_rcc)
        return ABST_WRONG_PARAMS;

    encoder->timer = opencm_timer;

    // Pointer to a register
    encoder->_reg = &TIM_CNT(opencm_timer);

    encoder->_counter = 0;

    rcc_periph_clock_enable(opencm_timer_rcc);

    timer_set_mode(opencm_timer, opencm_timer_div, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    timer_slave_set_mode(opencm_timer, TIM_SMCR_SMS_EM3); // Counting on TI1 and TI2 edges

    timer_set_period(opencm_timer, 65535);

    timer_continuous_mode(opencm_timer);

    timer_ic_set_input(opencm_timer, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(opencm_timer, TIM_IC2, TIM_IC_IN_TI2);


    timer_ic_set_polarity(opencm_timer, TIM_IC1, TIM_IC_RISING);
    timer_ic_set_polarity(opencm_timer, TIM_IC2, TIM_IC_RISING);


    timer_slave_set_polarity(opencm_timer, TIM_ET_RISING);

    // timer_update_on_any(opencm_timer);

    // timer_enable_irq(opencm_timer, )
    
    timer_ic_enable(opencm_timer, TIM_IC2);
    timer_ic_enable(opencm_timer, TIM_IC1);

    timer_enable_counter(opencm_timer);
}

/** 
 * Read the value from the encoder 
 * 
 * :param encoder: Pointer to :c:type:`abst_encoder`.
 * :return: Value from the encoder
 */
int64_t abst_encoder_read(struct abst_encoder *encoder)
{
    uint8_t sign = (encoder->_counter >= 0) ? 1 : -1;

    // return sign * ((abs(encoder->_counter) << 16) + *(encoder->_reg));
    return timer_get_counter(encoder->timer);
}