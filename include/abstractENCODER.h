#ifndef _ABSTRACT_ENCODER_H_
#define _ABSTRACT_ENCODER_H_

#include "abstractSTM32.h"

/** Data type for storing data about encoder */
struct abst_encoder
{
    /** Number of timer that is used. Can be only Advanced timers */
    uint32_t timer;

    /* Pointer to a register */
    uint16_t *_reg;

    /* External counter for overflows and underflows */
    int32_t _counter;
};

/** Divition ratio for timers */
enum abst_timer_div
{
    /** No division */
    ABST_TIM_DIV_1 = 0,

    /** Division by 2 */
    ABST_TIM_DIV_2,

    /** Division by 4 */
    ABST_TIM_DIV_4,
};

enum abst_errors abst_encoder_init(struct abst_encoder *encoder, 
                                   uint8_t timer, 
                                   enum abst_timer_div divider);

int64_t abst_encoder_read(struct abst_encoder *encoder);

void abst_encoder_interrupt_handler(struct abst_encoder *encoder);

#endif // _ABSTRACT_ENCODER_H_