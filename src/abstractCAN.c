#include "abstractCAN.h"
#include "abstractSTM32.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/can.h>

#include <stddef.h>

static uint32_t abst_opencm_can_conv(uint8_t can)
{
    switch (can) {
        case 1:
            return CAN1;
            break;
        case 2:
            return CAN2;
            break;
        default:
            return NULL;
    }
}

static uint32_t abst_opencm_can_rcc_conv(uint8_t can)
{
#ifdef STM32F1
    return RCC_CAN;
#endif // STM32F1
#ifdef STM32F4
    switch (can) {
        case 1:
            return RCC_CAN1;
            break;
        case 2:
            return RCC_CAN2;
            break;
        default:
            return NULL;
    }
#endif // STM32F4
}


/**
 * Initialize the CAN
 * 
 * :param: Pointer to a :c:type:`abst_can` with filled parameters
 * :return: Error code according to :c:type:`abst_errors`.
 */ 
enum abst_errors abst_can_init(const struct abst_can *can)
{
    uint32_t opencm_can = abst_opencm_can_conv(can->can_num);
    uint32_t opencm_can_rcc = abst_opencm_can_rcc_conv(can->can_num);
    if (!opencm_can)
        return ABST_WRONG_PARAMS;
    
#ifdef STM32F1
    rcc_periph_clock_enable(RCC_AFIO);
#endif //STM32F1
    
    rcc_periph_clock_enable(opencm_can_rcc);
    
    can_reset(opencm_can);
    bool init_status = can_init(opencm_can,
                                can->ttcm, // Time triggered comm mode
                                can->abom,  // Automatic bus-off management
                                can->awum, // Automatic wakeup mode
                                can->nart, // No automatic retransmission
                                can->rflm, // Receive FIFO locked mode
                                can->txfp, // Transmit FIFO priority
                                can->sjw, //  Resynchronization jump width
                                can->ts1, // Time segment 1
                                can->ts2, // Time segment 2
                                can->brp, // Baud rate prescaler
                                can->loopback, // Loop back mode
                                can->silent // Silent mode
                               );

    if (init_status)
        return ABST_OPERATION_FAILED;
    
    return ABST_OK;
}

/**
 * Initialize CAN 32 bit filter
 * 
 * :param: Pointer to a :c:type:`abst_can_filter_32_bit` with filled parameters
 * :return: Error code according to :c:type:`abst_errors`.
 */
void abst_can_init_filter_32_bit(const struct abst_can_filter_32_bit *filter)
{
    can_filter_id_list_32bit_init(filter->filter_id,
                                  filter->id1,
                                  filter->id2,
                                  filter->fifo,
                                  filter->enable);
    abst_delay_ms(50);
}

/**
 * Initialize CAN 16 bit filter
 * 
 * :param: Pointer to a :c:type:`abst_can_filter_16_bit` with filled parameters
 * :return: Error code according to :c:type:`abst_errors`.
 */
void abst_can_init_filter_16_bit(const struct abst_can_filter_16_bit *filter)
{
    can_filter_id_list_16bit_init(filter->filter_id,
                                  filter->id1,
                                  filter->id2,
                                  filter->id3,
                                  filter->id4,
                                  filter->fifo,
                                  filter->enable);
}

/**
 * Get number of messages are pending in the receive FIFO.
 * 
 * :param can_num: Number of CAN (1, 2).
 * :param fifo_num: Number of FIFO (1, 2).
 * :return: Number of messages are pending in the receive FIFO.
 */
uint8_t abst_can_get_fifo_pending(uint8_t can_num, uint8_t fifo_num)
{
    uint32_t opencm_can = abst_opencm_can_conv(can_num);
    if (fifo_num == 0)
        return CAN_RF0R(opencm_can) & CAN_RF0R_FMP0_MASK;
    else if (fifo_num == 1)
        return CAN_RF1R(opencm_can) & CAN_RF1R_FMP1_MASK;
    else
        return 0;
}

// void abst_can_read_fifo(uint8_t fifo, uint8_t data[], uint8_t N)
// {
//     
// }
