#ifndef _ABSTRACT_CAN_H_
#define _ABSTRACT_CAN_H_

#include "abstractSTM32.h"
#include <libopencm3/stm32/can.h>

#include <stdbool.h>

/** Struct for storing data used for CAN initialization */
struct abst_can
{
    /** Number of CAN (1, 2). */
    uint8_t can_num : 2;
    
    /** Time triggered communication mode */
    uint8_t ttcm : 1;
    
    /** Automatic bus-off management */
    uint8_t abom : 1;
    
    /** Automatic wakeup mode. */
    uint8_t awum : 1;
    
    /** No automatic retransmission */
    uint8_t nart : 1;
    
    /** Receive FIFO locked mode. */
    uint8_t rflm : 1;
    
    /** Transmit FIFO priority */
    uint8_t txfp : 1;
    
    /** Resynchronization time quanta jump width. */
    uint32_t sjw;
    
    /** Time segment 1 time quanta width. */
    uint32_t ts1;
    
    /** Time segment 2 time quanta width. */
    uint32_t ts2;
    
    /** Baud rate prescaler */
    uint16_t brp : 9;
    
    /** Loopback mode */
    uint8_t loopback : 1;
    
    /** Silent mode */
    uint8_t silent : 1;
};

/** Struct for storing data used for 32 bit CAN filter initialization */
struct abst_can_filter_32_bit
{
    /** Filter ID */
    uint8_t filter_id;
    
    /** First message ID to match. */
    uint32_t id1;
    
    /** Second message ID to match. */
    uint32_t id2;
    
    /** FIFO ID. */
    uint8_t fifo;
    
    /** Enable filter */
    uint8_t enable;
};

enum abst_errors abst_can_init(const struct abst_can *can);

void abst_can_init_filter_32_bit(const struct abst_can_filter_32_bit *filter);

uint8_t abst_can_get_fifo_pending(uint8_t can_num, uint8_t fifo_num);

#endif // _ABSTRACT_CAN_H_
