#ifndef _ABSTRACT_ADC_H_
#define _ABSTRACT_ADC_H_

#include "abstractSTM32.h"

/** Sample time of ADC in cycles for STM32F4 */
enum abst_adc_sample_time_f4
{
    /** 3 cycles */
    ABST_ADC_SMPR_SMP_3CYC = 0,
    /** 15 cycles */
    ABST_ADC_SMPR_SMP_15CYC,
    /** 28 cycles */
    ABST_ADC_SMPR_SMP_28CYC,
    /** 56 cycles */
    ABST_ADC_SMPR_SMP_56CYC,
    /** 84 cycles */
    ABST_ADC_SMPR_SMP_84CYC,
    /** 112 cycles */
    ABST_ADC_SMPR_SMP_112CYC,
    /** 144 cycles */
    ABST_ADC_SMPR_SMP_144CYC,
    /** 480 cycles */
    ABST_ADC_SMPR_SMP_480CYC,
};

/** 
 * Sample time of ADC in cycles for STM32F1.
 */
enum abst_adc_sample_time_f1
{
    /** 1.5 cycle */
    ABST_ADC_SMPR_SMP_1DOT5CYC = 0,
    /** 7.5 cycles */
    ABST_ADC_SMPR_SMP_7DOT5CYC,
    /** 13.5 cycles */
    ABST_ADC_SMPR_SMP_13DOT5CYC,
    /** 28.5 cycles */
    ABST_ADC_SMPR_SMP_28DOT5CYC,
    /** 41.5 cycles */
    ABST_ADC_SMPR_SMP_41DOT5CYC,
    /** 55.5 cycles */
    ABST_ADC_SMPR_SMP_55DOT5CYC,
    /** 71.5 cycles */
    ABST_ADC_SMPR_SMP_71DOT5CYC,
    /** 239.5 cycles */
    ABST_ADC_SMPR_SMP_239DOT5CYC
};

/** Resolution of ADC */
enum abst_adc_resolution 
{
    /** 12 bits */
    ABST_ADC_RES_12BIT = 0,
    /** 10 bits */
    ABST_ADC_RES_10BIT,
    /** 8 bits */
    ABST_ADC_RES_8BIT, 
    /** 6 bits */
    ABST_ADC_RES_6BIT 
};

uint16_t abst_adc_read(struct abst_pin *pin_ptr);

enum abst_errors _abst_init_adc_channel(const struct abst_pin *pin_ptr);

enum abst_errors abst_adc_init_single_conv(uint8_t adc_n, uint8_t prescale);

enum abst_errors abst_adc_read_cont(struct abst_pin *pins_arr[], 
                                    uint16_t dest_arr[], 
                                    uint8_t N, 
                                    uint8_t prescale, 
                                    uint8_t prior);

#endif // _ABSTRACT_ADC_H_