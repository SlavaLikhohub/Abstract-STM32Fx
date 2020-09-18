#ifndef _ABSTRACT_FLASH_H_
#define _ABSTRACT_FLASH_H_

#include "abstractSTM32.h"

enum abst_errors abst_flash_erase_page(uint32_t start_address);

enum abst_errors abst_flash_write(uint32_t start_address, 
                                  uint16_t num_elements, 
                                  uint32_t input_data[]);

enum absr_errors abst_flash_read(uint32_t start_address, 
                                 uint16_t num_elements, 
                                 uint32_t *output_data);

#endif // _ABSTRACT_FLASH_H_
