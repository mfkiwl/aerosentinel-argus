#ifndef COMMON_PORTING_H__
#define COMMON_PORTING_H__

#include <stdint.h>
#include "user_define.h"




#define BUS_TIMEOUT             1000

typedef enum
{
 BHI260_GPIO_PIN_LOW = 0,
BHI260_GPIO_PIN_HIGH
}BHI260_GPIO_PinState;


enum
{
BHY2_INTERRUPT_IDLE = 0,
BHY2_INTERRUPT_READY = 1
};


void bst_delay_us(uint32_t period, void *intf_ptr);

int8_t SensorAPI_SPIx_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t SensorAPI_SPIx_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);



#endif
