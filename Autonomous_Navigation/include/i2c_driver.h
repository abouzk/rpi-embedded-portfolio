#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H
#include "engr2350_msp432.h"

// Function Prototypes
void I2C_init(void);
void I2C_writeData(uint32_t moduleInstance, uint8_t PeriphAddress, uint8_t StartReg, uint8_t *data, uint8_t len);
void I2C_readData(uint32_t moduleInstance, uint8_t PeriphAddress, uint8_t StartReg, uint8_t *data, uint8_t len);

#endif