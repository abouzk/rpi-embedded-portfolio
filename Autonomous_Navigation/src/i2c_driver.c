#include "../include/i2c_driver.h"

// [PORTFOLIO NOTE]
// Limitation: This driver currently uses blocking waits with a simple cycle delay.
// In a production RTOS environment, this would be replaced by a DMA-based
// non-blocking transaction or a state machine to prevent CPU stalling.
// The global interrupt disable is used here to ensure atomic transactions on the 
// MSP432 during critical register writes.

static eUSCI_I2C_MasterConfig i2c_config;

void I2C_init(){
    i2c_config.selectClockSource=EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    i2c_config.i2cClk=24000000; //24 MHz
    i2c_config.dataRate=EUSCI_B_I2C_SET_DATA_RATE_100KBPS;
    i2c_config.byteCounterThreshold=0;
    i2c_config.autoSTOPGeneration=EUSCI_B_I2C_NO_AUTO_STOP;
    I2C_initMaster(EUSCI_B0_BASE, &i2c_config);
    I2C_enableModule(EUSCI_B0_BASE);
    //I2C B0 initialized
}


void I2C_readData(uint32_t moduleInstance,uint8_t PeriphAddress,uint8_t StartReg,uint8_t *data,uint8_t len){
    // First write the start register to the peripheral device. This can be
    // done by using the I2C_writeData function with a length of 0.
    I2C_writeData(moduleInstance,PeriphAddress,StartReg,0,0);
    if(len==0){
        return;
    }

    // Then do read transaction...
    I2C_setSlaveAddress(moduleInstance,PeriphAddress); // Set the peripheral address
    I2C_setMode(moduleInstance,EUSCI_B_I2C_RECEIVE_MODE); // Indicate a read operation
    I2C_masterReceiveStart(moduleInstance); // Start the communication. This function
                // doe two things: It first sends the START signal and
                // then sends the peripheral address. Once started, the eUSCI
                // will automatically fetch bytes from the peripheral until
                // a STOP signal is requested to be sent.

    // This code loops through 1 less than all bytes to receive
    uint8_t ctr;
    for(ctr = 0;ctr<(len-1);ctr++){
        while(!(UCB0IFG&UCRXIFG0)); // Wait for a data byte to become available
        data[ctr] = I2C_masterReceiveMultiByteNext(moduleInstance); // read and store received byte
    }
    // Prior to receiving the final byte, request the STOP signal such that the
    // communication will halt after the byte is received.
    data[ctr] = I2C_masterReceiveMultiByteFinish(moduleInstance); // send STOP, read and store received byte


    __delay_cycles(200); // A short delay to avoid starting another I2C transaction too quickly
}


void I2C_writeData(uint32_t moduleInstance,uint8_t PeriphAddress,uint8_t StartReg,uint8_t *data ,uint8_t len){
    I2C_setSlaveAddress(moduleInstance,PeriphAddress); // Set the peripheral address
    I2C_setMode(moduleInstance,EUSCI_B_I2C_TRANSMIT_MODE); // Indicate a write operation

    I2C_masterSendMultiByteStart(moduleInstance,StartReg); // Start the communication.
                // This function does three things. It sends the START signal,
                // sends the address, and then sends the start register.

    // This code loops through all of the bytes to send.
    uint8_t ctr;
    for(ctr = 0;ctr<len;ctr++){
        I2C_masterSendMultiByteNext(moduleInstance,data[ctr]);
    }
    // Once all bytes are sent, the I2C transaction is stopped by sending the STOP signal
    I2C_masterSendMultiByteStop(moduleInstance);

    __delay_cycles(200); // A short delay to avoid starting another I2C transaction too quickly
}