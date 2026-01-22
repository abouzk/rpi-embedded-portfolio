#include "../include/i2c_driver.h"
#include "engr2350_msp432.h"
#include <stdlib.h>
#include <stdio.h>

// Function Prototypes
void GPIO_init(void);
void timer_init(void);
void timer_ISR(void);
void CCR_ISR(void);
uint16_t readCompass(void);
uint16_t readRanger(void);

// Global Variables
uint16_t heading, compass, distance, distance1 = 0;
int16_t target_compass = 0;
uint8_t data[4];

// Hardware Constants
#define COMPASS_I2C_ADDR    0x60
#define RANGER_I2C_ADDR     0x70
#define RANGER_START_CMD    0x51
#define MOTOR_STOP_DELAY    2400000 // approx 100ms

// Timer Configs
Timer_A_UpModeConfig timer1_config;
Timer_A_ContinuousModeConfig timer3_config;
Timer_A_CompareModeConfig timer_compare_config_left, timer_compare_config_right;


int main(void){
    SysInit();
    GPIO_init();
    I2C_init();
    timer_init();

    while(1){
        // State 1: Move Forward until Obstacle
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7); // Motors OFF
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4 | GPIO_PIN5); // Direction FWD
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);// Motors ON
        
        distance1 = readRanger();
        printf("Distance: %u\r\n", distance1); // Debug logging - disable for real-time operation
        __delay_cycles(2400000); // 100ms delay

        // Blocking wait for obstacle (Simple Sequential Logic)
        while(distance1 > 25){
            distance1 = readRanger();
            printf("Path Clear: %u\r\n", distance1); // Debug logging - disable for real-time operation
            __delay_cycles(2400000); 
        }

        // State 2: Stop and Scan
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7); // Stop
        compass = readCompass();
        printf("Heading: %u\r\n", compass); // Debug logging - disable for real-time operation
        
        // Calculate Turn Target (-90 degrees)
        target_compass = compass - 900;
        if(target_compass < 0){
            target_compass += 3600;
        }

        // State 3: Execute Turn
        while(abs(target_compass - compass) > 10){
            GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4);
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7); 
            __delay_cycles(2400000); 
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);
            compass = readCompass();
        }

        // State 4: Re-Check and Adjust
        __delay_cycles(2400000); 
        distance1 = readRanger();
        
        if(distance1 < 25){
            // Obstacle still there? Turn 180.
            target_compass = compass + 1800;
            if(target_compass > 3600) target_compass -= 3600;
            
            while(abs(target_compass - compass) > 10){
                GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN5);
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4);
                GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);
                __delay_cycles(240000); // Faster checks during fine adjustment
                GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);
                compass = readCompass();
            }
        }
    }
}


// Function declarations:
void GPIO_init(){
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN4 | GPIO_PIN5);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P10, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P10, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1,GPIO_PIN6,GPIO_PRIMARY_MODULE_FUNCTION); //SDA
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1,GPIO_PIN7,GPIO_PRIMARY_MODULE_FUNCTION); //SCL

}

void timer_init(){
    timer1_config.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
        timer1_config.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
        timer1_config.timerPeriod = 800;
        timer1_config.timerClear = TIMER_A_DO_CLEAR;
        Timer_A_configureUpMode(TIMER_A0_BASE,&timer1_config);

        timer_compare_config_left.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
        timer_compare_config_left.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
        timer_compare_config_left.compareValue = 240;
        Timer_A_initCompare(TIMER_A0_BASE, &timer_compare_config_left);

        timer_compare_config_right.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
        timer_compare_config_right.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
        timer_compare_config_right.compareValue = 240;
        Timer_A_initCompare(TIMER_A0_BASE, &timer_compare_config_right);

        Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
}

uint16_t readCompass(){
    I2C_readData(EUSCI_B0_BASE,COMPASS_I2C_ADDR,2,data,2);
    heading=(data[0]<<8) | data[1];
    return heading;
}

uint16_t readRanger(){
    //putchar('a'); //checkpoints 1
    I2C_readData(EUSCI_B0_BASE,RANGER_I2C_ADDR,2,data,2);
    //putchar('b'); //2
    distance=(data[0]<<8) | data[1];
    //putchar('c'); //3
    data[0]=RANGER_START_CMD;
    //putchar('d'); //4
    I2C_writeData(EUSCI_B0_BASE,RANGER_I2C_ADDR,0,data,1);
    //putchar('e'); //5 checkpoints to make sure each part is run
    return distance;
}

