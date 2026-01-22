#include "../include/i2c_driver.h"
#include "engr2350_msp432.h"
#include <stdlib.h>
#include <stdio.h>

// Function Prototypes
void GPIO_init(void);
void timer_init(void);
void timer_ISR(void);
void CCR_ISR(void);
void motor_control(void);
uint16_t readCompass(void);
uint16_t readRanger(void);

// Global Variables
uint16_t heading = 0;
uint16_t distance = 0;
uint16_t distance1 = 0;
uint16_t compass = 0;
int16_t target_compass = 0;
uint8_t data[4];

// Timer Configs
Timer_A_UpModeConfig timer1_config;
Timer_A_ContinuousModeConfig timer3_config;
Timer_A_CaptureModeConfig timer_captureL, timer_captureR;
Timer_A_CompareModeConfig timer_compare_config_left, timer_compare_config_right;

// Encoder Variables
uint32_t enc_totalR, enc_totalL;
int32_t enc_counts_trackR, enc_counts_trackL;
int32_t enc_countsR, enc_countsL;
uint8_t enc_flagR, enc_flagL;
uint32_t timer_counts_right = 0, measurement_counts_right = 0;
uint32_t timer_counts_left = 0, measurement_counts_left = 0;
uint16_t timeravg_R = 0, timeravg_L = 0;


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
        printf("Distance: %u\r\n", distance1);
        __delay_cycles(2400000); // 100ms delay

        // Blocking wait for obstacle (Simple Sequential Logic)
        while(distance1 > 25){
            distance1 = readRanger();
            printf("Path Clear: %u\r\n", distance1);
            __delay_cycles(2400000); 
        }

        // State 2: Stop and Scan
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7); // Stop
        compass = readCompass();
        printf("Heading: %u\r\n", compass);
        
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

        timer3_config.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
        timer3_config.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
        timer3_config.timerClear = TIMER_A_DO_CLEAR;
        timer3_config.timerInterruptEnable_TAIE=TIMER_A_TAIE_INTERRUPT_ENABLE;
        Timer_A_configureContinuousMode(TIMER_A3_BASE,&timer3_config);

        timer_captureR.captureRegister=(TIMER_A_CAPTURECOMPARE_REGISTER_0);
        timer_captureR.captureMode=(TIMER_A_CAPTUREMODE_RISING_EDGE);
        timer_captureR.captureInputSelect=(TIMER_A_CAPTURE_INPUTSELECT_CCIxA);
        timer_captureR.synchronizeCaptureSource=(TIMER_A_CAPTURE_SYNCHRONOUS);
        timer_captureR.captureInterruptEnable=(TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE);
        Timer_A_initCapture(TIMER_A3_BASE, &timer_captureR);
        Timer_A_registerInterrupt(TIMER_A3_BASE,TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT,CCR_ISR);
        Timer_A_registerInterrupt(TIMER_A3_BASE,TIMER_A_CCR0_INTERRUPT,CCR_ISR);

        timer_captureL.captureRegister=(TIMER_A_CAPTURECOMPARE_REGISTER_1);
        timer_captureL.captureMode=(TIMER_A_CAPTUREMODE_RISING_EDGE);
        timer_captureL.captureInputSelect=(TIMER_A_CAPTURE_INPUTSELECT_CCIxA);
        timer_captureL.synchronizeCaptureSource=(TIMER_A_CAPTURE_SYNCHRONOUS);
        timer_captureL.captureInterruptEnable=(TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE);
        Timer_A_initCapture(TIMER_A3_BASE, &timer_captureL);
        Timer_A_registerInterrupt(TIMER_A3_BASE,TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT,CCR_ISR);
        Timer_A_registerInterrupt(TIMER_A3_BASE,TIMER_A_CCR0_INTERRUPT,CCR_ISR);


        Timer_A_startCounter(TIMER_A3_BASE,TIMER_A_CONTINUOUS_MODE);

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
    I2C_readData(EUSCI_B0_BASE,0x60,2,data,2);
    heading=(data[0]<<8) | data[1];
    return heading;
}

uint16_t readRanger(){
    //putchar('a'); //checkpoints 1
    I2C_readData(EUSCI_B0_BASE,0x70,2,data,2);
    //putchar('b'); //2
    distance=(data[0]<<8) | data[1];
    //putchar('c'); //3
    data[0]=0x51;
    //putchar('d'); //4
    I2C_writeData(EUSCI_B0_BASE,0x70,0,data,1);
    //putchar('e'); //5 checkpoints to make sure each part is run
    return distance;
}


void motor_control(){
    uint16_t direction = readCompass();
    __delay_cycles(24000000);
    uint16_t direction2 = readCompass();
    if((direction-direction2) > 10){
        RPWM -= 1;
    }
    else if((direction-direction2) < 10){
        RPWM += 1;
    }
}
// Add interrupt functions last so they are easy to find
void CCR_ISR(){
    if(Timer_A_getEnabledInterruptStatus(TIMER_A3_BASE)==TIMER_A_INTERRUPT_PENDING){
        Timer_A_clearInterruptFlag(TIMER_A3_BASE);
        enc_counts_trackL+=65536;
        enc_counts_trackR+=65536;
        //enc_flag=1;
    }

    else if(Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0) & TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG){
        Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        enc_totalR++;
        enc_countsR=enc_counts_trackR+Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        enc_counts_trackR = -Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        enc_flagR=1;
        timer_counts_right += enc_countsR;
        measurement_counts_right++;
    }
    else if(Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1) & TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG){
        Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        enc_totalL++;
        enc_countsL=enc_counts_trackL+Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        enc_counts_trackL= -Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        enc_flagL=1;
        timer_counts_left += enc_countsL;
        measurement_counts_left++;
    }
    if(measurement_counts_right == 6){
        timeravg_R = timer_counts_right/6;
        timer_counts_right = 0;
        measurement_counts_right = 0;
    }

    if(measurement_counts_left == 6){
        timeravg_L = timer_counts_left/6;
        timer_counts_left = 0;
        measurement_counts_left = 0;
        }

}