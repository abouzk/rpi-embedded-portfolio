#include "../include/i2c_driver.h"
#include "engr2350_msp432.h"
#include <stdlib.h>
#include <stdio.h>

// Add function prototypes here, as needed.
void GPIO_init();
void timer_init();
void timer_ISR();
void CCR_ISR();
void motor_control();
uint16_t readCompass(),readRanger(),heading,distance;
uint8_t data[4];

// Add global variables here, as needed.
uint8_t timer_isr_var;
uint16_t distance1;
uint16_t compass;
int16_t target_compass=0;
uint32_t timer_edges_left;
uint32_t timer_counts_left = 0;
uint32_t measurement_counts_left = 0;
uint32_t current_PWM_left;
uint32_t timer_edges_right;
uint32_t timer_counts_right = 0;
uint32_t measurement_counts_right = 0;
uint32_t current_PWM_right;
uint16_t LPWM = 0;
uint16_t RPWM = 0;
Timer_A_UpModeConfig timer1_config;
Timer_A_UpModeConfig timer2_config;
Timer_A_CompareModeConfig timer_compare_config_left,timer_compare_config_right;
Timer_A_ContinuousModeConfig timer3_config;
Timer_A_CaptureModeConfig timer_captureL;
Timer_A_CaptureModeConfig timer_captureR;
uint32_t enc_totalR;
int32_t enc_counts_trackR,enc_countsR;
uint8_t enc_flagR;
uint32_t enc_totalL;
int32_t enc_counts_trackL,enc_countsL;
uint8_t enc_flagL;
uint16_t timeravg_R = 0;
uint16_t timeravg_L = 0;

// Main Function
int main(void){
    SysInit();
    GPIO_init();
    I2C_init();
    timer_init();
    //data[0]=0x51;
    //I2C_writeData(EUSCI_B0_BASE,0x70,0,data,1);
    // Place initialization code (or run-once) code here
    //LPWM =.3*timer1_config.timerPeriod;
    //RPWM =.3*timer1_config.timerPeriod;
    //Timer_A_setCompareValue(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_3,LPWM);
    //Timer_A_setCompareValue(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_4,RPWM);
    while(1){
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7); //motors off
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4 | GPIO_PIN5); //direction forwards
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);//motors on
        distance1 = readRanger();
        printf("distance: %u\r\n",distance1);
        __delay_cycles(2400000); // Wait 1/10 of a second
        while(distance1>25){
            distance1 = readRanger();
            printf("while distance: %u\r\n",distance1);
            __delay_cycles(2400000); // Wait 1/10 of a second
        }
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7); //motors off
        compass = readCompass();
        printf("compass: %u\r\n",compass);
        target_compass = compass - 900;
        if(target_compass < 0){
            target_compass += 3600;
        }
        while(abs(target_compass-compass)>10){
            //turn - pins may need to be swapped
            GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4);
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7); //motor on
            __delay_cycles(2400000); // Wait 1/10 of a second
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);
            compass = readCompass();
            printf("compass: %u\r\n",compass);
        }
        __delay_cycles(2400000); // Wait 1/10 of a second
        distance1 = readRanger();
        if(distance1 < 25){
            target_compass = compass + 1800;
            if(target_compass > 3600){
                target_compass -= 3600;
            }
            while(abs(target_compass-compass)>10){
                //turn
                GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN5);
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4);
                GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);
                __delay_cycles(240000);
                GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);
                compass = readCompass();
            }
        }
    }
}

// Add function declarations here as needed
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