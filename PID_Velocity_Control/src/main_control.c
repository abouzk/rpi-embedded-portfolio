#include "engr2350_msp432.h"

// Function Prototypes
void GPIOInit();
void TimerInit();
void ADCInit();
void Encoder_ISR();
void T1_100ms_ISR();

// Hardware Configuration
Timer_A_UpModeConfig TA0cfg;
Timer_A_UpModeConfig TA1cfg;
Timer_A_ContinuousModeConfig TA3cfg;
Timer_A_CompareModeConfig TA0_ccr3;
Timer_A_CompareModeConfig TA0_ccr4;
Timer_A_CaptureModeConfig TA3_ccr0;
Timer_A_CaptureModeConfig TA3_ccr1;

// Encoder total events
uint32_t enc_total_L, enc_total_R;

// Speed measurement variables
int32_t Tach_L_count, Tach_L, Tach_L_sum, Tach_L_sum_count, Tach_L_avg; 
int32_t Tach_R_count, Tach_R, Tach_R_sum, Tach_R_sum_count, Tach_R_avg; 

// Control Variables (Using float to prevent integer overflow/underflow bugs)
uint8_t run_control = 0;
uint8_t turn = 0;
float diff_speed, desired_wheel_l, desired_wheel_r;
float measured_speed_l, measured_speed_r;
float kp = 1.0; // Proportional Gain (Kp) - Tuned for stability
float corrected_l, corrected_r; 

// Control Architecture: Feedforward + Proportional Controller
// Note: MSP432 Cortex-M4F FPU enabled allows for floating point math 
// without significant cycle penalty in the 100ms control loop. Not viable on
// smaller MCUs without hardware FPU.

int main(void)
{
    SysInit();
    GPIOInit();
    ADCInit();
    TimerInit();

    __delay_cycles(24000000); // Wait 1s for system to settle
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);

    while(1){
        if(run_control){    // Run control loop @ 10Hz (100ms)
            run_control = 0;
            
            // --- 1. Odometry Logic ---
            float distance = (2.0 * 3.14159 * (.035 / 360.0)) * enc_total_L * 1000.0;
            
            if(distance >= 20 && distance < 95){
                turn = 0;
            } else if(distance >= 95){
                turn = 1;
            }

            // --- 2. Sensor Fusion (ADC Inputs) ---
            uint16_t speed_pot = ADC14_getResult(ADC_MEM0);
            float desired_speed = (40.0 * speed_pot / 16384.0) + 10.0; 

            uint16_t track_pot = ADC14_getResult(ADC_MEM1);
            float actual_r = (102.0 * track_pot / 16384.0) + 15.0;

            // --- 3. Differential Drive Mixing ---
            if(turn == 1){
                diff_speed = desired_speed * (0.5 * 0.0145 / actual_r);
            } else {
                diff_speed = 0;
            }

            desired_wheel_l = desired_speed - diff_speed;
            desired_wheel_r = desired_speed + diff_speed;

            // --- 4. Motor Safety (Deadband) ---
            if(desired_wheel_r < 10) { desired_wheel_r = 0; TA0_ccr3.compareValue = 0; }
            if(desired_wheel_l < 10) { desired_wheel_l = 0; TA0_ccr4.compareValue = 0; }
            
            // --- 5. Closed-Loop Control (Feedforward + Proportional) ---
            if(desired_wheel_l >= 10){
                // Safety: Prevent divide by zero if wheels are stopped
                if(Tach_L_avg > 0) measured_speed_l = 1500000.0 / Tach_L_avg;
                else measured_speed_l = 0;

                // Control Law: Output = Desired (Feedforward) + Kp * Error
                corrected_l = desired_wheel_l + kp * (desired_wheel_l - measured_speed_l);
                
                // Clamp Output to PWM limits
                if(corrected_l > 2300) corrected_l = 2300;
                if(corrected_l < 0) corrected_l = 0;
                
                Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, (uint16_t)corrected_l);
            }

            if(desired_wheel_r >= 10){
                if(Tach_R_avg > 0) measured_speed_r = 1500000.0 / Tach_R_avg;
                else measured_speed_r = 0;

                corrected_r = desired_wheel_r + kp * (desired_wheel_r - measured_speed_r);

                if(corrected_r > 2300) corrected_r = 2300;
                if(corrected_r < 0) corrected_r = 0;

                Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, (uint16_t)corrected_r);
            }
        }
    }
}


void ADCInit(){
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_SMCLK, 1, 1, 0);
    
    // Configure Memory for Potentiometer Inputs (A0, A1)
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A0, false);
    ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A1, false);
    
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true);
    ADC14_setSampleHoldTrigger(ADC_TRIGGER_ADCSC, false);
    ADC14_toggleConversionTrigger();
}

void GPIOInit(){
    // Input Buttons
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN4);
    
    // Motor Control Pins
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN4 | GPIO_PIN5);   // Direction
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);   // Enable
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4 | GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);

    // PWM Pins (P2.6, P2.7)
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
    
    // Encoder Pins (P10.4, P10.5)
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P10, GPIO_PIN4 | GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
}

void TimerInit(){
    // --- PWM Timer (30 kHz) ---
    TA0cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA0cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    TA0cfg.timerPeriod = 800;
    Timer_A_configureUpMode(TIMER_A0_BASE, &TA0cfg);

    // Left Wheel PWM
    TA0_ccr3.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
    TA0_ccr3.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    TA0_ccr3.compareValue = 0;
    Timer_A_initCompare(TIMER_A0_BASE, &TA0_ccr3);

    // Right Wheel PWM
    TA0_ccr4.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
    TA0_ccr4.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    TA0_ccr4.compareValue = 0;
    Timer_A_initCompare(TIMER_A0_BASE, &TA0_ccr4);

    // --- Encoder Timer (Continuous Mode) ---
    TA3cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA3cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    TA3cfg.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    Timer_A_configureContinuousMode(TIMER_A3_BASE, &TA3cfg);

    // Left Encoder Capture
    TA3_ccr0.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
    TA3_ccr0.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;
    TA3_ccr0.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
    TA3_ccr0.synchronizeCaptureSource = TIMER_A_CAPTURE_SYNCHRONOUS;
    TA3_ccr0.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    Timer_A_initCapture(TIMER_A3_BASE, &TA3_ccr0);

    // Right Encoder Capture
    TA3_ccr1.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    TA3_ccr1.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;
    TA3_ccr1.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
    TA3_ccr1.synchronizeCaptureSource = TIMER_A_CAPTURE_SYNCHRONOUS;
    TA3_ccr1.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    Timer_A_initCapture(TIMER_A3_BASE, &TA3_ccr1);

    // Interrupt Registration
    Timer_A_registerInterrupt(TIMER_A3_BASE, TIMER_A_CCR0_INTERRUPT, Encoder_ISR);
    Timer_A_registerInterrupt(TIMER_A3_BASE, TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT, Encoder_ISR);

    // --- Control Loop Timer (100ms / 10Hz) ---
    TA1cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA1cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64;
    TA1cfg.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    TA1cfg.timerPeriod = 37500;
    Timer_A_configureUpMode(TIMER_A1_BASE, &TA1cfg);
    Timer_A_registerInterrupt(TIMER_A1_BASE, TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT, T1_100ms_ISR);

    // Start Timers
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
    Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_CONTINUOUS_MODE);
}


void Encoder_ISR(){
    // Overflow Handling
    if(Timer_A_getEnabledInterruptStatus(TIMER_A3_BASE) == TIMER_A_INTERRUPT_PENDING){
        Timer_A_clearInterruptFlag(TIMER_A3_BASE);
        Tach_R_count += 65536;
        Tach_L_count += 65536;
        
    // Left Encoder Event
    }else if(Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0) & TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG){
        Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
        enc_total_R++;
        
        Tach_R = Tach_R_count + Timer_A_getCaptureCompareCount(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
        Tach_R_count = -Timer_A_getCaptureCompareCount(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
        
        Tach_R_sum_count++;
        Tach_R_sum += Tach_R;
        if(Tach_R_sum_count == 6){ // Moving Average Filter
            Tach_R_avg = Tach_R_sum / 6;
            Tach_R_sum_count = 0;
            Tach_R_sum = 0;
        }
        
    // Right Encoder Event
    }else if(Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1) & TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG){
        Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
        enc_total_L++;
        
        Tach_L = Tach_L_count + Timer_A_getCaptureCompareCount(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
        Tach_L_count = -Timer_A_getCaptureCompareCount(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
        
        Tach_L_sum_count++;
        Tach_L_sum += Tach_L;
        if(Tach_L_sum_count == 6){ // Moving Average Filter
            Tach_L_avg = Tach_L_sum / 6;
            Tach_L_sum_count = 0;
            Tach_L_sum = 0;
        }
    }
}

void T1_100ms_ISR(){
    Timer_A_clearInterruptFlag(TIMER_A1_BASE);
    run_control = 1;
}