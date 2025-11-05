/***************************************************************************
 * File Name          : main.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/10/16
 * Description        : Main header file of Aquarium air pump controller
***************************************************************************/

#ifndef USER_MAIN_H_
#define USER_MAIN_H_

/* Includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/*###########################################################################################################################################################*/
/* Defines */

// Device constants
#define DEVICE_NAME    "Pump control"   // Name of the device
#define SW_VER              1           // Software version (used in UART packets)
#define SW_DATE        "October 2025"   // Date of software

#define OVER_TEMP           80          // Over temperature protection
#define BASE_VOLTAGE        500         // Expected voltage on the input
#define UNDER_VOLT          350         // Under voltage protection (3.5V)
#define MIN_DUTY            10          // Minimum value of duty cycle (lower is 0)
#define MAX_DUTY            90          // Maximum value of duty cycle (higher is 100)
#define V_IN_R1             10000       // Voltage divider (input voltage) R1 
#define V_IN_R2             100000      // Voltage divider (input voltage) R2 
#define DUTY_VREF           5           // Duty cycle ideal reference voltage
//#define DUTY_POT          20000       // Potenciometer (duty cycle), PCB Rev.: 0.1
#define DUTY_POT            50000       // Potenciometer (duty cycle), PCB Rev.: 0.2
#define DUTY_R              2000        // Resistor R2 (duty cycle), PCB Rev.: 0.2
#define TEMP_BETA           39500       // Voltage divider (temperature) NTC Beta value
#define TEMP_R0             10000       // R_0 value of NTC
#define TEMP_R              10000       // Voltage divider (temperature) R2 
#define TEMP_T_0            29815       // T_0 - reference temperature 25C / 298.15K

// Pinout
#define LED_RED             GPIO_Pin_3  // PC3: Pin of red LED
#define LS_SW               GPIO_Pin_2  // PD2: Low side switching mosfet
#define USART_TX            GPIO_Pin_5  // PD5: UART TX
#define USART_RX            GPIO_Pin_6  // PD6: UART RX
#define AN_V_IN             GPIO_Pin_4  // PC4: Analog read - USB voltage
#define ADC_CH_V_IN         ADC_Channel_2   // ADC Channel of PC4
#define AN_TEMP             GPIO_Pin_1  // PA1: Analog read - Temperature
#define ADC_CH_TEMP         ADC_Channel_1   // ADC Channel of PA1
#define AN_POT              GPIO_Pin_2  // PA2: Analog read - Potenciomeret voltage
#define ADC_CH_POT          ADC_Channel_0   // ADC Channel of PA2

/*###########################################################################################################################################################*/
/* Structs - marked with s at the beginning*/

typedef struct {
    uint8_t     bootDone;               // Flag: signal end of device initialization
    uint8_t     state;                  // Main state machine variable   
    uint8_t     analog_state;           // Analog read state machine variable  
    uint8_t     flag_analog_state;      // Flag used in IRQ for state monitoring
    uint8_t     flag_adc_read;          // Flag: read ADC value from current channel
    uint8_t     flag_overtemp;          // Flag: overtemperature protection
    uint8_t     flag_undervoltage;      // Flag: undervoltage protection
    uint16_t    duty;                   // Value if current duty cycle
} S_DEVICE;

typedef struct {            
    uint16_t    adcResults[3];          // buffer for ADC DMA results
    uint16_t    analogVoltage;          // Raw analog value of input voltage (10bit)
    uint16_t    analogTemperature;      // Raw analog value of temperature (10bit)
    uint16_t    analogPotenciometer;    // Raw analog value of potenciometer (10bit)
    uint16_t    voltage;                // Current value of input voltage
    uint16_t    temperature;            // Current value of temprature
    uint16_t    potVal;                 // Current value of potenciometer
    uint16_t    potPercent[10];         // Array of measurements from potenciometers
} S_ANALOG;


// Enumerate of states used in main state machine
enum eMainStates {
    devBoot,
    running,
    error
};

// Enumerate of states used in analog read state machine
enum eAnalogStates {
    ADCread,
    compute
};


#endif /* USER_MAIN_H_ */