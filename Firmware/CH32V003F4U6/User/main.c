/*****************************************************************
 * File Name          : main.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/10/16
 * Description        : Main file of Aquarium air pump controller
*****************************************************************/

/* Includes */
#include "debug.h"
#include "main.h"
#include "math.h"

/*###########################################################################################################################################################*/
/* Global define */
S_DEVICE sDevice;
S_ANALOG sAnalog;


/* Interrupts defines */
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));


/*###########################################################################################################################################################*/
/* Functions */
void USART1_Init(void);
void ControlPinout_Init(void);
void TIM_INT_Init(void);
void UartSendBuffer(uint8_t* buffer, uint16_t length);
void UART_buffer_clear(void);
void PWM1_Init(uint16_t arr, uint16_t psc, uint16_t pulse);
void PWM1_SetDuty(uint16_t pulse);
int16_t calculate_ntc_temperature(S_ANALOG *values);
uint16_t duty_to_timVal(uint8_t duty);
int8_t calculate_duty(S_ANALOG *values);


/*###########################################################################################################################################################*/
/* Global variables */



/*###########################################################################################################################################################*/

int main(void)
{
    // Device init //
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);         // Init USART
    SystemCoreClockUpdate();                                // Init all the clocks (used 48MHz HSI)
    Delay_Init();                                           // Enable delay
    ControlPinout_Init();                                   // Init the pinout
    TIM_INT_Init();                                         // Init the timer
    USART1_Init();                                          // Init USART
   
    // 1 kHz PWM: 
    //  - prescaler = 48-1 = 47 >> timer clock = 1 MHz, 
    //  - ARR = 1000-1 = 999 = period = 1000 counts = 1 kHz
    PWM1_Init(999, 47, 0);                                  // 0% duty (pulse = 0)
    // e.g.: PWM1_SetDuty(250);                             // 25% duty

    sDevice.bootDone = 0;                                   // Enable boot routine
    sDevice.flag_adc_read = 0;                              // Turn off ADC read
    sDevice.state = devBoot;                                // Set state of main state machine
    sDevice.analog_state = ADCread;                         // Set state of analog state machine
    sDevice.flag_analog_state = 0;                          // Flag used in IRQ to change mode of analog operation

    while(1)
    {
        /* MAIN STATE MACHINE */  
        switch (sDevice.state) 
        {

        // State: Device boot routine
        case devBoot:

            if (sDevice.bootDone == 1) 
            {
                sDevice.state = running;     // End of Boot sequence
                sDevice.analog_state = ADCread;
                break;
            }

            if (sDevice.flag_adc_read == 1)
            {
                sDevice.flag_adc_read = 0;  // Clear flag
                
                /* ANALOG STATE MACHINE */
                switch (sDevice.analog_state)
                {
                // State: Read analog values
                case ADCread:

                    // Read new ADC values and wait on DMA
                    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
                    while (DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);
                    DMA_ClearFlag(DMA1_FLAG_TC1);

                    sAnalog.analogVoltage = sAnalog.adcResults[0];
                    sAnalog.analogTemperature = sAnalog.adcResults[1];
                    sAnalog.analogPotenciometer = sAnalog.adcResults[2];

                    break;

                // State: Compute values
                case compute:

                    // Calculate raw values to messurements and set outputs
                    // --- Input voltage ---
                    sAnalog.voltage = (uint16_t)((float)sAnalog.analogVoltage / (float)(1024 - 1) * BASE_VOLTAGE);

                    // Undervoltage -> error
                    if (sAnalog.voltage < UNDER_VOLT)
                    {
                        sDevice.state = error;
                        break;
                    }

                    // --- Temperature ---
                    sAnalog.temperature = calculate_ntc_temperature(&sAnalog);

                    // Overtemperature -> error
                    if (sAnalog.temperature > OVER_TEMP)
                    {
                        sDevice.state = error;
                        break;
                    }

                    // --- Duty cycle ---
                    uint8_t percent = 0;
                    uint16_t percentSum = 0;
                    
                    // Shift values
                    for (int i = sizeof(sAnalog.potPercent) - 1; i > 0; i--)
                    {
                        sAnalog.potPercent[i] = sAnalog.potPercent[i-1];
                    }

                    // sAnalog.potPercent[0] = (uint8_t)((sAnalog.analogPotenciometer * 100) / 1024);       // PCB Rev.: 0.1
                    sAnalog.potPercent[0] = calculate_duty(&sAnalog);                                       // PCB Rev.: 0.2

                    // Average values
                    for (int i = 0; i < 10; i++)
                    {
                        percentSum += sAnalog.potPercent[i];
                    }

                    percent = (uint8_t)round(percentSum / sizeof(percentSum));

                    // Set PWM duty cycle
                    PWM1_SetDuty(duty_to_timVal(percent));

                    break;
                }
            }
            break;

        // State: Device is running normaly
        case running:
        
            if (sDevice.flag_adc_read == 1)
            {
                sDevice.flag_adc_read = 0;  // Clear flag
                
                /* ANALOG STATE MACHINE */
                switch (sDevice.analog_state)
                {
                // State: Read analog values
                case ADCread:

                    // Read new ADC values and wait on DMA
                    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
                    while (DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);
                    DMA_ClearFlag(DMA1_FLAG_TC1);

                    sAnalog.analogVoltage = sAnalog.adcResults[0];
                    sAnalog.analogTemperature = sAnalog.adcResults[1];
                    sAnalog.analogPotenciometer = sAnalog.adcResults[2];

                    break;

                // State: Compute values
                case compute:

                    // Calculate raw values to messurements and set outputs
                    // --- Input voltage ---
                    sAnalog.voltage = (uint16_t)((float)sAnalog.analogVoltage / (float)(1024 - 1) * BASE_VOLTAGE);

                    // Undervoltage -> error
                    if (sAnalog.voltage < UNDER_VOLT)
                    {
                        sDevice.state = error;
                        break;
                    }

                    // --- Temperature ---
                    sAnalog.temperature = calculate_ntc_temperature(&sAnalog);

                    // Overtemperature -> error
                    if (sAnalog.temperature > OVER_TEMP)
                    {
                        sDevice.state = error;
                        break;
                    }

                    // --- Duty cycle ---
                    uint8_t percent = 0;
                    uint16_t percentSum = 0;
                    
                    // Shift values
                    for (int i = sizeof(sAnalog.potPercent) - 1; i > 0; i--)
                    {
                        sAnalog.potPercent[i] = sAnalog.potPercent[i-1];
                    }
                    sAnalog.potPercent[0] = (uint8_t)((sAnalog.analogPotenciometer * 100) / 1024);

                    // Average values
                    for (int i = 0; i < 10; i++)
                    {
                        percentSum += sAnalog.potPercent[i];
                    }

                    percent = (uint8_t)round(percentSum / sizeof(percentSum));

                    // Set PWM duty cycle
                    PWM1_SetDuty(duty_to_timVal(percent));

                    break;
                }
            }

            break;

        // State: Error state
        case error:
                // Do nothing
                GPIO_WriteBit(GPIOC, LED_RED, Bit_SET);   // Turn ON led at error
            break;
        }
    }
}


/*###########################################################################################################################################################*/
/* Initialization */


/*********************************************************************
 * @fcn     GPIO_Init
 *
 * @brief   Initialises the GPIOs
 *
 * @return  none
 */
void ControlPinout_Init(void)
{
    ADC_InitTypeDef     ADC_InitStructure = {0};
    GPIO_InitTypeDef    GPIO_InitStructure = {0};
    DMA_InitTypeDef     DMA_InitStruct  = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO | RCC_APB2Periph_ADC1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // Set RED LED
    GPIO_InitStructure.GPIO_Pin = LED_RED;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Set AN_TEMP, AN_POT and AN_V_IN
    GPIO_InitStructure.GPIO_Pin = AN_TEMP | AN_POT;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = AN_V_IN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 3;
    ADC_Init(ADC1, &ADC_InitStructure);

    // Configure channel rank
    ADC_RegularChannelConfig(ADC1, ADC_CH_V_IN,  1, ADC_SampleTime_241Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_CH_TEMP, 2, ADC_SampleTime_241Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_CH_POT,  3, ADC_SampleTime_241Cycles);

    // DMA configuration
    DMA_DeInit(DMA1_Channel1);                                      // ADC1
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->RDATAR;
    DMA_InitStruct.DMA_MemoryBaseAddr     = (uint32_t)sAnalog.adcResults;
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralSRC;  // peripheral to memory
    DMA_InitStruct.DMA_BufferSize         = 3;
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  // 16-bit
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Circular;          
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;
    DMA_InitStruct.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStruct);

    DMA_Cmd(DMA1_Channel1, ENABLE);

    // Enable ADC DMA
    ADC_DMACmd(ADC1, ENABLE);

    // Enable ADC
    ADC_Cmd(ADC1, ENABLE);

    // Calibrate ADC
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

}



/*********************************************************************
 * @fcn     TIM_INT_Init
 *
 * @brief   Initialises Timer2: 50Hz / 20ms
 *
 * @return  none
 */
void TIM_INT_Init(void)
{
    // TIM1 - Advanced-control Timer (ADTM)
    // TIM2 - General-purpose Timer (GPTM)

    TIM_TimeBaseInitTypeDef TIMBase_InitStruct = {0};
    NVIC_InitTypeDef NVIC_InitStruct = {0};

    // Set values of registers
    uint16_t arr = 59999;
    uint16_t psc = 15; 

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIMBase_InitStruct.TIM_Period = arr;
    TIMBase_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIMBase_InitStruct.TIM_Prescaler = psc;
    TIMBase_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIMBase_InitStruct);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStruct);
    #ifdef DEBUG
        Delay_Ms(50);
    #endif
    TIM_Cmd(TIM2, ENABLE);
}



/*********************************************************************
 * @fcn     PWM1_Init
 *
 * @brief   Initialize Timer1 PWM
 *
 * @param   arr      ！ auto-reload value (period)
 * @param   psc      ！ prescaler
 * @param   pulse    ！ initial pulse width (CCR)
 *
 * @return  none
 */
void PWM1_Init(uint16_t arr, uint16_t psc, uint16_t pulse)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_TIM1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = LS_SW;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = pulse;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    // Use Timer1
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    // Enable preload and PWM output enable
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    // For TIM1 you need to enable main outputs
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    // Start timer
    TIM_Cmd(TIM1, ENABLE);
}



/*********************************************************************
 * @fcn      PWM1_SetDuty
 *
 * @brief   Change duty (pulse width) for LS_SW channel
 *          In tight corelation with timer frequency
 *
 * @param   pulse ！ new pulse value (0 .. arr)
 */
void PWM1_SetDuty(uint16_t pulse)
{
    TIM_SetCompare1(TIM1, pulse);
}



/*********************************************************************
 * @fcn      duty_to_timVal
 *
 * @brief   Calculate PWM timer value from percent
 *
 * @param   timer value
 */
uint16_t duty_to_timVal(uint8_t duty)
{
    uint16_t currentFreq = 1000;

    // Area of unvalid dutys
    if (duty < MIN_DUTY && duty != 0)
    {
        duty = 0;
    }
    else if (duty > MAX_DUTY && duty != 100)
    {
        duty = 100;
    }

    // Calculate timer value
    uint16_t value = (duty * currentFreq) / 100;

    if (value >= 1000) value = 999;
    if (value < 0) value = 0;

    return value;
}



/*********************************************************************
 * @fcn      USART1_Init
 *
 * @brief   Initializes the USART1 peripheral
 *
 * @return  none
 */
void USART1_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

    // Set pin USART TX
    GPIO_InitStructure.GPIO_Pin = USART_RX;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Set pin: USART RX
    GPIO_InitStructure.GPIO_Pin = USART_TX;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // USART1 setup
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    // USART1 IRQ - RX
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);

}



/*********************************************************************
 * @fcn     USART1_IRQHandler
 *
 * @brief   Receive data over USART - USB comunication and saves
 *          them to the array in struct. Once the entire string is
 *          received (end marked with \n) the flag is set. This
 *          indicates that data can be decoded.
 *          Used only for parameters, setup...
 *
 * @return  none
 */
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        // TODO: PWM value over UART and other commands

        // Clear RX flag (otherwise constantly triggered IRQ)
        USART_ClearITPendingBit(USART1, USART_IT_RXNE); 
    }
}



/*********************************************************************
 * @fn      TIM2_IRQHandler
 *
 * @brief   Timer interrupt - 50Hz/20ms
 *
 * @return  none
 */
void TIM2_IRQHandler(void)
{
    static uint8_t cntBlink = 0;
    static uint8_t cntBoot = 0;

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        // Analog read channel switch //
        if (sDevice.state == devBoot || sDevice.state == running)
        {
            if (sDevice.flag_analog_state == 1)                     // Reset state/channel of ADC  
            {
                sDevice.flag_analog_state = 0;
                sDevice.analog_state = compute;                     // Change state/channel of ADC

                if (sDevice.state == devBoot)                       // Count cycles at boot
                {
                    cntBoot++;
                    if (cntBoot > 9) sDevice.bootDone = 1;          // End boot process
                } 
            } 
            else
            {
                sDevice.analog_state = ADCread;                     // Change state/channel of ADC
                sDevice.flag_analog_state = 1;
            }
            sDevice.flag_adc_read = 1;                              // Trigger ADC reading and computing
        }

        // Boot led //
        if (sDevice.bootDone == 0)
        {
            cntBlink++;                                             // Timer counters

            // Blink LED (twice per second)
            if (cntBlink >= 6)
            {
                GPIO_WriteBit(GPIOC, LED_RED, (BitAction)!GPIO_ReadOutputDataBit(GPIOC, LED_RED));
                cntBlink = 0;
            }
        }
        else
        {
            GPIO_WriteBit(GPIOC, LED_RED, Bit_RESET);               // Turn OFF led after boot
        }

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);                 // Clear flag            
    }
}



/*********************************************************************
 * @fn      calculate_ntc_temperature
 *
 * @brief   Simplified ADC to temp calculation
 *
 * @return  temperature
 */
int16_t calculate_ntc_temperature(S_ANALOG *values) {
    
    uint16_t adc_value = values->analogTemperature;
    
    if (adc_value == 0 || adc_value >= 1023) return -100;
    
    // R_ntc = R2 * (ADC_max - ADC_value) / ADC_value
    uint32_t r_ntc = (uint32_t)10000 * (1023 - adc_value) / adc_value;
    
    // Lookup table
    const struct {
        int16_t temp;           // Temperature column
        uint16_t resistance;    // Resistance column
    } ntc_table[] = {
        { -10, 56800 },
        {   0, 33180 },
        {  10, 20070 },
        {  15, 15790 },
        {  20, 15000 },
        {  25, 10000 },  // Reference
        {  30,  8040 },
        {  35,  6510 },
        {  40,  5300 },
        {  45,  4340 },
        {  50,  3580 },
        {  55,  2960 },
        {  60,  2470 },
        {  65,  2070 },
        {  70,  1740 },
        {  75,  1470 },
        {  80,  1250 },
        {  85,  1060 },
        {  90,   910 },
        {  95,   780 },
        {  100,  670 },
        {  110,  510 },
    };
    
    // Find where our resistance fits
    for (int i = 0; i < sizeof(ntc_table)/sizeof(ntc_table[0]) - 1; i++) {
        if (r_ntc <= ntc_table[i].resistance && r_ntc >= ntc_table[i+1].resistance) {
            // Linear interpolation
            uint16_t r1 = ntc_table[i].resistance;
            uint16_t r2 = ntc_table[i+1].resistance;
            int16_t t1 = ntc_table[i].temp;
            int16_t t2 = ntc_table[i+1].temp;
            
            return t1 + ((int32_t)(r1 - r_ntc) * (t2 - t1)) / (r1 - r2);
        }
    }
    
    return -100; // Out of range
}


/*********************************************************************
 * @fn      calculate_duty
 *
 * @brief   Calculation of a duty cycle percent biased to 5V
 *
 * @return  percent for duty cycle
 */
int8_t calculate_duty(S_ANALOG *values) {

    uint8_t percentRaw = (uint8_t)((values->analogPotenciometer * 100) / 1023);                     // Calculate percent from V_pot when ADC V_ref is V_in
    uint8_t delta = (uint8_t)(((DUTY_VREF * percentRaw) / (values->voltage / 100))) - percentRaw;   // Calculate percent with 5V bias 

    return percentRaw + delta;
}


