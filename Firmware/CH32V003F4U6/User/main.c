/*****************************************************************
 * File Name          : main.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/02/11
 * Description        : Main file of Aquarium air pump controller
*****************************************************************/

/* Includes */
#include "debug.h"
#include "main.h"


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
uint16_t ADC_ReadChannel(uint8_t channel);
void PWM1_Init(uint16_t arr, uint16_t psc, uint16_t pulse);
void PWM1_SetDuty(uint16_t pulse);

/*###########################################################################################################################################################*/
/* Global define */
S_DEVICE sDevice;
S_ANALOG sAnalog;



int main(void)
{
    // Device init //
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);         // Init USART
    SystemCoreClockUpdate();                                // Init all the clocks (used 48MHz HSI)
    Delay_Init();                                           // Enable delay
    ControlPinout_Init();                                   // Init the pinout
    TIM_INT_Init();                                         // Init the timer
    USART1_Init();                                          // Init USART
    // 1 kHz PWM: prescaler = 48-1 = 47 >> timer clock = 1 MHz, ARR = 1000-1 = 999 = period = 1000 counts = 1 kHz
    PWM1_Init(999, 47, 0);                                  // 0% duty (pulse = 0)
    // e.g.: PWM1_SetDuty(250);                                   // 5% duty

    sDevice.bootDone = 0;                                   // Enable boot routine
    sDevice.flag_adc_read = 0;                              // Turn off ADC read
    sDevice.state = devBoot;                                // Set state of main state machine
    sDevice.analog_state = readVoltage;                     // Set state of analog state machine

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
                break;
            }

            if (sDevice.flag_adc_read == 1)
            {
                sDevice.flag_adc_read = 0;  // Clear flag
                
                /* ANALOG STATE MACHINE */
                switch (sDevice.analog_state)
                {
                // State: Read input voltage
                case readVoltage:
                    sAnalog.analogVoltage = ADC_ReadChannel(ADC_CH_V_IN);
                    break;

                // State: Read temperature
                case readTemperature:
                    sAnalog.analogTemperature = ADC_ReadChannel(ADC_CH_TEMP);
                    break;

                // State: Read potenciometer for duty cycle adjustment
                case readPotenciometer:
                    sAnalog.analogPotenciometer = ADC_ReadChannel(ADC_CH_POT);
                    break;

                // State: Compute values
                case compute:
                    // TODO: Add your computation and error
                    


                    PWM1_SetDuty(sDevice.duty);     // Set PWM duty cycle
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
                // State: Read input voltage
                case readVoltage:
                    sAnalog.analogVoltage = ADC_ReadChannel(ADC_CH_V_IN);
                    break;

                // State: Read temperature
                case readTemperature:
                    sAnalog.analogTemperature = ADC_ReadChannel(ADC_CH_TEMP);
                    break;

                // State: Read potenciometer for duty cycle adjustment
                case readPotenciometer:
                    sAnalog.analogPotenciometer = ADC_ReadChannel(ADC_CH_POT);
                    break;

                // State: Compute values
                case compute:
                    // TODO: Add your computation and error


                    PWM1_SetDuty(sDevice.duty);     // Set PWM duty cycle

                    break;
                }
            }

            break;

        // State: Error state
        case error:
                // Do nothing
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
     ADC_InitTypeDef ADC_InitStructure = {0};
    GPIO_InitTypeDef  GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO | RCC_APB2Periph_ADC1, ENABLE);

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
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    /* Enable ADC */
    ADC_Cmd(ADC1, ENABLE);

    /* Calibrate ADC */
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

}



/*********************************************************************
 * @fcn     ADC_ReadChannel
 *
 * @brief   Reads one ADC channel and returns 12-bit value
 *
 * @param   channel - ADC channel number (e.g. ADC_Channel_1)
 * @return  12-bit ADC result
 *********************************************************************/
uint16_t ADC_ReadChannel(uint8_t channel)
{
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_241Cycles);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    return ADC_GetConversionValue(ADC1);
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
 *
 * @param   pulse ！ new pulse value (0 .. arr)
 */
void PWM1_SetDuty(uint16_t pulse)
{
    TIM_SetCompare1(TIM1, pulse);
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
        // TODO: PWM value over UART

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
            sDevice.analog_state++;                                 // Change state/channel of ADC
            sDevice.flag_adc_read = 1;                              // Trigger ADC reading

            if (sDevice.analog_state > 3)                           // Reset state/channel of ADC  
            {
                sDevice.analog_state = 0;

                if (sDevice.state == devBoot)                       // Count cycles at boot
                {
                    cntBoot++;
                    if (cntBoot > 9) sDevice.bootDone = 1;          // End boot process
                } 
            } 
        }

        // Boot led //
        if (sDevice.bootDone != 1)
        {
            cntBlink++;                                             // Timer counters

            // Blink LED (twice per second)
            if (cntBlink >= 12)
            {
                GPIO_WriteBit(GPIOC, LED_RED, (BitAction)!GPIO_ReadOutputDataBit(GPIOC, LED_RED));
                cntBlink = 0;
            }
        }
        else
        {
            GPIO_WriteBit(GPIOC, LED_RED, Bit_RESET);   // Turn OFF led after boot
        }

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);   // Clear flag            
    }
}