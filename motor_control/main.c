/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup USART_Printf
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define LED_ON 1
#define I2C_ULTRASONIC_ADDR 0xE0
#define I2C_ADU_ADDR 0x90
#define I2C_SPEED 100000
/* Private variables ---------------------------------------------------------*/
__IO float TIM1Freq = 0;
uint16_t TimerPeriod1 =0; 
uint16_t TimerPeriod2 =0; 
uint8_t readtemp1 = 0;
uint8_t readtemp2 = 0;
__IO uint16_t TimerPrescaler1  = 0;
__IO uint16_t TimerPrescaler2  = 0;
uint16_t ChannelPulse = 0;
char print_str[200] = "";

/*non-extern variable*/
TIM_OCInitTypeDef  TIM2_OCInitStructure;

/* Private function prototypes -----------------------------------------------*/
void LED_Toggle(void);
void GPIO_Configuration(void);
void RCC_Configuration(void);
void USART2_Configuration(void);
void TIM1_Configuration(void);
void TIM2_Configuration(void);
void Usart_SendByte(USART_TypeDef* pUSARTx, uint8_t data);
void Usart_SendStr(USART_TypeDef* pUSARTx, char *str);
void PWMOutput_Configuration(void);
void NVIC_Configuration(void);
void my_Printf(float fVal);
void I2C_Sensor_Configuration(I2C_TypeDef* I2Cx, uint8_t SlaveAddr);
void UltraSonic_Configuration(I2C_TypeDef* I2Cx, uint8_t SlaveAddr);
void ADC_Configuration(I2C_TypeDef* I2Cx, uint8_t SlaveAddr);
void I2C_Read2Byte(I2C_TypeDef* I2Cx, uint8_t SlaveAddr);
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void){
  /* Output a message on Hyperterminal using printf function */
	GPIO_Configuration();
	USART2_Configuration();		
	TIM1_Configuration();
	TIM2_Configuration();	
	PWMOutput_Configuration();
//TODO: Update interrupt
	GPIO_SetBits(GPIOA, GPIO_Pin_5);
	Usart_SendStr(USART2," this is a test\n");
	I2C_Sensor_Configuration(I2C1,I2C_ADU_ADDR);
	ADC_Configuration(I2C1,I2C_ADU_ADDR);
	// UltraSonic_Configuration(I2C1,I2C_ULTRASONIC_ADDR);
	printf("\nhello!every body!\n");
	while (1){

	}
}
void GPIO_Configuration(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
	
	/*disable SMJ and JTAG function to enable PB3 and PB4*/	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	/*
	 *GPIO A
	 */
	/*Timer 3 partial remap: CH1 -> PB4 CH2 ->PB5*/
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); 
	/*Initialize USART_TX at PA2*/
    /* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*Initialize USART_RX at PA3*/
	/* Configure USART Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*LED GPIO configuration at PA5*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	/* TIM1 channel 3 pin (PA10) configuration */
	/*	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	*/
	/*
	 *GPIO B
	 */
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE);
	/* TIM2 channel 2 pin (PB3) configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}


void USART2_Configuration(void){
	/* USARTx configured as follow:
	- BaudRate = 115200 baud  
	- Word Length = 8 Bits
	- One Stop Bit
	- No parity
	- Hardware flow control disabled (RTS and CTS signals)
	- Receive and transmit enabled
	*/
	USART_InitTypeDef USART_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


	/* USART configuration */
	USART_Init(USART2, &USART_InitStructure);
	/* Interrupt */
	NVIC_EnableIRQ(USART2_IRQn);
	/*USART RX interrupt*/
	USART_ITConfig(USART2, USART_IT_RXNE,ENABLE);
	/* Enable USART */
	USART_Cmd(USART2, ENABLE);
}

void TIM2_Configuration(void){
	/* TIM2 configuration */
	/* Period maximum is 2^16-1 */
	TimerPeriod2 = (SystemCoreClock / 17570 ) - 1;
	// TimerPeriod = 60000 - 1;//Prescaler = 40 -1
	// TimerPrescaler = 100 - 1;//Period = 60000 -1
	//TimerPeriod2 = 50000 - 1;//Prescaler = 40 -1
	TimerPrescaler2 = 720*2 - 1;//Period = 60000 -1
	/* Compute CCR1 value to generate a duty cycle at 50% for channel 1*/


	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseStructure.TIM_Prescaler = TimerPrescaler2;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TimerPeriod2;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	
	
}
void TIM1_Configuration(void)

{

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_ICInitTypeDef  TIM_ICInitStructure;

    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	// TimerPeriod1 = 1000 - 1;//Prescaler = 40 -1
	// TimerPrescaler1 = 720*8-1;
	TimerPeriod1 = 6666 - 1;//Prescaler = 40 -1
	TimerPrescaler1 = 3600*5 - 1;//Period = 60000 -1
    TIM_TimeBaseStructure.TIM_Period =TimerPeriod1;
    TIM_TimeBaseStructure.TIM_Prescaler =TimerPrescaler1;   
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseStructure.TIM_RepetitionCounter=0; 

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 


    TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; 
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x00;

    //TIM_PWMIConfig(TIM1,&TIM_ICInitStructure);
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
    TIM_Cmd(TIM1, ENABLE);  
    TIM_ITConfig(TIM1,TIM_IT_CC3,ENABLE);

}
void PWMOutput_Configuration(void){

	
	ChannelPulse = (uint16_t) (((uint32_t) 6 * (TimerPeriod2 + 1)) / 10);
	/* Channel 2 Configuration in PWM output mode */
	TIM2_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM2_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM2_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM2_OCInitStructure.TIM_Pulse = ChannelPulse;
	TIM2_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM2_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM2_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM2_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OC2Init(TIM2, &TIM2_OCInitStructure);

	/* TIM2 counter enable */
	TIM_Cmd(TIM2, ENABLE);

	/* TIM2 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM2, ENABLE);
}
void delay(int millis) {
	while (millis-- > 0) {
		volatile int x = 5971;
		while (x-- > 0) {
			__asm("nop");
		}
	}
}

void PWMOutputDC_Set(float duty_cycle){
	ChannelPulse = (uint16_t) (duty_cycle * (TimerPeriod2 + 1));
	TIM_SetCompare2(TIM2,ChannelPulse);
}
void I2C_Sensor_Configuration(I2C_TypeDef* I2Cx, uint8_t SlaveAddr){
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 1;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);

}
void ADC_Configuration(I2C_TypeDef* I2Cx,uint8_t SlaveAddr){
	
	I2C_GenerateSTART(I2Cx, ENABLE);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){ //EVENT 5
	} 
	
	I2C_Send7bitAddress(I2Cx,SlaveAddr,I2C_Direction_Transmitter);//clear Event 5 flag
	while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
	}
	
	I2C_SendData(I2Cx,0x43);
	while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
	}
	I2C_GenerateSTOP(I2Cx, ENABLE);
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)){
	}
	delay(200);
	printf("adc configured");
}
void UltraSonic_Configuration(I2C_TypeDef* I2Cx,uint8_t SlaveAddr){
	
	I2C_GenerateSTART(I2Cx, ENABLE);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){ //EVENT 5
	} 
	
	I2C_Send7bitAddress(I2Cx,SlaveAddr,I2C_Direction_Transmitter);//clear Event 5 flag
	while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
	}
	
	I2C_SendData(I2Cx,0x00);
	while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
	}
	I2C_SendData(I2Cx,0x51);
	while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
	}
	I2C_GenerateSTOP(I2Cx, ENABLE);
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)){
	}
	delay(200);
	printf("UltraSonic configured");
}
void I2C_Read2Byte(I2C_TypeDef* I2Cx, uint8_t SlaveAddr){
	I2C_GenerateSTART(I2Cx, ENABLE);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){ //EVENT 5
	} 
	/*enable ack bit and POS bit*/
	/*refer to user mannual*/
	I2C_AcknowledgeConfig(I2Cx,ENABLE );
	I2C_NACKPositionConfig(I2C1,I2C_NACKPosition_Next); //set POS. Next byte in shift register will be NACK.
	I2C_Send7bitAddress(I2Cx,SlaveAddr,I2C_Direction_Receiver);//clear Event 5 flag

	/* Event 6 */
	while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
	}
	/*disable ACK bit*/
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	/* wait for receiving data */
	while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF)){
	}
	I2C_GenerateSTOP(I2Cx, ENABLE);
	
	readtemp1 = I2C_ReceiveData(I2Cx);
	readtemp2 = I2C_ReceiveData(I2Cx);
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));	
	printf("ultrasonic data read\n");
}

void LED_Toggle(void){
	uint8_t led_bit = GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_5);
	
	if(led_bit == (uint8_t) LED_ON){
		GPIO_ResetBits(GPIOA,GPIO_Pin_5);
	}
	
	else{
		GPIO_SetBits(GPIOA,GPIO_Pin_5);
	}
	
}


/* send bype via USART */
void Usart_SendByte(USART_TypeDef* pUSARTx, uint8_t data){
    USART_SendData(pUSARTx, data);
    while(USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}
/* send string via USART */
void Usart_SendStr(USART_TypeDef* pUSARTx, char *str){
    uint8_t i = 0;
    do
    {
		Usart_SendByte(pUSARTx, *(str+i));
		i++;
    }while(*(str+i) != '\0');

    while(USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET);
}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
