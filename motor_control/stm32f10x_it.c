/**
  ******************************************************************************
  * @file    TIM/InputCapture/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stdio.h"
/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup TIM_Input_Capture
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint16_t IC3ReadValue1 = 0, IC3ReadValue2 = 0;
__IO uint16_t CaptureNumber = 0;
__IO uint32_t Capture = 0;
__IO uint16_t tim_cnt = 0;
__IO extern float TIM1Freq;
__IO extern uint32_t TimerPrescaler1;
extern uint16_t TimerPeriod1;
extern char print_str[30];
extern uint16_t ChannelPulse;
extern uint8_t readtemp1;
extern uint8_t readtemp2;
uint16_t led_flag = 0;
/* Private function prototypes -----------------------------------------------*/
extern void LED_Toggle(void);
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */	
	USART_SendData(USART2,'a');
	LED_Toggle();	
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{}



// void TIM1_TRG_COM_IRQHandler(void){
	// TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
	// TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	// TIM_ClearITPendingBit(TIM1, TIM_IT_COM);
	// LED_Toggle();

	
	
// }

// void TIM1_UP_IRQHandler(void){
		// //TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
		// TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		// //TIM_ClearITPendingBit(TIM1, TIM_IT_COM);
		// LED_Toggle();
		// if(TIM_GetFlagStatus(TIM1, TIM_FLAG_CC3OF) == SET){
		// Usart_SendStr(USART2,"overflow 3\n");
		// }
	// /*
	// if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) {
		// TIM_ClearITPendingBit(TIM1, TIM_IT_Update);	
		// Usart_SendStr(USART2,"hakuna matata");
	// }
	// */
// }

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/
/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */

void TIM1_CC_IRQHandler(void)
{ 
	//GPIO_ResetBits(GPIOA,GPIO_Pin_5);
	if(TIM_GetITStatus(TIM1, TIM_IT_CC3) == SET) {
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
		//LED_Toggle();
		if(CaptureNumber == 0){
      		IC3ReadValue1 = TIM_GetCapture3(TIM1);
    		CaptureNumber = 1;
    	}
    	else{

      		IC3ReadValue2 = TIM_GetCapture3(TIM1); 
      	
			/* Capture computation */
			if (IC3ReadValue2 >= IC3ReadValue1){
				Capture = (IC3ReadValue2 - IC3ReadValue1); 
			}
			else{
				Capture = ((TimerPeriod1+1 - IC3ReadValue1) + IC3ReadValue2); 
			}
			/* Frequency computation */ 
			CaptureNumber = 0;
		}
	}
}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
*/


void USART2_IRQHandler(void){ 
	float temp = 1.2;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET){
		if((char)USART_ReceiveData(USART2) == 't'){ //TODO:float cannot print
			LED_Toggle();
			//USART_SendData(USART2,'T');
			// TIM1Freq =  (uint16_t) SystemCoreClock /( TimerPrescaler1 +1 ) / Capture; //TODO: to be modified
			TIM1Freq =  2.2/2*10; //TODO: to be modified
			//sprintf(print_str,"Moter Frequency is %f \n",temp);
			sprintf(print_str,"%f \n",10.5);
			Usart_SendStr(USART2,print_str);
		}
		if((char)USART_ReceiveData(USART2) == '1'){
			LED_Toggle();
			//USART_SendData(USART2,'T');
			sprintf(print_str,"Capture %d \n",Capture);
			Usart_SendStr(USART2,print_str);
		}
		if((char)USART_ReceiveData(USART2) == 'c'){
			LED_Toggle();
			//USART_SendData(USART2,'T');
			sprintf(print_str,"Clock Frequency %u \n", SystemCoreClock);
			Usart_SendStr(USART2,print_str);
		}
		if((char)USART_ReceiveData(USART2) == 'v'){
			LED_Toggle();
			//USART_SendData(USART2,'T');
      		IC3ReadValue1 = TIM_GetCapture3(TIM1);
			sprintf(print_str,"Clock Frequency %u\n", IC3ReadValue1);
			Usart_SendStr(USART2,print_str);	
		}
		if((char)USART_ReceiveData(USART2) == 'a'){
			LED_Toggle();
			tim_cnt = TIM_GetCounter(TIM1);
			//USART_SendData(USART2,'T');
			sprintf(print_str,"Timer counter %d \n", tim_cnt);
			Usart_SendStr(USART2,print_str);	
		}		
		if((char)USART_ReceiveData(USART2) == 'n'){
			LED_Toggle();
			sprintf(print_str,"Channel Pulse is %d \n", ChannelPulse);
			Usart_SendStr(USART2,print_str);	
		}
		if((char)USART_ReceiveData(USART2) == '+'){
			LED_Toggle();
			ChannelPulse += 1;
			TIM_SetCompare2(TIM2,ChannelPulse);
			sprintf(print_str,"Channel Pulse updated %d \n", ChannelPulse);
			Usart_SendStr(USART2,print_str);	
		}
		if((char)USART_ReceiveData(USART2) == '-'){
			LED_Toggle();
			ChannelPulse -= 1;
			TIM_SetCompare2(TIM2,ChannelPulse);
			sprintf(print_str,"Channel Pulse updated %d \n", ChannelPulse);
			Usart_SendStr(USART2,print_str);	
		}
		if((char)USART_ReceiveData(USART2) == 's'){
			LED_Toggle();			
			I2C_Read2Byte(I2C1, 0x90);
			printf("distance:1 %d \n",readtemp1);
			printf("distance:2 %d \n",readtemp2);
		}
	USART_ClearFlag(USART2,USART_FLAG_RXNE);
	}
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

