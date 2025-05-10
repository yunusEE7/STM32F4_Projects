/*
 * stm32f407xx_tim_driver.h
 *
 *  Created on: Apr 24, 2025
 *      Author: yunus
 */

#ifndef INC_STM32F407XX_TIM_DRIVER_H_
#define INC_STM32F407XX_TIM_DRIVER_H_


#include "stm32f407xx.h"


/* This is a configuration structure for a GPIO pin */
typedef struct {
	uint16_t Prescaler;
	uint16_t Period;
	uint8_t AutoReloadPreload;
	uint8_t OnePulseMode;
	uint8_t UpdateRequestSource;
}TIM_Base_Config_t;


/* This is a Handle structure for a GPIO pin */
typedef struct {
	TIM_RegDef_t *pTIMx;
	TIM_Base_Config_t TIM_Base_Config;
	uint8_t State;
}TIM_Handle_t;


/*
 * Possilble TIMER Application States
 */
#define TIM_ON				0
#define TIM_OFF				1


/*
 * Possilble TIMER Application Events
 */
#define UIF_FLAG_RESET			0
#define UIF_FLAG_SET			1


/*
 * @TIM_AutoReloadPreload
 */
#define TIM_AUTORELOAD_PRELOAD_DISABLE			0		/*!< TIMx_ARR register is not buffered */
#define TIM_AUTORELOAD_PRELOAD_ENABLE			1


/*
 * @TIM_OnePulseMode
 */
#define TIM_ONEPULSEMODE_DISABLE				0
#define TIM_ONEPULSEMODE_ENABLE					1


/*
 * @UpdateRequestSource
 */
#define TIM_UPDATE_REQ_ANY_EVENT				0
#define TIM_UPDATE_REQ_ONLY_COUNTER_OFUF		1



/*
 * TIMER related status flags definitions
 */
#define TIM_SR_UIF_FLAG			( 1 << TIM_SR_UIF)



/************************************************************************************
 * 							APIs supported by this driver
 * 		For more information about the APIs, check the function definitions
 ************************************************************************************/

/* Peripheral clock setup */
void TIM_PeriClockControl(TIM_RegDef_t *pTIMx, uint8_t EnorDi);

/* Init and DeInit */
void TIM_Init(TIM_Handle_t *pTIMHandle);
void TIM_DeInit(TIM_RegDef_t *pTIMx);

/* Timer start and stop */
void TIM_Base_Start(TIM_Handle_t *pTIMHandle);
void TIM_Base_Stop(TIM_Handle_t *pTIMHandle);
#define __TIM_BASE_START(htim)			( ( (htim)->pTIMx->CR1 ) |= (1 << TIM_CR1_CEN) )
#define __TIM_BASE_STOP(htim)			( ( (htim)->pTIMx->CR1 ) &= ~(1 << TIM_CR1_CEN) )

void TIM_Base_Start_IT(TIM_Handle_t *pTIMHandle);
void TIM_Base_Stop_IT(TIM_Handle_t *pTIMHandle);


void TIM_UpdatePeriod(TIM_Handle_t *pTIMHandle, uint16_t newARRVal);
void TIM_UpdatePrescaler(TIM_Handle_t *pTIMHandle, uint16_t newPSCVal);

uint8_t TIM_GetFlagStatus(TIM_Handle_t *pTIMHandle, uint32_t Flagname);
/*
uint16_t TIM_GetCounter(TIM_Handle_t *pTIMHandle);
void TIM_ClearUIFlag(TIM_Handle_t *pTIMHandle);
*/
#define __TIM_GET_COUNTER(htim)			( ( (htim)->pTIMx->CNT ) )
#define __TIM_CLEAR_UIFLAG(htim)		( ( (htim)->pTIMx->SR ) &= ~(TIM_SR_UIF_FLAG) )

/* IRQ configuration and ISR handling */
void TIM_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void TIM_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void TIM_IRQHandling(TIM_Handle_t *pTIMHandle);
#define __TIM_INTERRUPT_ENABLE(htim)	( ( (htim)->pTIMx->DIER) |= ( 1 << TIM_DIER_UIE) )
#define __TIM_INTERRUPT_DISABLE(htim)	( ( (htim)->pTIMx->DIER) &= ~( 1 << TIM_DIER_UIE) )

void TIM_ApplicationEventCallback(TIM_Handle_t *pTIMHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_TIM_DRIVER_H_ */
