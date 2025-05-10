/*
 * stm32f407xx_tim_driver.c
 *
 *  Created on: Apr 22, 2025
 *      Author: yunus
 */

#include "stm32f407xx_tim_driver.h"


/* Peripheral clock setup */
void TIM_PeriClockControl(TIM_RegDef_t *pTIMx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		if(pTIMx == TIM6)
			TIM6_PCLK_EN();
		else if(pTIMx == TIM7)
			TIM7_PCLK_EN();

	}else {
		if(pTIMx == TIM6)
			TIM6_PCLK_DI();
		else if(pTIMx == TIM7)
			TIM7_PCLK_DI();
	}
}


/* Init and DeInit */
void TIM_Init(TIM_Handle_t *pTIMHandle) {
	TIM_PeriClockControl(pTIMHandle->pTIMx, ENABLE);

	pTIMHandle->pTIMx->CR1 &= ~( 1 << TIM_CR1_CEN);

	/* Set the timer prescalar value */
	pTIMHandle->pTIMx->PSC = pTIMHandle->TIM_Base_Config.Prescaler;

	/* Set the period value */
	pTIMHandle->pTIMx->ARR = pTIMHandle->TIM_Base_Config.Period;

	/* Enable AutoReload preload register */
	pTIMHandle->pTIMx->CR1 |= ( pTIMHandle->TIM_Base_Config.AutoReloadPreload << TIM_CR1_ARPE );

	/* Configure Update Request Source */
	pTIMHandle->pTIMx->CR1 |= ( pTIMHandle->TIM_Base_Config.UpdateRequestSource << TIM_CR1_URS );

	//Clear UIF
	pTIMHandle->pTIMx->SR &= ~( 1 << TIM_SR_UIF);
}


void TIM_DeInit(TIM_RegDef_t *pTIMx) {
	if(pTIMx == TIM6){
		TIM6_REG_RESET();
	}
	else if(pTIMx == TIM7){
		TIM7_REG_RESET();
	}
}


/* Timer start and stop */
void TIM_Base_Start(TIM_Handle_t *pTIMHandle) {
	pTIMHandle->pTIMx->CR1 |= (1 << TIM_CR1_CEN);
}


void TIM_Base_Stop(TIM_Handle_t *pTIMHandle) {
	pTIMHandle->pTIMx->CR1 &= ~(1 << TIM_CR1_CEN);
}


void TIM_Base_Start_IT(TIM_Handle_t *pTIMHandle) {
	__TIM_INTERRUPT_ENABLE(pTIMHandle);
	__TIM_BASE_START(pTIMHandle);
	pTIMHandle->State = TIM_ON;

}


void TIM_Base_Stop_IT(TIM_Handle_t *pTIMHandle) {
	__TIM_INTERRUPT_DISABLE(pTIMHandle);
	__TIM_BASE_STOP(pTIMHandle);
	pTIMHandle->State = TIM_OFF;
}


void TIM_UpdatePeriod(TIM_Handle_t *pTIMHandle, uint16_t newARRVal) {
	pTIMHandle->TIM_Base_Config.Period = newARRVal--;
	pTIMHandle->pTIMx->CR1 |= ( 1 << TIM_CR1_UDIS );
	pTIMHandle->pTIMx->ARR = pTIMHandle->TIM_Base_Config.Period;
	pTIMHandle->pTIMx->CR1 &= ~( 1 << TIM_CR1_UDIS );

	//Generate Event
	if(pTIMHandle->TIM_Base_Config.UpdateRequestSource == TIM_UPDATE_REQ_ONLY_COUNTER_OFUF) {
		pTIMHandle->TIM_Base_Config.UpdateRequestSource = TIM_UPDATE_REQ_ANY_EVENT;
		pTIMHandle->pTIMx->CR1 &= ~( pTIMHandle->TIM_Base_Config.UpdateRequestSource << TIM_CR1_URS);
	}

	pTIMHandle->pTIMx->EGR |= ( 1 << TIM_EGR_UG);
	pTIMHandle->pTIMx->SR &= ~(TIM_SR_UIF_FLAG);
}


void TIM_UpdatePrescaler(TIM_Handle_t *pTIMHandle, uint16_t newPSCVal) {
	pTIMHandle->TIM_Base_Config.Prescaler = newPSCVal;
	pTIMHandle->pTIMx->CR1 |= ( 1 << TIM_CR1_UDIS );
	pTIMHandle->pTIMx->PSC = pTIMHandle->TIM_Base_Config.Prescaler;
	pTIMHandle->pTIMx->CR1 &= ~( 1 << TIM_CR1_UDIS );

	if(pTIMHandle->TIM_Base_Config.UpdateRequestSource == TIM_UPDATE_REQ_ONLY_COUNTER_OFUF) {
		pTIMHandle->TIM_Base_Config.UpdateRequestSource = TIM_UPDATE_REQ_ANY_EVENT;
		pTIMHandle->pTIMx->CR1 &= ~( pTIMHandle->TIM_Base_Config.UpdateRequestSource << TIM_CR1_URS);
	}

	//Generate Event
	pTIMHandle->pTIMx->EGR |= ( 1 << TIM_EGR_UG);
	pTIMHandle->pTIMx->SR &= ~(TIM_SR_UIF_FLAG);
}


/*
uint16_t TIM_GetCounter(TIM_Handle_t *pTIMHandle) {
	uint16_t Counter = pTIMHandle->pTIMx->CNT;
	return(Counter);
}


//Timer Status Register Flag

uint8_t TIM_GetFlagStatus(TIM_Handle_t *pTIMHandle, uint32_t Flagname) {
	if(pTIMHandle->pTIMx->SR & Flagname) {
		return(FLAG_SET);

	}else {
		return(FLAG_RESET);
	}
}


void TIM_ClearUIFlag(TIM_Handle_t *pTIMHandle) {
	pTIMHandle->pTIMx->SR &= ~(TIM_SR_UIF_FLAG);
}
*/

void TIM_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {

	if(EnorDi == ENABLE) {
		if(IRQNumber <= 31) {
			//Program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber);

		}else if(IRQNumber >= 32 && IRQNumber < 64) {
			//Program ISER1 register
			*NVIC_ISER1 |= ( 1 << ( IRQNumber % 32 ));

		}else if(IRQNumber >= 64 && IRQNumber < 96) {
			//Program ISER2 register
			*NVIC_ISER2 |= ( 1 << ( IRQNumber % 64 ));
		}

	}else {
		if(IRQNumber <= 31) {
			//Program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber);

		}else if(IRQNumber >= 32 && IRQNumber <= 64) {
			//Program ICER1 register
			*NVIC_ICER1 |= ( 1 << ( IRQNumber % 32 ));

		}else if(IRQNumber >= 64 && IRQNumber <= 96) {
			//Program ICER2 register
			*NVIC_ICER2 |= ( 1 << ( IRQNumber % 64 ));
		}
	}
}


void TIM_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	//find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED));
	//*(NVIC_PR_BASEADDR + (iprx * 4)) |= ( IRQPriority << shift_amount); error: pointer addition
	*(NVIC_PR_BASEADDR + iprx) |= ( IRQPriority << shift_amount);
}


void TIM_IRQHandling(TIM_Handle_t *pTIMHandle) {

	uint8_t temp1 = 0;

	temp1 = pTIMHandle->pTIMx->SR & ( TIM_SR_UIF_FLAG );

	if(temp1) {
		__TIM_CLEAR_UIFLAG(pTIMHandle);
		TIM_ApplicationEventCallback(pTIMHandle, UIF_FLAG_RESET);
	}

}


__weak void TIM_ApplicationEventCallback(TIM_Handle_t *pTIMHandle, uint8_t AppEv) {
	//This is a weak implementation. The application may override
	//this function
}


