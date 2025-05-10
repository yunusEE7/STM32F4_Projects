/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: Dec 4, 2024
 *      Author: yunus
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"


/* Bit position definitions RCC_CR */
#define RCC_CR_HSEON				16
#define RCC_CR_HSERDY				17
#define RCC_CR_PLLON				24
#define RCC_CR_PLLRDY				25
#define RCC_CR_HSION				0

/* Bit position definitions RCC_PLLCFGR */
#define RCC_PLLCFGR_PLLSRC			22
#define RCC_PLLCFGR_PLLM			5
#define RCC_PLLCFGR_PLLN			14
#define RCC_PLLCFGR_PLLP			17
#define RCC_PLLCFGR_PLLQ			27

/* Bit position definitions RCC_CFGR */
#define RCC_CFGR_HPRE				7
#define RCC_CFGR_PPRE1				12
#define RCC_CFGR_PPRE2				15
#define RCC_CFGR_SW					1

/* Bit position definitions RCC_AHB2ENR */
#define RCC_AHB2ENR_OTGFSEN			7



//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);

uint32_t RCC_GetPLLOutputClock(void);


#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
