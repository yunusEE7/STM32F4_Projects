/*
 * stm32f407xx.h
 *
 *  Created on: Apr 7, 2024
 *      Author: yunus
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_


#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak	__attribute__((weak))
//#define NULL ((void *)0)


/*********************START: Processor specific Details*********************/
/*
 * ARM Cortex Mx processor NVIC ISERx register addresses
 */

#define NVIC_ISER0		((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1		((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2		((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3		((__vo uint32_t*)0xE000E10C)


/*
 * ARM Cortex Mx processor NVIC ISERx register addresses
 */
#define NVIC_ICER0		((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1		((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2		((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3		((__vo uint32_t*)0xE000E18C)

/*
 * ARM Cortex Mx processor priority register address calculation
 */
#define NVIC_PR_BASEADDR		((__vo uint32_t*)0xE000E400)


/*
 * ARM Cortex Mx processor number of priority bits implemented in Priority register
 */
#define NO_PR_BITS_IMPLEMENTED			4


/**********************END: Processor specific Retails**********************/


/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR						0x08000000U		/*512KB*/
#define SRAM1_BASEADDR						0x20000000U		/*112KB*/
#define SRAM2_BASEADDR						0x2001C000U		/*16KB*/
#define ROM_BASEADDR						0x1FFF0000U		/*30KB*/
#define SRAM								SRAM1_BASEADDR	/*Main SRAM*/


/*
 * Base addresses of bus domains
 */

#define PERIPH_BASE							0x40000000U
#define APB1PERIPH_BASE						PERIPH_BASE
#define APB2PERIPH_BASE						0x40010000U
#define AHB1PERIPH_BASE						0x40020000U
#define AHB2PERIPH_BASE						0x50000000U


/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO : Complete for all other peripherals
 */
#define GPIOA_BASEADDR						( ( AHB1PERIPH_BASE ) + ( 0x0000U ) )
#define GPIOB_BASEADDR						( ( AHB1PERIPH_BASE ) + ( 0x0400U ) )
#define GPIOC_BASEADDR						( ( AHB1PERIPH_BASE ) + ( 0x0800U ) )
#define GPIOD_BASEADDR						( ( AHB1PERIPH_BASE ) + ( 0x0C00U ) )
#define GPIOE_BASEADDR						( ( AHB1PERIPH_BASE ) + ( 0x1000U ) )
#define GPIOF_BASEADDR						( ( AHB1PERIPH_BASE ) + ( 0x1400U ) )
#define GPIOG_BASEADDR						( ( AHB1PERIPH_BASE ) + ( 0x1800U ) )
#define GPIOH_BASEADDR						( ( AHB1PERIPH_BASE ) + ( 0x1C00U ) )
#define GPIOI_BASEADDR						( ( AHB1PERIPH_BASE ) + ( 0x2000U ) )

#define RCC_BASEADDR						( ( AHB1PERIPH_BASE ) + ( 0x3800U ) )


/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 *
 * ----------To be added later on----------
 *
 */


/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR						( ( APB1PERIPH_BASE ) + ( 0x5400U ) )
#define I2C2_BASEADDR						( ( APB1PERIPH_BASE ) + ( 0x5800U ) )
#define I2C3_BASEADDR						( ( APB1PERIPH_BASE ) + ( 0x5C00U ) )

#define SPI2_I2S_BASEADDR					( ( APB1PERIPH_BASE ) + ( 0x3800U ) )
#define SPI3_I2S_BASEADDR					( ( APB1PERIPH_BASE ) + ( 0x3C00U ) )

#define USART2_BASEADDR						( ( APB1PERIPH_BASE ) + ( 0x4400U ) )
#define USART3_BASEADDR						( ( APB1PERIPH_BASE ) + ( 0x4800U ) )
#define UART4_BASEADDR						( ( APB1PERIPH_BASE ) + ( 0x4C00U ) )
#define UART5_BASEADDR						( ( APB1PERIPH_BASE ) + ( 0x5000U ) )

#define TIM6_BASEADDR						( ( APB1PERIPH_BASE ) + ( 0x1000U ) )
#define TIM7_BASEADDR						( ( APB1PERIPH_BASE ) + ( 0x1400U ) )

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define EXTI_BASEADDR						( ( APB2PERIPH_BASE ) + ( 0x3C00U ) )
#define SPI1_BASEADDR						( ( APB2PERIPH_BASE ) + ( 0x3000U ) )
#define SYSCFG_BASEADDR						( ( APB2PERIPH_BASE ) + ( 0x3800U ) )
#define USART1_BASEADDR						( ( APB2PERIPH_BASE ) + ( 0x1000U ) )
#define USART6_BASEADDR						( ( APB2PERIPH_BASE ) + ( 0x1400U ) )


/******************** Peripheral register definition structures *******************/

/*
 * Note : Registers of a peripheral are specific to MCU
 */

__vo typedef struct
{
	uint32_t MODER;			/*	GPIO port mode register,					Address offset: 0x00	*/
	uint32_t OTYPER;		/*	GPIO port output type register,				Address offset: 0x04	*/
	uint32_t OSPEEDR;		/*	GPIO port output speed register,			Address offset: 0x08	*/
	uint32_t PUPDR;			/*	GPIO port pull-up/pull-down register,		Address offset: 0x0C	*/
	uint32_t IDR;			/*	GPIO port input data register,				Address offset: 0x10	*/
	uint32_t ODR;			/*	GPIO port output data register,				Address offset: 0x14	*/
	uint32_t BSRR;			/*	GPIO port bit set/reset register,			Address offset: 0x18	*/
	uint32_t LCKR;			/*	GPIO port configuration lock register,		Address offset: 0x1C	*/
	uint32_t AFR[2];		/*	GPIO alternate function registers,			Address offset: 0x20	*/
}GPIO_RegDef_t;


__vo typedef struct
{
	uint32_t CR;			/*	clock control register,										Address offset: 0x00	*/
	uint32_t PLLCFGR;		/*	PLL configuration register,									Address offset: 0x04	*/
	uint32_t CFGR;			/*	clock configuration register,								Address offset: 0x08	*/
	uint32_t CIR;			/*	clock interrupt register,									Address offset: 0x0C	*/
	uint32_t AHB1RSTR;		/*	AHB1 peripheral reset register,								Address offset: 0x10	*/
	uint32_t AHB2RSTR;		/*	AHB2 peripheral reset register,								Address offset: 0x14	*/
	uint32_t AHB3RSTR;		/*	AHB3 peripheral reset register,								Address offset: 0x18	*/

	uint32_t Reserved0;		/*	Reserved: 0x1C	*/

	uint32_t APB1RSTR;		/*	APB1 peripheral reset register,								Address offset: 0x20	*/
	uint32_t APB2RSTR;		/*	APB2 peripheral reset register,								Address offset: 0x24	*/

	uint32_t Reserved1[2];	/*	Reserved: 0x28 - 0x2C	*/

	uint32_t AHB1ENR;		/*	AHB1 peripheral clock enable register,						Address offset: 0x30	*/
	uint32_t AHB2ENR;		/*	AHB2 peripheral clock enable register,						Address offset: 0x34	*/
	uint32_t AHB3ENR;		/*	AHB3 peripheral clock enable register,						Address offset: 0x38	*/

	uint32_t Reserved2;		/*	Reserved: 0x3C			*/

	uint32_t APB1ENR;		/*	APB1 peripheral clock enable register,						Address offset: 0x40	*/
	uint32_t APB2ENR;		/*	APB2 peripheral clock enable register,						Address offset: 0x44	*/

	uint32_t Reserved3[2];	/*	Reserved: 0x48 - 0x4C	*/

	uint32_t AHB1LPENR;		/*	AHB1 peripheral clock enable in low power mode register,	Address offset: 0x50	*/
	uint32_t AHB2LPENR;		/*	AHB2 peripheral clock enable in low power mode register,	Address offset: 0x54	*/
	uint32_t AHB3LPENR;		/*	AHB3 peripheral clock enable in low power mode register,	Address offset: 0x58	*/

	uint32_t Reserved4;		/*	Reserved: 0x5C			*/

	uint32_t APB1LPENR;		/*	APB1 peripheral clock enable in low power mode register,	Address offset: 0x60	*/
	uint32_t APB2LPENR;		/*	APB2 peripheral clock enable in low power mode register,	Address offset: 0x64	*/

	uint32_t Reserved5[2];	/*	Reserved: 0x68 - 0x6C	*/

	uint32_t BDCR;			/*	Backup domain control register,								Address offset: 0x70	*/
	uint32_t CSR;			/*	clock control & status register,							Address offset: 0x74	*/

	uint32_t Reserved6[2];	/*	Reserved: 0x78 - 0x7C	*/

	uint32_t SSCGR;			/*	spread spectrum clock generation register,					Address offset: 0x80	*/
	uint32_t PLLI2SCFGR;	/*	PLLI2S configuration register,								Address offset: 0x84	*/
}RCC_RegDef_t;


/* Peripheral register definition structure for EXTI */
__vo typedef struct {
	uint32_t IMR;			/*	Interrupt mask register,						Address offset: 0x00	*/
	uint32_t EMR;			/*	Event mask register,							Address offset: 0x04	*/
	uint32_t RTSR;			/*	Rising trigger selection register,				Address offset: 0x08	*/
	uint32_t FTSR;			/*	Falling trigger selection register,				Address offset: 0x0C	*/
	uint32_t SWIER;			/*	Software interrupt event register,				Address offset: 0x10	*/
	uint32_t PR;			/*	Pending register,								Address offset: 0x14	*/
}EXTI_RegDef_t;


/* Peripheral register definition structure for SYSCFG */
__vo typedef struct {
	uint32_t MEMRMP;		/*	memory remap register,							Address offset: 0x00	*/
	uint32_t PMC;			/*	peripheral mode configuration register,			Address offset: 0x04	*/
	uint32_t EXTICR[4];		/*	external interrupt configuration register,		Address offset: 0x08 - 0x14	*/
	uint32_t Reserved[2];	/*	Reserved: 0x18 - 0x1C */
	uint32_t CMPCR;			/*	Compensation cell control register,				Address offset: 0x20	*/
}SYSCFG_RegDef_t;


/* Peripheral register definition structure for SPI */
__vo typedef struct {
	uint32_t CR[2];
	uint32_t SR;
	uint32_t DR;
	uint32_t CRCPR;
	uint32_t RXCRCR;
	uint32_t TXCRCR;
	uint32_t I2SCFGR;
	uint32_t I2SPR;
}SPI_RegDef_t;


/* Peripheral register definition structure for I2C */
__vo typedef struct {
	uint32_t CR1;
	uint32_t CR2;
	uint32_t OAR1;
	uint32_t OAR2;
	uint32_t DR;
	uint32_t SR1;
	uint32_t SR2;
	uint32_t CCR;
	uint32_t TRISE;
	uint32_t FLTR;
}I2C_RegDef_t;


/* Peripheral register definition structure for USART */
__vo typedef struct {
	uint32_t SR;
	uint32_t DR;
	uint32_t BRR;
	uint32_t CR1;
	uint32_t CR2;
	uint32_t CR3;
	uint32_t GTPR;
}USART_RegDef_t;


/* Peripheral register definition structure for TIMER */
__vo typedef struct {
	uint32_t CR1;
	uint32_t CR2;
	uint32_t res1;
	uint32_t DIER;
	uint32_t SR;
	uint32_t EGR;
	uint32_t res2[3];
	uint32_t CNT;
	uint32_t PSC;
	uint32_t ARR;
}TIM_RegDef_t;


/* Peripheral definitions (Peripheral base addresses type-casted to GPIO_RegDef_t*) */
#define GPIOA	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF	((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG	((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH	((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI	((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC		((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI	((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/* SPI peripheral definitions (Peripheral base addresses type-casted to SPI_RegDef_t*) */
#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2_I2S	((SPI_RegDef_t*)SPI2_I2S_BASEADDR)
#define SPI3_I2S	((SPI_RegDef_t*)SPI3_I2S_BASEADDR)


/* I2C peripheral definitions (Peripheral base addresses type-casted to I2C_RegDef_t*) */
#define I2C1		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t*)I2C3_BASEADDR)


/* USART/UART peripheral definitions (Peripheral base addresses type-casted to USART_RegDef_t*) */
#define USART1		((USART_RegDef_t*)USART1_BASEADDR)
#define USART2		((USART_RegDef_t*)USART2_BASEADDR)
#define USART3		((USART_RegDef_t*)USART3_BASEADDR)
#define UART4		((USART_RegDef_t*)UART4_BASEADDR)
#define UART5		((USART_RegDef_t*)UART5_BASEADDR)
#define USART6		((USART_RegDef_t*)USART6_BASEADDR)


/* TIM peripheral definitions (Peripheral base addresses type-casted to TIM_RegDef_t*) */
#define TIM6		((TIM_RegDef_t*)TIM6_BASEADDR)
#define TIM7		((TIM_RegDef_t*)TIM7_BASEADDR)


/*
 *  Clock enable macros
 */


/* Clock enable macros for GPIOx peripherals */
#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOF_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 5 ) )
#define GPIOG_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 6 ) )
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 7 ) )
#define GPIOI_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 8 ) )


/* Clock enable macros for I2Cx peripherals */
#define I2C1_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 21 ) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 22 ) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 23 ) )


/* Clock enable macros for SPIx peripherals */
#define SPI1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 12 ) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 14 ) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 15 ) )


/* Clock enable macros for USARTx peripherals */
#define USART1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 4 ) )
#define USART2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 17 ) )
#define USART3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 18 ) )
#define UART4_PCLK_EN()			( RCC->APB1ENR |= ( 1 << 19 ) )
#define UART5_PCLK_EN()			( RCC->APB1ENR |= ( 1 << 20 ) )
#define USART6_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 5 ) )


/* Clock enable macros for TIMx peripherals */
#define TIM6_PCLK_EN()			( RCC->APB1ENR |= ( 1 << 4 ) )
#define TIM7_PCLK_EN()			( RCC->APB1ENR |= ( 1 << 5 ) )


/* Clock enable macros for SYSCFG peripherals */
#define SYSCFG_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 14 ) )


/*
 *  Clock disable macros
 */


/* Clock disable macros for GPIOx peripherals */
#define GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOF_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 5 ) )
#define GPIOG_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 6 ) )
#define GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 7 ) )
#define GPIOI_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 8 ) )


/* Clock disable macros for I2Cx peripherals */
#define I2C1_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 21 ) )
#define I2C2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 22 ) )
#define I2C3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 23 ) )


/* Clock disable macros for SPIx peripherals */
#define SPI1_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 12 ) )
#define SPI2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 14 ) )
#define SPI3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 15 ) )


/* Clock disable macros for USARTx peripherals */
#define USART1_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 4 ) )
#define USART2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 17 ) )
#define USART3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 18 ) )
#define UART4_PCLK_DI()			( RCC->APB1ENR &= ~( 1 << 19 ) )
#define UART5_PCLK_DI()			( RCC->APB1ENR &= ~( 1 << 20 ) )
#define USART6_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 5 ) )


/* Clock disable macros for TIMx peripherals */
#define TIM6_PCLK_DI()			( RCC->APB1ENR &= ~( 1 << 4 ) )
#define TIM7_PCLK_DI()			( RCC->APB1ENR &= ~( 1 << 5 ) )


/* Clock disable macros for SYSCFG peripherals */
#define SYSCFG_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 14 ) )


/* GPIOx peripheral reset macros */
#define GPIOA_REG_RESET()		do{( RCC->AHB1RSTR |= ( 1 << 0 ));	( RCC->AHB1RSTR &= ~( 1 << 0 ));}while(0)
#define GPIOB_REG_RESET()		do{( RCC->AHB1RSTR |= ( 1 << 1 ));	( RCC->AHB1RSTR &= ~( 1 << 1 ));}while(0)
#define GPIOC_REG_RESET()		do{( RCC->AHB1RSTR |= ( 1 << 2 ));	( RCC->AHB1RSTR &= ~( 1 << 2 ));}while(0)
#define GPIOD_REG_RESET()		do{( RCC->AHB1RSTR |= ( 1 << 3 ));	( RCC->AHB1RSTR &= ~( 1 << 3 ));}while(0)
#define GPIOE_REG_RESET()		do{( RCC->AHB1RSTR |= ( 1 << 4 ));	( RCC->AHB1RSTR &= ~( 1 << 4 ));}while(0)
#define GPIOF_REG_RESET()		do{( RCC->AHB1RSTR |= ( 1 << 5 ));	( RCC->AHB1RSTR &= ~( 1 << 5 ));}while(0)
#define GPIOG_REG_RESET()		do{( RCC->AHB1RSTR |= ( 1 << 6 ));	( RCC->AHB1RSTR &= ~( 1 << 6 ));}while(0)
#define GPIOH_REG_RESET()		do{( RCC->AHB1RSTR |= ( 1 << 7 ));	( RCC->AHB1RSTR &= ~( 1 << 7 ));}while(0)
#define GPIOI_REG_RESET()		do{( RCC->AHB1RSTR |= ( 1 << 8 ));	( RCC->AHB1RSTR &= ~( 1 << 8 ));}while(0)


/* Returns port code for GPIOx base address*/
#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA)?0:\
										 (x == GPIOB)?1:\
										 (x == GPIOC)?2:\
										 (x == GPIOD)?3:\
										 (x == GPIOE)?4:\
										 (x == GPIOF)?5:\
										 (x == GPIOG)?6:\
										 (x == GPIOH)?7:\
										 (x == GPIOI)?8:0)


/* SPIx peripheral reset macros */
#define SPI1_REG_RESET()		do{ (RCC->APB2RSTR |= ( 1 << 12 )); ( RCC->APB2RSTR &= ~( 1 << 12 ));}while(0)
#define SPI2_I2S_REG_RESET()	do{ (RCC->APB1RSTR |= ( 1 << 14 )); ( RCC->APB1RSTR &= ~( 1 << 14 ));}while(0)
#define SPI3_I2S_REG_RESET()	do{ (RCC->APB1RSTR |= ( 1 << 15 )); ( RCC->APB1RSTR &= ~( 1 << 15 ));}while(0)


/* I2Cx peripheral reset macros */
#define I2C1_REG_RESET()		do{ (RCC->APB1RSTR |= ( 1 << 21 )); ( RCC->APB1RSTR &= ~( 1 << 21 ));}while(0)
#define I2C2_REG_RESET()		do{ (RCC->APB1RSTR |= ( 1 << 22 )); ( RCC->APB1RSTR &= ~( 1 << 22 ));}while(0)
#define I2C3_REG_RESET()		do{ (RCC->APB1RSTR |= ( 1 << 23 )); ( RCC->APB1RSTR &= ~( 1 << 23 ));}while(0)


/* USARTx peripheral reset macros */
#define USART1_REG_RESET()		do{ (RCC->APB2RSTR |= ( 1 << 4 )); ( RCC->APB2RSTR &= ~( 1 << 4 ));}while(0)
#define USART6_REG_RESET()		do{ (RCC->APB2RSTR |= ( 1 << 5 )); ( RCC->APB2RSTR &= ~( 1 << 5 ));}while(0)

#define USART2_REG_RESET()		do{ (RCC->APB1RSTR |= ( 1 << 17 )); ( RCC->APB1RSTR &= ~( 1 << 17 ));}while(0)
#define USART3_REG_RESET()		do{ (RCC->APB1RSTR |= ( 1 << 18 )); ( RCC->APB1RSTR &= ~( 1 << 18 ));}while(0)
#define UART4_REG_RESET()		do{ (RCC->APB1RSTR |= ( 1 << 19 )); ( RCC->APB1RSTR &= ~( 1 << 19 ));}while(0)
#define UART5_REG_RESET()		do{ (RCC->APB1RSTR |= ( 1 << 20 )); ( RCC->APB1RSTR &= ~( 1 << 20 ));}while(0)


/* TIMx peripheral reset macros */
#define TIM6_REG_RESET()		do{ (RCC->APB1RSTR |= ( 1 << 4 )); ( RCC->APB1RSTR &= ~( 1 << 4 ));}while(0)
#define TIM7_REG_RESET()		do{ (RCC->APB1RSTR |= ( 1 << 5 )); ( RCC->APB1RSTR &= ~( 1 << 5 ));}while(0)


/* IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51

#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32
#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34
#define IRQ_NO_I2C3_EV			72
#define IRQ_NO_I2C3_ER			73

#define IRQ_NO_USART1			37
#define IRQ_NO_USART2			38
#define IRQ_NO_USART3			39
#define IRQ_NO_UART4			52
#define IRQ_NO_UART5			53
#define IRQ_NO_USART6			71

#define IRQ_NO_TIM6				54
#define IRQ_NO_TIM7				55

/* macros for all the possible priority levels */
#define NVIC_IRQ_PRI0			0
#define NVIC_IRQ_PRI1			1
#define NVIC_IRQ_PRI2			2
#define NVIC_IRQ_PRI3			3
#define NVIC_IRQ_PRI4			4
#define NVIC_IRQ_PRI5			5
#define NVIC_IRQ_PRI6			6
#define NVIC_IRQ_PRI7			7
#define NVIC_IRQ_PRI8			8
#define NVIC_IRQ_PRI9			9
#define NVIC_IRQ_PRI10			10
#define NVIC_IRQ_PRI11			11
#define NVIC_IRQ_PRI12			12
#define NVIC_IRQ_PRI13			13
#define NVIC_IRQ_PRI14			14
#define NVIC_IRQ_PRI15			15


/* Some generic macros */
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET


/*************************************************************
 * Bit position definition of SPI peripheral registers		 *
 *************************************************************/

/* Bit position definition SPI_CR1 */
/*
#define SPI_CR1(x)			(( x == CPHA )?0:\
							 ( x == CPOL )?1:\
							 ( x == BR )?3:\
							 ( x == SPE )?6:\
							 ( x == LSB_FIRST )?7:\
							 ( x == SSI )?8:\
							 ( x == SSM )?9:\
							 ( x == RXONLY )?10:\
							 ( x == DFF )?11:\
							 ( x == CRC_NEXT)?12:\
							 ( x == CRC_EN )?13:\
							 ( x == BIDI_OE )?14:\
							 ( x == BIDI_MODE )?15)
*/
#define SPI_CR1_CPHA				0
#define SPI_CR1_CPOL				1
#define SPI_CR1_MSTR				2
#define SPI_CR1_BR					3
#define SPI_CR1_SPE					6
#define SPI_CR1_LSB_FIRST			7
#define SPI_CR1_SSI					8
#define SPI_CR1_SSM					9
#define SPI_CR1_RXONLY				10
#define SPI_CR1_DFF					11
#define SPI_CR1_CRC_NEXT			12
#define SPI_CR1_EN					13
#define SPI_CR1_BIDI_OE				14
#define SPI_CR1_BIDI_MODE			15


/* Bit position definition SPI_CR2 */
#define SPI_CR2_RXDMEAN				0
#define SPI_CR2_TXDMEAN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE				7


/* Bit position definition SPI_SR */
#define SPI_SR_RXNE					0
#define SPI_SR_TXE					1
#define SPI_SR_CHSIDE				2
#define SPI_SR_UDR					3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF					5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8


/*************************************************************
 * Bit position definition of I2C peripheral registers		 *
 *************************************************************/

/* Bit position definition I2C_CR1 */
#define I2C_CR1_PE					0
#define I2C_CR1_SMBUS				1
#define I2C_CR1_SMBTYPE				3
#define I2C_CR1_ENARP				4
#define I2C_CR1_ENPEC				5
#define I2C_CR1_ENGC				6
#define I2C_CR1_NOSTRETCH			7
#define I2C_CR1_START				8
#define I2C_CR1_STOP				9
#define I2C_CR1_ACK					10
#define I2C_CR1_POS					11
#define I2C_CR1_PEC					12
#define I2C_CR1_ALERT				12
#define I2C_CR1_SWRST				15


/* Bit position definition I2C_CR2 */
#define I2C_CR2_FREQ				5
#define I2C_CR2_ITERREN				8
#define I2C_CR2_ITEVTEN				9
#define I2C_CR2_ITBUFEN				10
#define I2C_CR2_DMAEN				11
#define I2C_CR2_LAST				12


/* Bit position definition I2C_SR1 */
#define I2C_SR1_SB					0
#define I2C_SR1_ADDR				1
#define I2C_SR1_BTF					2
#define I2C_SR1_ADD10				3
#define I2C_SR1_STOPF				4
#define I2C_SR1_RxNE				6
#define I2C_SR1_TxE					7
#define I2C_SR1_BERR				8
#define I2C_SR1_ARLO				9
#define I2C_SR1_AF					10
#define I2C_SR1_OVR					11
#define I2C_SR1_PECERR				12
#define I2C_SR1_TIMEOUT				14
#define I2C_SR1_SMBALERT			15


/* Bit position definition I2C_SR2 */
#define I2C_SR2_MSL					0
#define I2C_SR2_BUSY				1
#define I2C_SR2_TRA					2
#define I2C_SR2_GENCALL				3
#define I2C_SR2_SMBDEFAUL			5
#define I2C_SR2_SMBHOST				6
#define I2C_SR2_DUALF				7
#define I2C_SR2_PEC					15


/****************************************************************
 *  Bit position definition of USART/UART peripheral registers  *
 ****************************************************************/

/* Bit position definition USART_CR1 */
#define USART_CR1_SBK				0
#define USART_CR1_RWU				1
#define USART_CR1_RE				2
#define USART_CR1_TE				3
#define USART_CR1_IDLEIE			4
#define USART_CR1_RXNEIE			5
#define USART_CR1_TCIE				6
#define USART_CR1_TXEIE				7
#define USART_CR1_PEIE				8
#define USART_CR1_PS				9
#define USART_CR1_PCE				10
#define USART_CR1_WAKE				11
#define USART_CR1_M					12
#define USART_CR1_UE				13
#define USART_CR1_OVER8				15

/* Bit position definition USART_CR2 */
#define USART_CR2_ADD				3
#define USART_CR2_LBDL				5
#define USART_CR2_LBDIE				6
#define USART_CR2_LBCL				8
#define USART_CR2_CPHA				9
#define USART_CR2_CPOL				10
#define USART_CR2_CLKEN				11
#define USART_CR2_STOP				13
#define USART_CR2_LINEN				14

/* Bit position definition USART_CR3 */
#define USART_CR3_EIE				0
#define USART_CR3_IREN				1
#define USART_CR3_IRLP				2
#define USART_CR3_HDSEL				3
#define USART_CR3_NACK				4
#define USART_CR3_SCEN				5
#define USART_CR3_DMAR				6
#define USART_CR3_DMAT				7
#define USART_CR3_RTSE				8
#define USART_CR3_CTSE				9
#define USART_CR3_CTSIE				10
#define USART_CR3_ONEBIT			11

/* Bit position definition USART_SR */
#define USART_SR_PE					0
#define USART_SR_FE					1
#define USART_SR_NF					2
#define USART_SR_ORE				3
#define USART_SR_IDLE				4
#define USART_SR_RXNE				5
#define USART_SR_TC					6
#define USART_SR_TXE				7
#define USART_SR_LBD				8
#define USART_SR_CTS				9

/* Bit position definition USART_BRR */
#define USART_BRR_DIV_Fraction		3
#define USART_BRR_DIV_Mantissa		15

/* Bit position definition USART_GTPR */
#define USART_GTPR_PSC				7
#define USART_GTPR_GT				15


/****************************************************************
 *  Bit position definition of USART/UART peripheral registers  *
 ****************************************************************/


/* Bit position definition of TIM_CR1 */
#define TIM_CR1_CEN					0
#define TIM_CR1_UDIS				1
#define TIM_CR1_URS					2
#define TIM_CR1_OPM					3
#define TIM_CR1_ARPE				7

/* Bit position definition of TIM_CR2 */
#define TIM_CR2_MMS					4

/* Bit position definition of TIM_DIER */
#define TIM_DIER_UIE				0
#define TIM_DIER_UDE				8

/* Bit position definition of TIM_SR */
#define TIM_SR_UIF					0

/* Bit position definition of TIM_EGR */
#define TIM_EGR_UG					0


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx_tim_driver.h"


#endif /* INC_STM32F407XX_H_ */
