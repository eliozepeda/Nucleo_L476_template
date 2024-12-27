/*
 * stm32l476xx.h
 *
 *  Created on: Jul 2, 2023
 *      Author: ezepedasanchez
 *
 *  This header file includes the definition of different addresses of the STM32 NUCLEO-L476RG with the
 *  drivers creation purpose
 */

#ifndef INC_STM32L476XX_H_
#define INC_STM32L476XX_H_
#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))
/*CORTEX M4 Processor specific*/
/*Interrupt Set-enable Registers*/
#define NVIC_ISER0  		((__vo uint32_t*)0xE000E100U)
#define NVIC_ISER1  		((__vo uint32_t*)0xE000E104U)
#define NVIC_ISER2  		((__vo uint32_t*)0xE000E108U)
#define NVIC_ISER3  		((__vo uint32_t*)0xE000E10CU)

/*Interrupt Clear-enable Registers*/
#define NVIC_ICER0  		((__vo uint32_t*)0XE000E180U)
#define NVIC_ICER1  		((__vo uint32_t*)0XE000E184U)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188U)
#define NVIC_ICER3  		((__vo uint32_t*)0XE000E18CU)

/*Interrupt Priority Registers*/
#define NVIC_IPR_BASE_ADDR  ((__vo uint32_t*)0xE000E400U)

#define NO_PR_BITS_IMPLEMENTED 	4

/*Systick registers*/
#define SYST_RVR			((__vo uint32_t*)0xE000E014U)	//Systick reload value register
#define SYST_CSR			((__vo uint32_t*)0xE000E010U)	//Systick control and status register
/*SysTick Control and Status Register Bits positions*/
#define TICKINT_BIT				1
#define CLKSOURCE_BIT			2
#define SYSTYCK_ENABLE_BIT		0

/*System Control Block Registers*/
#define ICSR 				((__vo uint32_t*)0xE000ED04U)
#define SCB_SHPR3			((__vo uint32_t*)0xE000ED20U) //System Handler Prioriry Register 3

/*Interrupt Control and State Register Bits positions*/
#define PENDSVSET_BIT 			28

/*System Handler Prioriry Register 3 Bits positions*/
#define SYSTICK_PRI_BIT 		28

/******************************/

/*---------------------BASE ADDRESSES BEGIN---------------------*/

/*Memories*/
/*All this definitions can be find in the page 73 of the reference manual
 * Figure 3. Memory map for STM32L476xx device*/
#define EZ_FLASH_BASEADDR 	0x08000000U
#define EZ_SRAM1_BASEADDR 	0x20000000U
#define EZ_SRAM2_BASEADDR 	0x10000000U 	/*Is not contiguous to SRAM1 in this board*/
#define EZ_ROM_BASEADDR 	0x1FFF0000U 	/*System memory*/

/*Peripheral Buses*/
#define EZ_APB1_BASEADDR 	0x40000000U
#define EZ_APB2_BASEADDR 	0x40010000U
#define EZ_AHB1_BASEADDR 	0x40020000U
#define EZ_AHB2_BASEADDR 	0x48000000U
#define EZ_AHB3_BASEADDR 	0xA0000000U /*This is the base addr according to the manual reference 2.2.2*/

/*
 * Base addresses of peripherals connected to APB1 bus
 */

#define EZ_SPI2_BASEADDR 	0x40003800U
#define EZ_SPI3_BASEADDR 	0x40003C00U
#define EZ_I2C1_BASEADDR 	0x40005400U
#define EZ_I2C2_BASEADDR 	0x40005800U
#define EZ_I2C3_BASEADDR 	0x40005C00U


/*
 * Base addresses of peripherals connected to APB2 bus
 */

#define EZ_SYSCFG_BASEADDR 	0x40010000U
#define EZ_EXTI_BASEADDR 	0x40010400U
#define EZ_SPI1_BASEADDR 	0x40013000U

/*
 * Base addresses of peripherals connected to AHB2 bus
 */

#define EZ_GPIOA_BASEADDR 	0x48000000U
#define EZ_GPIOB_BASEADDR 	0x48000400U
#define EZ_GPIOC_BASEADDR 	0x48000800U
#define EZ_GPIOD_BASEADDR 	0x48000C00U
#define EZ_GPIOE_BASEADDR 	0x48001000U
#define EZ_GPIOF_BASEADDR 	0x48001400U
#define EZ_GPIOG_BASEADDR 	0x48001800U
#define EZ_GPIOH_BASEADDR 	0x48001C00U

#define EZ_RCC_BASEADDR 	0x40021000U


/*---------------------BASE ADDRESSES END---------------------*/

/************************Peripheral Register Definitions Structures****************************/
/*Note: Registers of a peripheral are specific to STM32L476xx MCU*/

/*Peripheral register definition structure for GPIO*/
typedef struct
{
	__vo uint32_t MODER;	/*Mode register. These bits are written by software to configure the I/O mode. 																				Address offset:0x00*/
	__vo uint32_t OTYPER;	/*Output type register. These bits are written by software to configure the I/O output type. 																Address offset:0x04*/
	__vo uint32_t OSPEEDR;	/*Output speed register. These bits are written by software to configure the I/O output speed. 																Address offset:0x08*/
	__vo uint32_t PUPDR;	/*Pull-up/pull-down register. These bits are written by software to configure the I/O pull-up or pull-down. 												Address offset:0x0C*/
	__vo uint32_t IDR;		/*Input data register. These bits are read-only. They contain the input value of the corresponding I/O port. 												Address offset:0x10*/
	__vo uint32_t ODR;		/*Output data register. These bits are used to control the I/O state of the I/O pins. 																		Address offset:0x14*/
	__vo uint32_t BSRR;		/*Bit set/reset register. These bits are used to set, reset the pins of the GPIO, write only. 																Address offset:0x18*/
	__vo uint32_t LCKR;		/*Configuration lock register. These bits are used to lock the configuration of the port bits when a correct write sequence is applied to bit 16 (LCKK). 	Address offset:0x1C*/
	__vo uint32_t AFR[2];	/*Alternate function register AFR[0] LOW REG AFR[1] HIGH REG. These bits are written by software to configure alternate function I/O. 						Address offset:0x20*/
	__vo uint32_t BRR;		/*Bit reset register. These bits are used to reset the corresponding ODx bit. 																				Address offset:0x28*/
	__vo uint32_t ASCR;		/*Analog switch control register. These bits are written by software to configure the analog connection of the IO											Address offset:0x2C*/
}GPIO_Reg_Def_t;


/*Peripheral register definition structure for RCC*/
typedef struct
{
	__vo uint32_t CR;	    		/*Clock control register. These bits are written by software to TURN ON the clocks. 																			Address offset:0x00*/
	__vo uint32_t ICSCR;			/*Internal clock sources calibration register. These bits are written by software to configure the clock calibration. 											Address offset:0x04*/
	__vo uint32_t CFGR;	    		/*Clock configuration register. These bits are written by software to select the clock source. 																	Address offset:0x08*/
	__vo uint32_t PLLCFGR;			/*PLL configuration register. These bits are written by software to configure the PLL clock outputs according to the formulas in RM. 							Address offset:0x0C*/
	__vo uint32_t PLLSAI1CFGR;		/*PLLSAI1 configuration register. These bits are written by software to configure the PLLSAI1 clock outputs according to the formulas in RM. 					Address offset:0x10*/
	__vo uint32_t PLLSAI2CFGR;		/*PLLSAI2 configuration register. These bits are written by software to configure the PLLSAI2 clock outputs according to the formulas in RM. 					Address offset:0x14*/
	__vo uint32_t CIER;				/*Clock interrupt enable register. These bits are written by software to configure the clock interrupts. 														Address offset:0x18*/
	__vo uint32_t CIFR;				/*Clock interrupt flag register. These bits are written by hardware to know the state of a clock interrupt. 													Address offset:0x1C*/
	__vo uint32_t CICR;				/*Clock interrupt clear register. These bits are written by software to clear clock interrupts. 																Address offset:0x20*/
	__vo uint32_t RCC_RESERVED_1;	/*This 4 bytes are reserved. 																																	Address offset:0x24*/
	__vo uint32_t AHB1RSTR;			/*AHB1 peripheral reset register. These bits are written by software to reset a peripheral connected to AHB1. 													Address offset:0x28*/
	__vo uint32_t AHB2RSTR;			/*AHB2 peripheral reset register. These bits are written by software to reset a peripheral connected to AHB2. 													Address offset:0x2C*/
	__vo uint32_t AHB3RSTR;			/*AHB3 peripheral reset register. These bits are written by software to reset a peripheral connected to AHB3. 													Address offset:0x30*/
	__vo uint32_t RCC_RESERVED_2;	/*This 4 bytes are reserved. 																																	Address offset:0x34*/
	__vo uint32_t APB1RSTR1;		/*APB1 peripheral reset register 1. These bits are written by software to reset a peripheral connected to APB1. 												Address offset:0x38*/
	__vo uint32_t APB1RSTR2;		/*APB1 peripheral reset register 2. These bits are written by software to reset a peripheral connected to APB1. 												Address offset:0x3C*/
	__vo uint32_t APB2RSTR;			/*APB2 peripheral reset register. These bits are written by software to reset a peripheral connected to APB2. 													Address offset:0x40*/
	__vo uint32_t RCC_RESERVED_3;	/*This 4 bytes are reserved. 																																	Address offset:0x44*/
	__vo uint32_t AHB1ENR;			/*AHB1 peripheral clock enable register. These bits are written by software to enable a peripheral connected to AHB1. 											Address offset:0x48*/
	__vo uint32_t AHB2ENR;			/*AHB2 peripheral clock enable register. These bits are written by software to enable a peripheral connected to AHB2. 											Address offset:0x4C*/
	__vo uint32_t AHB3ENR;			/*AHB3 peripheral clock enable register. These bits are written by software to enable a peripheral connected to AHB3. 											Address offset:0x50*/
	__vo uint32_t RCC_RESERVED_4;	/*This 4 bytes are reserved. 																																	Address offset:0x54*/
	__vo uint32_t APB1ENR1;			/*APB1 peripheral clock enable register 1. These bits are written by software to enable a peripheral connected to APB1. 										Address offset:0x58*/
	__vo uint32_t APB1ENR2;			/*APB1 peripheral clock enable register 2. These bits are written by software to enable a peripheral connected to APB1. 										Address offset:0x5C*/
	__vo uint32_t APB2ENR;			/*APB2 peripheral clock enable register. These bits are written by software to enable a peripheral connected to APB2. 											Address offset:0x60*/
	__vo uint32_t RCC_RESERVED_5;	/*This 4 bytes are reserved. 																																	Address offset:0x64*/
	__vo uint32_t AHB1SMENR;		/*AHB1 peripheral clocks enable in Sleep and Stop modes register. These bits are written by software to enable a peripheral connected to AHB1 in sleep mode. 	Address offset:0x68*/
	__vo uint32_t AHB2SMENR;		/*AHB2 peripheral clocks enable in Sleep and Stop modes register. These bits are written by software to enable a peripheral connected to AHB2 in sleep mode. 	Address offset:0x6C*/
	__vo uint32_t AHB3SMENR;		/*AHB3 peripheral clocks enable in Sleep and Stop modes register. These bits are written by software to enable a peripheral connected to AHB3 in sleep mode. 	Address offset:0x70*/
	__vo uint32_t RCC_RESERVED_6;	/*This 4 bytes are reserved. 																																	Address offset:0x74*/
	__vo uint32_t APB1SMENR1;		/*APB1 peripheral clocks enable in Sleep and Stop modes register 1. These bits are written by software to enable a peripheral connected to APB1 in sleep mode. 	Address offset:0x78*/
	__vo uint32_t APB1SMENR2;		/*APB1 peripheral clocks enable in Sleep and Stop modes register 2. These bits are written by software to enable a peripheral connected to APB1 in sleep mode. 	Address offset:0x7C*/
	__vo uint32_t APB2SMENR;		/*APB2 peripheral clocks enable in Sleep and Stop modes register. These bits are written by software to enable a peripheral connected to APB2 in sleep mode.	Address offset:0x80*/
	__vo uint32_t RCC_RESERVED_7;	/*This 4 bytes are reserved. 																																	Address offset:0x84*/
	__vo uint32_t CCIPR;			/*Peripherals independent clock configuration register. These bits are written by software to configure independent clocks. 									Address offset:0x88*/
	__vo uint32_t RCC_RESERVED_8;	/*This 4 bytes are reserved. 																																	Address offset:0x8C*/
	__vo uint32_t BDCR;				/*Backup domain control register. These bits are written by software to configure backup. 																		Address offset:0x90*/
	__vo uint32_t CSR;				/*Control/status register. These bits are written by software and hardware to read and remove reset flags. 														Address offset:0x94*/
	__vo uint32_t CRRCR;			/*Clock recovery RC register. These bits are written by software and hardware to manage clock recovery. 														Address offset:0x98*/
	__vo uint32_t CCIPR2;			/*Peripherals independent clock configuration register2. These bits are written by software to configure independent clocks. 									Address offset:0x9C*/

}RCC_Reg_Def_t;

/*Peripheral register definition structure for EXTI*/
typedef struct
{
	__vo uint32_t IMR1;        /*!< EXTI Interrupt mask register 1,             Address offset: 0x00 */
	__vo uint32_t EMR1;        /*!< EXTI Event mask register 1,                 Address offset: 0x04 */
	__vo uint32_t RTSR1;       /*!< EXTI Rising trigger selection register 1,   Address offset: 0x08 */
	__vo uint32_t FTSR1;       /*!< EXTI Falling trigger selection register 1,  Address offset: 0x0C */
	__vo uint32_t SWIER1;      /*!< EXTI Software interrupt event register 1,   Address offset: 0x10 */
	__vo uint32_t PR1;         /*!< EXTI Pending register 1,                    Address offset: 0x14 */
	uint32_t      RESERVED1;   /*!< Reserved, 0x18                                                   */
	uint32_t      RESERVED2;   /*!< Reserved, 0x1C                                                   */
	__vo uint32_t IMR2;        /*!< EXTI Interrupt mask register 2,             Address offset: 0x20 */
	__vo uint32_t EMR2;        /*!< EXTI Event mask register 2,                 Address offset: 0x24 */
	__vo uint32_t RTSR2;       /*!< EXTI Rising trigger selection register 2,   Address offset: 0x28 */
	__vo uint32_t FTSR2;       /*!< EXTI Falling trigger selection register 2,  Address offset: 0x2C */
	__vo uint32_t SWIER2;      /*!< EXTI Software interrupt event register 2,   Address offset: 0x30 */
	__vo uint32_t PR2;         /*!< EXTI Pending register 2,                    Address offset: 0x34 */
}EXTI_Reg_Def_t;

typedef struct
{
	__vo uint32_t MEMRMP;      /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
	__vo uint32_t CFGR1;       /*!< SYSCFG configuration register 1,                   Address offset: 0x04      */
	__vo uint32_t EXTICR[4];   /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
	__vo uint32_t SCSR;        /*!< SYSCFG SRAM2 control and status register,          Address offset: 0x18      */
	__vo uint32_t CFGR2;       /*!< SYSCFG configuration register 2,                   Address offset: 0x1C      */
	__vo uint32_t SWPR;        /*!< SYSCFG SRAM2 write protection register,            Address offset: 0x20      */
	__vo uint32_t SKR;         /*!< SYSCFG SRAM2 key register,                         Address offset: 0x24      */
} SYSCFG_Reg_Def_t;

typedef struct
{
	__vo uint32_t CR1;         /*!< SPI Control register 1,                              Address offset: 0x00 */
	__vo uint32_t CR2;         /*!< SPI Control register 2,                              Address offset: 0x04 */
	__vo uint32_t SR;          /*!< SPI Status register,                                 Address offset: 0x08 */
	__vo uint32_t DR;          /*!< SPI data register,                                   Address offset: 0x0C */
	__vo uint32_t CRCPR;       /*!< SPI CRC polynomial register,                         Address offset: 0x10 */
	__vo uint32_t RXCRCR;      /*!< SPI Rx CRC register,                                 Address offset: 0x14 */
	__vo uint32_t TXCRCR;      /*!< SPI Tx CRC register,                                 Address offset: 0x18 */
}SPI_Reg_Def_t;

typedef struct
{
  __vo uint32_t CR1;         /*!< I2C Control register 1,            Address offset: 0x00 */
  __vo uint32_t CR2;         /*!< I2C Control register 2,            Address offset: 0x04 */
  __vo uint32_t OAR1;        /*!< I2C Own address 1 register,        Address offset: 0x08 */
  __vo uint32_t OAR2;        /*!< I2C Own address 2 register,        Address offset: 0x0C */
  __vo uint32_t TIMINGR;     /*!< I2C Timing register,               Address offset: 0x10 */
  __vo uint32_t TIMEOUTR;    /*!< I2C Timeout register,              Address offset: 0x14 */
  __vo uint32_t ISR;         /*!< I2C Interrupt and status register, Address offset: 0x18 */
  __vo uint32_t ICR;         /*!< I2C Interrupt clear register,      Address offset: 0x1C */
  __vo uint32_t PECR;        /*!< I2C PEC register,                  Address offset: 0x20 */
  __vo uint32_t RXDR;        /*!< I2C Receive data register,         Address offset: 0x24 */
  __vo uint32_t TXDR;        /*!< I2C Transmit data register,        Address offset: 0x28 */
} I2C_Reg_Def_t;

/*
 * Peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t )
 * */

/*GPIOs definitions*/
#define GPIOA ((GPIO_Reg_Def_t*)EZ_GPIOA_BASEADDR)
#define GPIOB ((GPIO_Reg_Def_t*)EZ_GPIOB_BASEADDR)
#define GPIOC ((GPIO_Reg_Def_t*)EZ_GPIOC_BASEADDR)
#define GPIOD ((GPIO_Reg_Def_t*)EZ_GPIOD_BASEADDR)
#define GPIOE ((GPIO_Reg_Def_t*)EZ_GPIOE_BASEADDR)
#define GPIOF ((GPIO_Reg_Def_t*)EZ_GPIOF_BASEADDR)
#define GPIOG ((GPIO_Reg_Def_t*)EZ_GPIOG_BASEADDR)
#define GPIOH ((GPIO_Reg_Def_t*)EZ_GPIOH_BASEADDR)

/*RCC definition*/
#define RCC   ((RCC_Reg_Def_t*)EZ_RCC_BASEADDR)

/*EXTI definition*/
#define EXTI   ((EXTI_Reg_Def_t*)EZ_EXTI_BASEADDR)


/*SYSCFG definition*/
#define SYSCFG   ((SYSCFG_Reg_Def_t*)EZ_SYSCFG_BASEADDR)

/*SPI definition*/
#define SPI1   ((SPI_Reg_Def_t*)EZ_SPI1_BASEADDR)
#define SPI2   ((SPI_Reg_Def_t*)EZ_SPI2_BASEADDR)
#define SPI3   ((SPI_Reg_Def_t*)EZ_SPI3_BASEADDR)

/*I2C definition*/
#define I2C1   ((I2C_Reg_Def_t*)EZ_I2C1_BASEADDR)
#define I2C2   ((I2C_Reg_Def_t*)EZ_I2C2_BASEADDR)
#define I2C3   ((I2C_Reg_Def_t*)EZ_I2C3_BASEADDR)

/*
 * Peripheral Clock Enable Macros
 * */

/*MCU Source Clock*/
#define RCC_CFGR_SW_POS  0U
#define RCC_CFGR_SWS_POS 2U
#define RCC_CFGR_SWS_HSI 1U

#define HSI_ON() 			(RCC->CR |= (1 << 8))
#define SELECT_HSI() 		do {(RCC->CFGR &= ~(3 << RCC_CFGR_SW_POS)); (RCC->CFGR |= (1 << RCC_CFGR_SW_POS));} while(0)
#define WAIT_HSI_TO_BE_SET() while(((RCC->CFGR >> RCC_CFGR_SWS_POS) & 0x03) != RCC_CFGR_SWS_HSI)
#define SELECT_MSI_4MHZ() 	(RCC->CFGR &= ~(3 << RCC_CFGR_SW_POS))

/*Clock Enable Macros for GPIOx peripherals*/
#define GPIOA_PERI_CLOCK_ENABLE() 	(RCC->AHB2ENR |= (1 << 0))
#define GPIOB_PERI_CLOCK_ENABLE() 	(RCC->AHB2ENR |= (1 << 1))
#define GPIOC_PERI_CLOCK_ENABLE() 	(RCC->AHB2ENR |= (1 << 2))
#define GPIOD_PERI_CLOCK_ENABLE() 	(RCC->AHB2ENR |= (1 << 3))
#define GPIOE_PERI_CLOCK_ENABLE() 	(RCC->AHB2ENR |= (1 << 4))
#define GPIOF_PERI_CLOCK_ENABLE() 	(RCC->AHB2ENR |= (1 << 5))
#define GPIOG_PERI_CLOCK_ENABLE() 	(RCC->AHB2ENR |= (1 << 6))
#define GPIOH_PERI_CLOCK_ENABLE() 	(RCC->AHB2ENR |= (1 << 7))

/*Clock Enable Macros for I2Cx peripherals*/
#define I2C1_PERI_CLOCK_ENABLE() 	(RCC->APB1ENR1 |= (1 << 21))
#define I2C2_PERI_CLOCK_ENABLE() 	(RCC->APB1ENR1 |= (1 << 22))
#define I2C3_PERI_CLOCK_ENABLE() 	(RCC->APB1ENR1 |= (1 << 23))

/*Clock Enable Macros for SPIx peripherals*/
#define SPI1_PERI_CLOCK_ENABLE() 	(RCC->APB2ENR  |= (1 << 12))
#define SPI2_PERI_CLOCK_ENABLE() 	(RCC->APB1ENR1 |= (1 << 14))
#define SPI3_PERI_CLOCK_ENABLE() 	(RCC->APB1ENR1 |= (1 << 15))

/*Clock Enable Macros for SYSCFG peripheral*/
#define SYSCFG_PERI_CLOCK_ENABLE() 	(RCC->APB2ENR  |= (1 << 0))

/*
 * Peripheral Clock Disable Macros
 * */

/*Clock Disable Macros for GPIOx peripherals*/
#define GPIOA_PERI_CLOCK_DISABLE() 	(RCC->AHB2ENR &= ~(1 << 0))
#define GPIOB_PERI_CLOCK_DISABLE() 	(RCC->AHB2ENR &= ~(1 << 1))
#define GPIOC_PERI_CLOCK_DISABLE() 	(RCC->AHB2ENR &= ~(1 << 2))
#define GPIOD_PERI_CLOCK_DISABLE() 	(RCC->AHB2ENR &= ~(1 << 3))
#define GPIOE_PERI_CLOCK_DISABLE() 	(RCC->AHB2ENR &= ~(1 << 4))
#define GPIOF_PERI_CLOCK_DISABLE() 	(RCC->AHB2ENR &= ~(1 << 5))
#define GPIOG_PERI_CLOCK_DISABLE() 	(RCC->AHB2ENR &= ~(1 << 6))
#define GPIOH_PERI_CLOCK_DISABLE() 	(RCC->AHB2ENR &= ~(1 << 7))

/*Clock Disable Macros for I2Cx peripherals*/
#define I2C1_PERI_CLOCK_DISABLE() 	(RCC->APB1ENR1 &= ~(1 << 21))
#define I2C2_PERI_CLOCK_DISABLE() 	(RCC->APB1ENR1 &= ~(1 << 22))
#define I2C3_PERI_CLOCK_DISABLE() 	(RCC->APB1ENR1 &= ~(1 << 23))

/*Clock Disable Macros for SPIx peripherals*/
#define SPI1_PERI_CLOCK_DISABLE() 	(RCC->APB2ENR  &= ~(1 << 12))
#define SPI2_PERI_CLOCK_DISABLE() 	(RCC->APB1ENR1 &= ~(1 << 14))
#define SPI3_PERI_CLOCK_DISABLE() 	(RCC->APB1ENR1 &= ~(1 << 15))

/*Reset Macros for GPIOx peripherals*/
#define GPIOA_RESET() 	do{(RCC->AHB2RSTR |= (1 << 0)); (RCC->AHB2RSTR &= ~(1 << 0));} while(0)
#define GPIOB_RESET() 	do{(RCC->AHB2RSTR |= (1 << 1)); (RCC->AHB2RSTR &= ~(1 << 1));} while(0)
#define GPIOC_RESET() 	do{(RCC->AHB2RSTR |= (1 << 2)); (RCC->AHB2RSTR &= ~(1 << 2));} while(0)
#define GPIOD_RESET() 	do{(RCC->AHB2RSTR |= (1 << 3)); (RCC->AHB2RSTR &= ~(1 << 3));} while(0)
#define GPIOE_RESET() 	do{(RCC->AHB2RSTR |= (1 << 4)); (RCC->AHB2RSTR &= ~(1 << 4));} while(0)
#define GPIOF_RESET() 	do{(RCC->AHB2RSTR |= (1 << 5)); (RCC->AHB2RSTR &= ~(1 << 5));} while(0)
#define GPIOG_RESET() 	do{(RCC->AHB2RSTR |= (1 << 6)); (RCC->AHB2RSTR &= ~(1 << 6));} while(0)
#define GPIOH_RESET() 	do{(RCC->AHB2RSTR |= (1 << 7)); (RCC->AHB2RSTR &= ~(1 << 7));} while(0)

/*Reset Macros for SPIx peripherals*/
#define SPI1_RESET() 	do{(RCC->APB2RSTR |= (1 << 12));   (RCC->APB2RSTR &= ~(1 << 12));} while(0)
#define SPI2_RESET() 	do{(RCC->APB1RSTR1 |= (1 << 14)); (RCC->APB1RSTR1 &= ~(1 << 14));} while(0)
#define SPI3_RESET() 	do{(RCC->APB1RSTR1 |= (1 << 15)); (RCC->APB1RSTR1 &= ~(1 << 15));} while(0)

/*Reset Macros for I2Cx peripherals*/
#define I2C1_RESET() 	do{(RCC->APB2RSTR |= (1 << 21));  (RCC->APB1RSTR1 &= ~(1 << 21));} while(0)
#define I2C2_RESET() 	do{(RCC->APB1RSTR1 |= (1 << 22)); (RCC->APB1RSTR1 &= ~(1 << 22));} while(0)
#define I2C3_RESET() 	do{(RCC->APB1RSTR1 |= (1 << 23)); (RCC->APB1RSTR1 &= ~(1 << 23));} while(0)

inline uint8_t GPIO_BASEADDR_TO_CODE (GPIO_Reg_Def_t *pGPIOx) __attribute__((always_inline));
/*Inline function reduce function call overhead by replacing the call to the function with the
 * actual code of the function itself during compilation. */
inline uint8_t GPIO_BASEADDR_TO_CODE(GPIO_Reg_Def_t *pGPIOx)
{
    if      (pGPIOx == GPIOA) return 0;
    else if (pGPIOx == GPIOB) return 1;
    else if (pGPIOx == GPIOC) return 2;
    else if (pGPIOx == GPIOD) return 3;
    else if (pGPIOx == GPIOE) return 4;
    else if (pGPIOx == GPIOF) return 5;
    else if (pGPIOx == GPIOG) return 6;
    else if (pGPIOx == GPIOH) return 7;
    else                 return 0; // O un valor de error
}


/*General Macros Definitions*/

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define FLAG_RESET	RESET
#define FLAG_SET	SET



/******  STM32 specific Interrupt Numbers **********************************************************************/
typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Cortex-M4 Non Maskable Interrupt                                */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M4 Hard Fault Interrupt                                  */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_PVM_IRQn                = 1,      /*!< PVD/PVM1/PVM2/PVM3/PVM4 through EXTI Line detection Interrupts    */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Channel1_IRQn          = 11,     /*!< DMA1 Channel 1 global Interrupt                                   */
  DMA1_Channel2_IRQn          = 12,     /*!< DMA1 Channel 2 global Interrupt                                   */
  DMA1_Channel3_IRQn          = 13,     /*!< DMA1 Channel 3 global Interrupt                                   */
  DMA1_Channel4_IRQn          = 14,     /*!< DMA1 Channel 4 global Interrupt                                   */
  DMA1_Channel5_IRQn          = 15,     /*!< DMA1 Channel 5 global Interrupt                                   */
  DMA1_Channel6_IRQn          = 16,     /*!< DMA1 Channel 6 global Interrupt                                   */
  DMA1_Channel7_IRQn          = 17,     /*!< DMA1 Channel 7 global Interrupt                                   */
  ADC1_2_IRQn                 = 18,     /*!< ADC1, ADC2 SAR global Interrupts                                  */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM15_IRQn         = 24,     /*!< TIM1 Break interrupt and TIM15 global interrupt                   */
  TIM1_UP_TIM16_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM16 global interrupt                  */
  TIM1_TRG_COM_TIM17_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM17 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  DFSDM1_FLT3_IRQn            = 42,     /*!< DFSDM1 Filter 3 global Interrupt                                  */
  TIM8_BRK_IRQn               = 43,     /*!< TIM8 Break Interrupt                                              */
  TIM8_UP_IRQn                = 44,     /*!< TIM8 Update Interrupt                                             */
  TIM8_TRG_COM_IRQn           = 45,     /*!< TIM8 Trigger and Commutation Interrupt                            */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  ADC3_IRQn                   = 47,     /*!< ADC3 global  Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDMMC1_IRQn                 = 49,     /*!< SDMMC1 global Interrupt                                           */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                                   */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                                   */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                                   */
  DMA2_Channel4_IRQn          = 59,     /*!< DMA2 Channel 4 global Interrupt                                   */
  DMA2_Channel5_IRQn          = 60,     /*!< DMA2 Channel 5 global Interrupt                                   */
  DFSDM1_FLT0_IRQn            = 61,     /*!< DFSDM1 Filter 0 global Interrupt                                  */
  DFSDM1_FLT1_IRQn            = 62,     /*!< DFSDM1 Filter 1 global Interrupt                                  */
  DFSDM1_FLT2_IRQn            = 63,     /*!< DFSDM1 Filter 2 global Interrupt                                  */
  COMP_IRQn                   = 64,     /*!< COMP1 and COMP2 Interrupts                                        */
  LPTIM1_IRQn                 = 65,     /*!< LP TIM1 interrupt                                                 */
  LPTIM2_IRQn                 = 66,     /*!< LP TIM2 interrupt                                                 */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Channel6_IRQn          = 68,     /*!< DMA2 Channel 6 global interrupt                                   */
  DMA2_Channel7_IRQn          = 69,     /*!< DMA2 Channel 7 global interrupt                                   */
  LPUART1_IRQn                = 70,     /*!< LP UART1 interrupt                                                */
  QUADSPI_IRQn                = 71,     /*!< Quad SPI global interrupt                                         */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  SAI1_IRQn                   = 74,     /*!< Serial Audio Interface 1 global interrupt                         */
  SAI2_IRQn                   = 75,     /*!< Serial Audio Interface 2 global interrupt                         */
  SWPMI1_IRQn                 = 76,     /*!< Serial Wire Interface 1 global interrupt                          */
  TSC_IRQn                    = 77,     /*!< Touch Sense Controller global interrupt                           */
  LCD_IRQn                    = 78,     /*!< LCD global interrupt                                              */
  RNG_IRQn                    = 80,     /*!< RNG global interrupt                                              */
  FPU_IRQn                    = 81      /*!< FPU global interrupt                                              */
} IRQn_Type;

/******  STM32 specific Interrupt Priorities**********************************************************************/
typedef enum {
    NVIC_IRQ_PRI0  = 0,
    NVIC_IRQ_PRI1  = 1,
    NVIC_IRQ_PRI2  = 2,
    NVIC_IRQ_PRI3  = 3,
    NVIC_IRQ_PRI4  = 4,
    NVIC_IRQ_PRI5  = 5,
    NVIC_IRQ_PRI6  = 6,
    NVIC_IRQ_PRI7  = 7,
    NVIC_IRQ_PRI8  = 8,
    NVIC_IRQ_PRI9  = 9,
    NVIC_IRQ_PRI10 = 10,
    NVIC_IRQ_PRI11 = 11,
    NVIC_IRQ_PRI12 = 12,
    NVIC_IRQ_PRI13 = 13,
    NVIC_IRQ_PRI14 = 14,
    NVIC_IRQ_PRI15 = 15
} NVIC_Priority;

#endif /* INC_STM32L476XX_H_ */
