/*
 * stm32l476xx_spi_driver.h
 *
 *  Created on: Mar 26, 2024
 *      Author: Elio Zepeda
 */

#ifndef INC_STM32L476XX_SPI_DRIVER_H_
#define INC_STM32L476XX_SPI_DRIVER_H_


#include "stm32l476xx.h"

/*
 * Configuration Structure for SPI
 */

typedef struct
{
	uint8_t SPI_DeviceMode; /*Mater, slave @SPI_DeviceMode*/
	uint8_t SPI_BusConfig;  /*Full duplex, Half duplex, Simplex @SPI_BusConfig*/
	uint8_t SPI_SclkSpeed;  /*Speed selection @SPI_SclkSpeed*/
	uint8_t SPI_DS;			/*Data Size 8 or 16 bits @SPI_DFF*/
	uint8_t SPI_CPOL;		/*@SPI_CPOL*/
	uint8_t SPI_CPHA;		/*@SPI_CPHA*/
	uint8_t SPI_SSM;		/*Software Slave management, Hardware or Software @SPI_SSM*/
}SPI_Config_t;

/*
 * Handler Structure for SPI
 */

typedef struct
{
	SPI_Reg_Def_t 	*pSPIx; /*Holds the base address of the SPI*/
	SPI_Config_t 	SPI_Config; /*Holds GPIO pin configuration settings*/
	uint8_t 		*pTxBuffer; /*To store the app. tx buffer address*/
	uint8_t 		*pRxBuffer; /*To store app. Rx buffer address*/
	uint32_t 		TxLen;	/*To store Tx len*/
	uint32_t 		RxLen; /*To store Rx len*/
	uint8_t 		TxState; /*To store Tx state*/
	uint8_t 		RxState; /*To store Rx state*/

}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE  0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD			1 /*FULL DUPLEX*/
#define SPI_BUS_CONFIG_HD			2 /*HALF DUPLEX*/
#define SPI_BUS_CONFIG_S_RXONLY		3 /*SIMPLEX receive only*/

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 * @SPI_DS
 */
#define SPI_DS_8_BITS	7
#define SPI_DS_16_BITS	15

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_FIRST	0
#define SPI_CPHA_SECOND	1

/*
 * @SPI_SSM
 */
#define SPI_SSM_HW		0
#define SPI_SSM_SW		1

/*
 * Possible SPI Application States
 */
#define SPI_READY		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX	2

/*
 * Posibble SPI Application events
 */
#define SPI_EVENT_TX_CMPLT 		1
#define SPI_EVENT_RX_CMPLT 		2
#define SPI_EVENT_OVR_ERR 		3

/*
 * Other useful macros
 */
#define THRESHOLD_8_BIT		1
/*REGISTERS BITS POSITIONS*/
/*CR1 register*/
#define CPHA_BIT		0
#define CPOL_BIT		1
#define MSTR_BIT		2
#define BR_BIT			3
#define SPE_BIT 		6
#define SSI_BIT 		8
#define SSM_BIT			9
#define RXONLY_BIT 		10
#define BIDIMODE_BIT 	15
/*CR2 register*/
#define SSOE_BIT		2
#define DS_BIT			8
#define FRXTH_BIT		12
#define TXEIE_BIT		7
#define RXNEIE_BIT		6
#define ERRIE_BIT		5
/*Status register*/
#define TXE_BIT			1
#define RXNE_BIT		0
#define OVR_BIT			6
#define BUSY_BIT		7
/*Flags status register*/
#define SPI_TXE_FLAG	(1 << TXE_BIT)
#define SPI_RXNE_FLAG	(1 << RXNE_BIT)
#define SPI_BUSY_FLAG	(1 << BUSY_BIT)

/*********************************************************
 ************ APIs supported for this driver**************
 *********************************************************/

/*Peripheral clock setup*/
void SPI_PeriClockControl(SPI_Reg_Def_t *pSPIx, uint8_t EnorDi);

/*Init and De-init*/
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_Reg_Def_t *pSPIx);

/*Data send and receive*/
void SPI_SendData(SPI_Reg_Def_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_Reg_Def_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);

/*IRQ configuration and ISR handling*/
void SPI_IRQInterruptConfig(IRQn_Type IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(IRQn_Type IRQNumber, NVIC_Priority IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle); /*When an interrupts occurs the user application can call this function*/

/*Other control APIs*/
uint8_t SPI_GetFlagStatus(SPI_Reg_Def_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralClontrol(SPI_Reg_Def_t *pSPIx,uint8_t EnorDi);
void SPI_SSIConfig(SPI_Reg_Def_t *pSPIx,uint8_t EnorDi);
void SPI_SSOEConfig(SPI_Reg_Def_t *pSPIx,uint8_t EnorDi);
void SPI_ClearOVRFlag(SPI_Reg_Def_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application Callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t APP_EVENT);




#endif /* INC_STM32L476XX_SPI_DRIVER_H_ */
