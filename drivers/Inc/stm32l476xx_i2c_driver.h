/*
 * stm32l476xx_i2c_driver.h
 *
 *  Created on: May 5, 2024
 *      Author: Elio Zepeda
 */

#ifndef INC_STM32L476XX_I2C_DRIVER_H_
#define INC_STM32L476XX_I2C_DRIVER_H_

#include "stm32l476xx.h"

/*
 * Configuration Structure for I2C
 */

typedef struct
{
	uint32_t Timing;              /*!< Specifies the I2C_TIMINGR_register value.
									 This parameter calculated by referring to I2C initialization section
									 in Reference manual, also Timing configuration tool can be used*/

	uint32_t I2C_DeviceAddress;   /*!< Specifies the first device own address.
									 This parameter can be a 7-bit or 10-bit address. */

	uint8_t I2C_AnalogFilter;	  /*!< Specifies the Analog filter selected
									  @ref I2C_AnalogFilter_states.*/

	uint32_t I2C_DigitalFilter;	 /*!< Specifies the Digital filter value selected.
									  @ref I2C_DigitalFilter_values.*/
	uint32_t NoStretchMode;       /*!< Specifies if nostretch mode is selected.
									  This parameter can be a value of @ref I2C_NOSTRETCH_MODE */

	uint8_t I2C_clock_select;    /*!< Specifies the clock source to drive the timing
	 	 	 	 	 	 	 	 	  This parameter can be a value of @ref I2C_CLK */


}I2C_Config_t;

/*
 * Handler Structure for I2C
 */

typedef struct
{
	 I2C_Reg_Def_t 	*pI2Cx; /*Holds the base address of the I2C*/
	 I2C_Config_t 	I2C_Config; /*Holds I2C configuration settings*/
	 uint8_t 		*pTxBuffer; /*To store the app. tx buffer address*/
	 uint8_t 		*pRxBuffer; /*To store app. Rx buffer address*/
	 uint32_t 		TxLen;	/*To store Tx len*/
	 uint32_t 		RxLen; /*To store Rx len*/
	 uint8_t 		RxTxState; /*To store communication state ref @ I2C_application_states*/
	 uint8_t 		DevAddr; /*To store slave or device address*/
	 uint32_t 		RxSize; /*To store slave or device address*/
	 uint8_t 		Repeated_Start; /*To store slave or device address*/
}I2C_Handle_t;

/*
 * @I2C_application_states
 */
#define I2C_READY		0
#define I2C_BUSY_IN_TX	1
#define I2C_BUSY_IN_RX	2

/*
 * Posibble I2C Application events
 */
#define I2C_EVENT_TX_CMPLT 			1
#define I2C_EVENT_RX_CMPLT 			2
#define I2C_EVENT_BUS_ERR 			3
#define I2C_EVENT_ARLO_ERR 			4
#define I2C_EVENT_OVR_ERR 			5
/*
 * @I2C_AnalogFilter_states
 */
#define I2C_ANALOG_FILTER_ENABLE   0
#define I2C_ANALOG_FILTER_DISABLE  1

/*
 * @I2C_DigitalFilter_values
 */
#define I2C_DIGITAL_FILTER_DISABLE	0
#define I2C_DIGITAL_FILTER_1		1
#define I2C_DIGITAL_FILTER_2		2
#define I2C_DIGITAL_FILTER_3		3
#define I2C_DIGITAL_FILTER_4		4
#define I2C_DIGITAL_FILTER_5		5
#define I2C_DIGITAL_FILTER_6		6
#define I2C_DIGITAL_FILTER_7		7
#define I2C_DIGITAL_FILTER_8		8
#define I2C_DIGITAL_FILTER_9		9
#define I2C_DIGITAL_FILTER_10		10
#define I2C_DIGITAL_FILTER_11		11
#define I2C_DIGITAL_FILTER_12		12
#define I2C_DIGITAL_FILTER_13		13
#define I2C_DIGITAL_FILTER_14		14
#define I2C_DIGITAL_FILTER_15		15

/*
 * @I2C_NOSTRETCH_MODE
 */
#define I2C_CLK_STRETCHING_ENABLE	0
#define I2C_CLK_STRETCHING_DISABLE	1

/*
 * @I2C_CLK
 */
#define PCLK_SELECT		0
#define SYSCLK_SELECT	1
#define HSI16_SELECT	2


/*REGISTERS BITS POSITIONS*/
/*CR1 register*/
#define PE_BIT 			0
#define TXIE_BIT		1
#define RXIE_BIT		2
#define ADDRIE_BIT		3
#define NACKIE_BIT		4
#define STOPIE_BIT		5
#define TCIE_BIT		6
#define I2CERRIE_BIT	7
#define DNF_BIT 		8
#define ANFOFF_BIT 		12
#define SBC_BIT			16
#define NOSTRETCH_BIT 	17
/*CR2 register*/
#define START_BIT		13
#define STOP_BIT		14
#define NACK_BIT		15
#define NBYTES_BIT		16
#define SADD_BIT		1
#define RD_WRN_BIT		10
#define RD_WRN_BIT_MASK	(1 << RD_WRN_BIT)
/*OAR1 register*/
#define OA1EN_BIT 		15
#define OA1ADDR_BIT 	1
/*Interrupt and Status register*/
#define I2C_TXE_BIT		0
#define TXIS_BIT		1
#define I2C_RXNE_BIT	2
#define ADDR_BIT		3
#define NACKF_BIT		4
#define STOPF_BIT		5
#define TC_BIT			6
#define BERR_BIT		8
#define ARLO_BIT		9
#define I2C_OVR_BIT		10
#define I2C_BUSY_BIT	15
#define DIR_BIT			16
/*Interrupt clear register ICR*/
#define I2C_ADDRCF_BIT	3
#define I2C_NACKCF_BIT	4
#define I2C_STOPCF_BIT	5

/*Flags status register*/
#define I2C_TXE_FLAG	(1 << I2C_TXE_BIT)
#define I2C_TXIS_FLAG	(1 << TXIS_BIT)
#define I2C_RXNE_FLAG	(1 << I2C_RXNE_BIT)
#define I2C_NACKF_FLAG	(1 << NACKF_BIT)
#define I2C_TC_FLAG		(1 << TC_BIT)
#define I2C_BERR_FLAG	(1 << BERR_BIT)
#define I2C_ARLO_FLAG	(1 << ARLO_BIT)
#define I2C_OVR_FLAG	(1 << I2C_OVR_BIT)
#define I2C_ADDR_FLAG	(1 << ADDR_BIT)
#define I2C_DIR_FLAG	(1 << DIR_BIT)
#define I2C_STOPF_FLAG	(1 << STOPF_BIT)
#define I2C_BUSY_FLAG	(1 << I2C_BUSY_BIT)
/*Other Macros*/
#define WRITE 0
#define READ  1
#define I2C_NO_REPEAT_START	0
#define I2C_REPEAT_START	1
/*********************************************************
 ************ APIs supported for this driver**************
 *********************************************************/


/*Peripheral clock setup*/
void I2C_PeriClockControl(I2C_Reg_Def_t *pI2Cx, uint8_t EnorDi);

/*Init and De-init*/
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_Handle_t *pI2CHandle);
/*Data send and receive*/
void I2C_Master_Transmit(I2C_Handle_t *pI2CHandle, uint8_t DevAddress, uint8_t *pData, uint32_t Len, uint8_t repeat_start);
void I2C_Master_Receive(I2C_Handle_t *pI2CHandle, uint8_t DevAddress, uint8_t *pData, uint32_t Len, uint8_t repeat_start);
uint8_t I2C_Master_Transmit_IT(I2C_Handle_t *pI2CHandle, uint8_t DevAddress, uint8_t *pData, uint32_t Len, uint8_t repeat_start);
uint8_t I2C_Master_Receive_IT(I2C_Handle_t *pI2CHandle, uint8_t DevAddress, uint8_t *pData, uint32_t Len, uint8_t repeat_start);

void I2C_Slave_Transmit(I2C_Handle_t *pI2CHandle, uint8_t *pData, uint32_t Len);
void I2C_Slave_Receive(I2C_Handle_t *pI2CHandle, uint8_t *pData, uint32_t Len);

void Close_Master_Transmission(I2C_Handle_t *pI2CHandle);
void Close_Master_Reception(I2C_Handle_t *pI2CHandle);
void Generate_StopCondition(I2C_Handle_t *pI2CHandle);

/*IRQ configuration and ISR handling*/
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
/*Other control APIs*/
uint8_t I2C_GetFlagStatus(I2C_Reg_Def_t *pI2Cx, uint32_t FlagName);
void I2C_PeripheralClontrol(I2C_Reg_Def_t *pI2Cx,uint8_t EnorDi);


/*
 * Application Callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t APP_EVENT);



#endif /* INC_STM32L476XX_I2C_DRIVER_H_ */
