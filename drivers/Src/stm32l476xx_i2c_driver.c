/*
 * stm32l476xx_i2c_driver.c
 *
 *  Created on: May 5, 2024
 *      Author: Elio Zepeda
 */


#include "stm32l476xx_i2c_driver.h"

static void I2C_ExecuteAddressPhase(I2C_Reg_Def_t *pI2Cx, uint8_t SlaveAddr, uint8_t len, uint8_t R_W);
static uint32_t I2C_CLK_SELECT_POSITION(I2C_Reg_Def_t *pI2Cx);
static void I2C_MasterTransmit_interrupt_handle(I2C_Handle_t *pI2CHandle);
static void I2C_MasterTransferComplete_interrupt_handle(I2C_Handle_t *pI2CHandle);
static void I2C_MasterReceive_interrupt_handle(I2C_Handle_t *pI2CHandle);
static void I2C_Flush_TXDR(I2C_Handle_t *pI2CHandle);

/*---------------------------------------------------------------------------------------------------*/
/*									STATIC FUNCTIONS DEFINITIONS									 */
/*---------------------------------------------------------------------------------------------------*/
static void I2C_ExecuteAddressPhase(I2C_Reg_Def_t *pI2Cx, uint8_t SlaveAddr, uint8_t len, uint8_t R_W)
{
	uint32_t temp = 0;

	//clearing NBYTES
	pI2Cx->CR2 &= ~(0xFF << NBYTES_BIT);

	/*Saving Slave addres in SADD register*/
	temp |= SlaveAddr << SADD_BIT;
	/*Setting transfer direction*/
	temp |= R_W << RD_WRN_BIT;
	/*Setting number of bytes to be sent in NBYTES reg*/
	temp |= (len) << NBYTES_BIT; //Saving NBYTES


	pI2Cx->CR2 |= temp;

	/*Generate start condition*/
	pI2Cx->CR2 |= (1 << START_BIT);
}

static uint32_t I2C_CLK_SELECT_POSITION(I2C_Reg_Def_t *pI2Cx)
{
	uint32_t position = 0;

	if(pI2Cx == I2C1)
	{
		position = 12;
	}
	else if(pI2Cx == I2C2)
	{
		position = 14;
	}
	else if(pI2Cx == I2C3)
	{
		position = 16;
	}

	return position;
}

static void I2C_MasterTransmit_interrupt_handle(I2C_Handle_t *pI2CHandle)
{
		/*Send Data*/
		pI2CHandle->pI2Cx->TXDR = *((uint8_t*)pI2CHandle->pTxBuffer);
		pI2CHandle->TxLen--;
		pI2CHandle->pTxBuffer++;
}

static void I2C_MasterTransferComplete_interrupt_handle(I2C_Handle_t *pI2CHandle)
{

	if(pI2CHandle->Repeated_Start == I2C_NO_REPEAT_START)
	{
		pI2CHandle->pI2Cx->CR2 |= (1 << STOP_BIT);
	}

	if(pI2CHandle->pI2Cx->CR2 & RD_WRN_BIT_MASK)
	{
		Close_Master_Reception(pI2CHandle);
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EVENT_RX_CMPLT);
	}
	else
	{
		Close_Master_Transmission(pI2CHandle);
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EVENT_TX_CMPLT);
	}

}

static void I2C_MasterReceive_interrupt_handle(I2C_Handle_t *pI2CHandle)
{
		/*receive data Data*/
		*((uint8_t*)pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->RXDR;
		pI2CHandle->RxLen--;
		pI2CHandle->pRxBuffer++;
}

static void I2C_Flush_TXDR(I2C_Handle_t *pI2CHandle)
{
	  /* If a pending TXIS flag is set */
	  /* Write a dummy data in TXDR to clear it */
	  if (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXIS_FLAG) != RESET)
	  {
		  pI2CHandle->pI2Cx->TXDR = 0x00U;
	  }

	  /* Flush TX register if not empty */
	  if (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG) == RESET)
	  {
	    pI2CHandle->pI2Cx->ISR |= (1 << I2C_TXE_BIT);
	  }
}



/*---------------------------------------------------------------------------------------------------*/
/*									   		APIS DEFINITIONS										 */
/*---------------------------------------------------------------------------------------------------*/



/*Peripheral clock setup*/

/***************************************************************************************
 * @fn						-I2C_PeriClockControl
 *
 * @brief					-This functions enables or disables peripheral clock for the giving I2C
 *
 *
 * @pSPIx 					-Base address of the I2C peripheral
 * @EnorDi 					-Enable or disable MACRO
 *
 * @return 					- none
 */
void I2C_PeriClockControl(I2C_Reg_Def_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PERI_CLOCK_ENABLE();
			}
			else if (pI2Cx == I2C2)
			{
				I2C2_PERI_CLOCK_ENABLE();
			}
			else if (pI2Cx == I2C3)
			{
				I2C3_PERI_CLOCK_ENABLE();
			}
		}
		else
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PERI_CLOCK_DISABLE();
			}
			else if (pI2Cx == I2C2)
			{
				I2C2_PERI_CLOCK_DISABLE();
			}
			else if (pI2Cx == I2C3)
			{
				I2C3_PERI_CLOCK_DISABLE();
			}
		}
}

/*Init and De-init*/
/***************************************************************************************
 * @fn						- I2C_Init
 *
 * @brief					- Initializes the I2C
 *
 * @param SPI_Handle_t 		- Pointer to the I2C handle structure
 *
 * @return 					- None
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//Peripheral clock enable
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	/*1.- Disable the selected I2C peripheral IN CR1*/
	pI2CHandle->pI2Cx->CR1 &= ~(1 << PE_BIT);

	/*2.- Configure ANFOFF and DNF in CR1 */
	tempreg |= pI2CHandle->I2C_Config.I2C_AnalogFilter << ANFOFF_BIT; //Configuring Analog filter
	tempreg |= pI2CHandle->I2C_Config.I2C_DigitalFilter << DNF_BIT; //Configuring Digital filter

	/*3.- Configure NOSTRETCH in CR1 */
	tempreg |= pI2CHandle->I2C_Config.NoStretchMode << NOSTRETCH_BIT;
	pI2CHandle->pI2Cx->CR1 |= tempreg;

	/*4.- Configure TIMING in I2Cx TIMINGR*/
	pI2CHandle->pI2Cx->TIMINGR = pI2CHandle->I2C_Config.Timing;

	/*5.- Configure device address in OAR1*/
	  /* Disable Own Address1 before set the Own Address1 configuration */
	pI2CHandle->pI2Cx->OAR1 &= ~(1 << OA1EN_BIT);
	tempreg=0;
	 /* Set the Own Address1 configuration */
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << OA1ADDR_BIT;
	 /* Enable Own Address1 after set the Own Address1 configuration */
	tempreg |= ENABLE << OA1EN_BIT;
	  /* Writing configurations to OAR1 register */
	pI2CHandle->pI2Cx->OAR1 |= tempreg;

	/*6.- Configuring I2C SBC*/
	//pI2CHandle->pI2Cx->CR1 = ENABLE << SBC_BIT;

	/*7.- Configure I2C clock*/
	if(pI2CHandle->I2C_Config.I2C_clock_select != PCLK_SELECT)
	{
		uint32_t position = I2C_CLK_SELECT_POSITION(pI2CHandle->pI2Cx);
		RCC->CCIPR |= (pI2CHandle->I2C_Config.I2C_clock_select << position);
	}


	/*8.- Enable the selected I2C peripheral IN CR1*/
	pI2CHandle->pI2Cx->CR1 |= (1 << PE_BIT);
}

/***************************************************************************************
 * @fn						- I2C_DeInit
 *
 * @brief					- De-initializes the I2C
 *
 * @param SPI_Reg_Def_t 	- Base address of the I2C peripheral
 *
 * @return 					- None
 */
void I2C_DeInit(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->pI2Cx == I2C1)
	{
		I2C1_RESET();
	}
	else if (pI2CHandle->pI2Cx == I2C2)
	{
		I2C2_RESET();
	}
	else if (pI2CHandle->pI2Cx == I2C3)
	{
		I2C3_RESET();
	}

	/*Disabling I2C*/
	pI2CHandle->pI2Cx->CR1 &= ~(1 << PE_BIT);
	/*Disabling clock*/
	I2C_PeriClockControl(pI2CHandle->pI2Cx, DISABLE);

}

/***************************************************************************************
 * @fn						- I2C_Master_Transmit
 *
 * @brief					- Transmit data in Master Mode
 *
 * @param I2C_Handle_t	 	- Base address of the I2C peripheral
 *
 * @param DevAddress		-Slave Address
 *
 * @param pData				-Data to send
 *
 * @param Len				-Data length
 *
 * @param repeat_start		-Repeated Start condition (ENABLE, DISABLE)
 *
 * @return 					- None
 *
 * @Note 					-This is a blocking call
 */
void I2C_Master_Transmit(I2C_Handle_t *pI2CHandle, uint8_t DevAddress, uint8_t *pData, uint32_t Len, uint8_t repeat_start)
{

		/*Master communication initialization
		 * Generating Start Condition
		 * Sending slave address*/
		I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, DevAddress, Len, WRITE);

		while(Len>0)
		{
			//Wait to TXIS = 1 (I2C_TXDR register is empty)
			while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXIS_FLAG) == FLAG_RESET)
			{

			}

			/*Send Data*/
			pI2CHandle->pI2Cx->TXDR = *pData;
			Len--;
			pData++;


		}


		while(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TC_FLAG) == FLAG_RESET)
		{

		}
		/*Generate stop condition*/
		if(repeat_start == I2C_NO_REPEAT_START)
		{
			pI2CHandle->pI2Cx->CR2 |= (1 << STOP_BIT);
		}


}

/***************************************************************************************
 * @fn						- I2C_Master_Receive
 *
 * @brief					- Receive data in Master Mode
 *
 * @param I2C_Handle_t	 	- Base address of the I2C peripheral
 *
 * @param DevAddress		-Slave Address
 *
 * @param pData				-Variable to store received data
 *
 * @param Len				-Data length of data received
 *
 * @param repeat_start		-Repeated Start condition (ENABLE, DISABLE)
 *
 * @return 					- None
 *
 * @Note 					-This is a blocking call
 */
void I2C_Master_Receive(I2C_Handle_t *pI2CHandle, uint8_t DevAddress, uint8_t *pData, uint32_t Len, uint8_t repeat_start)
{
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, DevAddress, Len, READ);

	while(Len>0)
	{
		//Wait to RXNE = 1 (I2C_RXDR register is not empty)
		while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG) == FLAG_RESET)
		{

		}

		/*receive data Data*/
		*pData = pI2CHandle->pI2Cx->RXDR;
		Len--;
		pData++;
	}

	while(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TC_FLAG) == FLAG_RESET)
	{

	}
	/*Generate stop condition*/
	if(repeat_start == I2C_NO_REPEAT_START)
	{
		pI2CHandle->pI2Cx->CR2 |= (1 << STOP_BIT);
	}


	//clear configuration register 2
	pI2CHandle->pI2Cx->CR2 = 0;
}

/***************************************************************************************
 * @fn						- I2C_Slave_Transmit
 *
 * @brief					- Transmit data in slave mode
 *
 * @param I2C_Handle_t	 	- Base address of the I2C peripheral
 *
 * @param pData				- Variable to store received data
 *
 * @param Len				- Data length of data received
 *
 *
 * @return 					- None
 *
 * @Note 					- This is a blocking call
 */
void I2C_Slave_Transmit(I2C_Handle_t *pI2CHandle, uint8_t *pData, uint32_t Len)
{

	/*Enabling ACK generation in CR2*/
	pI2CHandle->pI2Cx->CR2 &= ~(ENABLE << NACK_BIT);

	/*Wait until ADDR_FLAG is enable*/
	while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG) == FLAG_RESET);

	/*Clear ADDR flag with ADDRCF bit in I2C_ICR*/
	pI2CHandle->pI2Cx->ICR = (ENABLE << I2C_ADDRCF_BIT);


	/*Wait until DIR flag (Transfer direction) is set to transmit mode, This flag is updated when an address match event occurs (ADDR=1).*/
	while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_DIR_FLAG) == FLAG_RESET);

	while(Len>0)
	{
		//Wait to TXIS = 1 (I2C_TXDR register is empty)
		while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXIS_FLAG) == FLAG_RESET)
		{

		}

		/*Send Data*/
		pI2CHandle->pI2Cx->TXDR = *pData;
		Len--;
		pData++;


	}

	/*Wait until receive NACK from Master*/
	while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_NACKF_FLAG) == FLAG_RESET);

	/*Flush the I2C_TXDR if needed*/
	I2C_Flush_TXDR(pI2CHandle);

	/*Clear NACKF flag*/
	pI2CHandle->pI2Cx->ICR = (ENABLE << I2C_NACKCF_BIT);

	/*Wait until STOP flag is set*/
	while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_STOPF_FLAG) == FLAG_RESET);

	/*Clear STOP flag*/
	pI2CHandle->pI2Cx->ICR = (ENABLE << I2C_STOPCF_BIT);

//	/*Wait until BUSY flag is reset*/
  	while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BUSY_FLAG) == FLAG_SET);

	/*Disabling ACK generation in CR2*/
	pI2CHandle->pI2Cx->CR2 |= (ENABLE << NACK_BIT);
}

/***************************************************************************************
 * @fn						- I2C_Slave_Receive
 *
 * @brief					- Receive data in slave mode
 *
 * @param I2C_Handle_t	 	- Base address of the I2C peripheral
 *
 * @param pData				- Variable to store received data
 *
 * @param Len				- Data length of data received
 *
 *
 * @return 					- None
 *
 * @Note 					- This is a blocking call
 */
void I2C_Slave_Receive(I2C_Handle_t *pI2CHandle, uint8_t *pData, uint32_t Len)
{
	/*Enabling ACK generation in CR2*/
	pI2CHandle->pI2Cx->CR2 &= ~(ENABLE << NACK_BIT);

	/*Wait until ADDR_FLAG is enable*/
	while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG) == FLAG_RESET);

	/*Clear ADDR flag with ADDRCF bit in I2C_ICR*/
	pI2CHandle->pI2Cx->ICR = (ENABLE << I2C_ADDRCF_BIT);


	/*Wait until DIR flag (Transfer direction) is set to receive mode, This flag is updated when an address match event occurs (ADDR=1).*/
	while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_DIR_FLAG) == FLAG_SET);

	while(Len>0)
	{
		//Wait to RXNE = 1 (I2C_RXDR register is not empty)
		while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG) == FLAG_RESET)
		{

		}

		/*Receive Data*/
		*pData = pI2CHandle->pI2Cx->RXDR;
		Len--;
		pData++;


	}

	/*Wait until STOP flag is set*/
	while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_STOPF_FLAG) == FLAG_RESET);

	/*Clear STOP flag*/
	pI2CHandle->pI2Cx->ICR = (ENABLE << I2C_STOPCF_BIT);

	/*Wait until BUSY flag is reset*/
	while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BUSY_FLAG) == FLAG_SET);

	/*Disabling ACK generation in CR2*/
	pI2CHandle->pI2Cx->CR2 |= (ENABLE << NACK_BIT);
}

/***************************************************************************************
 * @fn						- I2C_Master_Transmit_IT
 *
 * @brief					- Save information needed to transmit the data in global variables.
 *
 * @param I2C_Handle_t	 	- Handle structure of the I2C peripheral
 *
 * @param DevAddress		- Slave Address
 *
 * @param pData				- Data to send
 *
 * @param Len				- Data length
 *
 * @param repeat_start		- Repeated Start condition (ENABLE, DISABLE)
 *
 * @return 					- Bus status
 *
 */
uint8_t I2C_Master_Transmit_IT(I2C_Handle_t *pI2CHandle, uint8_t DevAddress, uint8_t *pData, uint32_t Len, uint8_t repeat_start)
{
	uint8_t Bus_state = pI2CHandle->RxTxState;
	uint8_t temp = 0;

	if((Bus_state != I2C_BUSY_IN_TX)&&(Bus_state != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pData;
		pI2CHandle->TxLen = Len;
		pI2CHandle->RxTxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = DevAddress;
		pI2CHandle->Repeated_Start = repeat_start;

		/*Enabling control bits for interrupts*/

		temp |= (ENABLE << TXIE_BIT); /*Enabling transmit buffer interrupt*/

		temp |= (ENABLE << TCIE_BIT); /*Enabling transfer complete interrupt*/

		temp |= (ENABLE << I2CERRIE_BIT); /*Enabling error interrupt*/

		pI2CHandle->pI2Cx->CR1 |= temp;

		/*Sending start bit and Slave Address and configure direction (write)*/
		I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, DevAddress, Len, WRITE);

	}
	Bus_state = pI2CHandle->RxTxState;
	return Bus_state;

}

/***************************************************************************************
 * @fn						- I2C_Master_Receive_IT
 *
 * @brief					- Save information needed to receive the data in global variables.
 *
 * @param I2C_Handle_t	 	- Handle structure of the I2C peripheral
 *
 * @param DevAddress		- Slave Address
 *
 * @param pData				- Variable to store received data
 *
 * @param Len				- Data length of data received
 *
 * @param repeat_start		- Repeated Start condition (ENABLE, DISABLE)
 *
 * @return 					- Bus status
 *
 */
uint8_t I2C_Master_Receive_IT(I2C_Handle_t *pI2CHandle, uint8_t DevAddress, uint8_t *pData, uint32_t Len, uint8_t repeat_start)
{
	uint8_t Bus_state = pI2CHandle->RxTxState;
	uint8_t temp = 0;

	if((Bus_state != I2C_BUSY_IN_TX)&&(Bus_state != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pData;
		pI2CHandle->RxLen = Len;
		pI2CHandle->RxTxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = DevAddress;
		pI2CHandle->Repeated_Start = repeat_start;

		/*Enabling control bits for interrupts*/

		temp |= (ENABLE << RXIE_BIT); /*Enabling Receive buffer not empty*/

		temp |= (ENABLE << TCIE_BIT); /*Enabling transfer complete interrupt*/

		temp |= (ENABLE << I2CERRIE_BIT); /*Enabling error interrupt*/

		pI2CHandle->pI2Cx->CR1 |= temp;

		/*Sending start bit and Slave Address and configure direction (read)*/
		I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, DevAddress, Len, READ);

	}

	Bus_state = pI2CHandle->RxTxState;
	return Bus_state;
}



/*IRQ configuration and ISR handling*/
/***************************************************************************************
 * @fn						- I2C_IRQInterruptConfig
 *
 * @brief					- Configures the I2C interrupt
 *
 * @param IRQNumber 		- Interrupt number
 * @param EnorDi 			- Enable or disable
 *
 * @return 					- None
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber %32));
		}
		else if(IRQNumber > 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber %64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber %32));
		}
		else if(IRQNumber > 64 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber %64));
		}
	}
}
/***************************************************************************************
 * @fn						- I2C_IRQPriorityConfig
 *
 * @brief					- Configures the I2C interrupt priority
 *
 * @param IRQNumber 		- Interrupt number
 * @param IRQPriority 		- Interrupt priority
 *
 * @return 					- None
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1 first find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_value = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_value);
}

/***************************************************************************************
 * @fn						- I2C_EV_IRQHandling
 *
 * @brief					- Handling for event interrupts
 *
 * @param I2C_Handle_t	 	- Handle structure of the I2C peripheral
 *
 * @return 					- None
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	/*Interrupt handling for both Master and Slave mode*/
	uint8_t temp1=0;
	uint8_t temp2=0;

	/*Handler for interrupt generated by Transmit buffer interrupt status (TXIS)*/
	temp1 = pI2CHandle->pI2Cx->CR1 & (ENABLE << TXIE_BIT);
	temp2 = pI2CHandle->pI2Cx->ISR & (ENABLE << TXIS_BIT);

	if(temp1 && temp2)
	{
		/*Interrupt generated for TXIS, we have to do data transmission*/
		if(pI2CHandle->RxTxState == I2C_BUSY_IN_TX)
		{
			I2C_MasterTransmit_interrupt_handle(pI2CHandle);

		}

	}

	/*Handler for interrupt generated by Receive buffer not empty (RXNE)*/
	temp1 = pI2CHandle->pI2Cx->CR1 & (ENABLE << RXIE_BIT);
	temp2 = pI2CHandle->pI2Cx->ISR & (ENABLE << I2C_RXNE_BIT);
	if(temp1 && temp2)
	{
		/*Interrupt generated for Receive buffer not empty, we have to read received data*/
		if(pI2CHandle->RxTxState == I2C_BUSY_IN_RX)
		{
			I2C_MasterReceive_interrupt_handle(pI2CHandle);
		}
	}

	/*Handler for interrupt generated by Transfer complete (TC)*/
	temp1 = pI2CHandle->pI2Cx->CR1 & (ENABLE << TCIE_BIT);
	temp2 = pI2CHandle->pI2Cx->ISR & (ENABLE << TC_BIT);
	if(temp1 && temp2)
	{
		/*Interrupt generated for transfer complete, we have to send stop condition and clear variables, call application callback*/
		if((pI2CHandle->RxTxState == I2C_BUSY_IN_TX) || (pI2CHandle->RxTxState == I2C_BUSY_IN_RX))
		{
			I2C_MasterTransferComplete_interrupt_handle(pI2CHandle);
		}
	}



}

/***************************************************************************************
 * @fn						- I2C_ER_IRQHandling
 *
 * @brief					- Handling for error interrupts
 *
 * @param I2C_Handle_t	 	- Handle structure of the I2C peripheral
 *
 * @return 					- None
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint8_t temp1, temp2;

	/*Getting the status of the Interrupt enable control bit ERRIE*/
	temp1 = (pI2CHandle->pI2Cx-> CR1) & (1 << I2CERRIE_BIT);

	/*Check for Bus error*/
	temp2 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BERR_FLAG);
	if(temp1 && temp2)
	{
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EVENT_BUS_ERR); /*Call the application callback*/
	}

	/*Check for Arbitration lost error*/
	temp2 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ARLO_FLAG);
	if(temp1 && temp2)
	{
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EVENT_ARLO_ERR); /*Call the application callback*/
	}

	/*Check for overrun/underrun error*/
	temp2 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_OVR_FLAG);
	if(temp1 && temp2)
	{
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EVENT_OVR_ERR); /*Call the application callback*/
	}



}

/***************************************************************************************
 * @fn						- Close_Master_Transmission
 *
 * @brief					- Calling this call we can close the master transmission
 *
 * @param I2C_Handle_t	 	- Handle structure of the I2C peripheral
 *
 * @return 					- None
 */
void Close_Master_Transmission(I2C_Handle_t *pI2CHandle)
{
	/*Clear transmission flags*/
	uint8_t temp = 0;
	temp |= (ENABLE << TXIE_BIT); /*Mask to disabling transmit buffer interrupt*/

	temp |= (ENABLE << TCIE_BIT); /*Mask to disabling transfer complete interrupt*/

	temp |= (ENABLE << I2CERRIE_BIT); /*Mask to disabling error interrupt*/

	pI2CHandle->pI2Cx->CR1 &= ~temp; /*disabling interrupts*/
	pI2CHandle->Repeated_Start = 0;
	pI2CHandle->TxLen = 0;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->RxTxState = I2C_READY;
}

/***************************************************************************************
 * @fn						- Close_Master_Reception
 *
 * @brief					- Calling this call we can close the master reception.
 *
 * @param I2C_Handle_t	 	- Handle structure of the I2C peripheral
 *
 * @return 					- None
 */
void Close_Master_Reception(I2C_Handle_t *pI2CHandle)
{
	/*Clear transmission flags*/
	uint8_t temp = 0;
	temp |= (ENABLE << RXIE_BIT); /*Mask to disabling transmit buffer interrupt*/

	temp |= (ENABLE << TCIE_BIT); /*Mask to disabling transfer complete interrupt*/

	temp |= (ENABLE << I2CERRIE_BIT); /*Mask to disabling error interrupt*/

	pI2CHandle->pI2Cx->CR1 &= ~temp; /*disabling interrupts*/
	pI2CHandle->Repeated_Start = 0;
	pI2CHandle->RxLen = 0;
	pI2CHandle->pRxBuffer = NULL;
	//clear configuration register 2
	pI2CHandle->pI2Cx->CR2 = 0;
	pI2CHandle->RxTxState = I2C_READY;
}

/***************************************************************************************
 * @fn						- Generate_StopCondition
 *
 * @brief					- In case of error we can call this API to release the bus.
 *
 * @param I2C_Handle_t	 	- Handle structure of the I2C peripheral
 *
 * @return 					- None
 */
void Generate_StopCondition(I2C_Handle_t *pI2CHandle)
{
	pI2CHandle->pI2Cx->CR2 |= (1 << STOP_BIT);
}

/***************************************************************************************
 * @fn						- I2C_GetFlagStatus
 *
 * @brief					- Obtains the state of a flag
 *
 * @param I2C_Reg_Def_t 	- Base address of the I2C peripheral
 * @param FlagName 			- Flag to get
 *
 * @return 					- None
 */
uint8_t I2C_GetFlagStatus(I2C_Reg_Def_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->ISR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/***************************************************************************************
 * @fn						- I2C_ApplicationEventCallback
 *
 * @brief					- This is the application callback, is application dependent, shall be
 * 							  defined in application
 *
 * @param *I2C_Handle_t 	- I2C Handle
 * @param APP_EVENT 		- Application event that causes the callback (TC)
 * @return 					- None
 */
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t APP_EVENT)
{
	//This is a weak implementation. The application may override this function
}
