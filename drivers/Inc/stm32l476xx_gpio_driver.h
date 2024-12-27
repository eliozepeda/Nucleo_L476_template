/*
 * stm32l476xx_gpio.h
 *
 *  Created on: Mar 9, 2024
 *      Author: Elio Zepeda
 */

#ifndef INC_STM32L476XX_GPIO_DRIVER_H_
#define INC_STM32L476XX_GPIO_DRIVER_H_

#include "stm32l476xx.h"

/*
 * Configuration Structure for a GPIO pin
 */

typedef struct
{
	uint8_t GPIO_PinNumber;			/*Possible values from @GPIO_PIN_NUMBERS*/
	uint8_t GPIO_PinMode; 			/*Possible values from @GPIO_PIN_MODES*/
	uint8_t GPIO_PinSpeed;			/*Possible values from @GPIO_PIN_SPEEDS*/
	uint8_t GPIO_PinPuPdControl;	/*Possible values from @GPIO_PUPD_CONFIGURATIONS*/
	uint8_t GPIO_PinOPType;			/*Possible values from @GPIO_OUTPUT_TYPES*/
	uint8_t GPIO_PinAltFunMode;		/*Possible values from @GPIO_ALTERNATE_FUNCTIONS*/
}GPIO_PinConfig_t;

/*
 * Handler Structure for a GPIO pin
 */

typedef struct
{
	GPIO_Reg_Def_t *pGPIOx; /*Holds the base address of the GPIO where the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig; /*Holds GPIO pin configuration settings*/

}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 */
typedef enum {
    GPIO_PIN_0  = 0,
    GPIO_PIN_1  = 1,
    GPIO_PIN_2  = 2,
    GPIO_PIN_3  = 3,
    GPIO_PIN_4  = 4,
    GPIO_PIN_5  = 5,
    GPIO_PIN_6  = 6,
    GPIO_PIN_7  = 7,
    GPIO_PIN_8  = 8,
    GPIO_PIN_9  = 9,
    GPIO_PIN_10 = 10,
    GPIO_PIN_11 = 11,
    GPIO_PIN_12 = 12,
    GPIO_PIN_13 = 13,
    GPIO_PIN_14 = 14,
    GPIO_PIN_15 = 15
} GPIO_Pin;
/*********************************************************
 ************ APIs supported for this driver**************
 *********************************************************/

/*Peripheral clock setup*/
void GPIO_PeriClockControl(GPIO_Reg_Def_t *pGPIOx, uint8_t EnorDi);

/*Init and De-init*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_Reg_Def_t *pGPIOx);

/*Data read and write*/
uint8_t GPIO_ReadFromInputPin(GPIO_Reg_Def_t *pGPIOx, GPIO_Pin PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Reg_Def_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_Reg_Def_t *pGPIOx, GPIO_Pin PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_Reg_Def_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_Reg_Def_t *pGPIOx, GPIO_Pin PinNumber);

/*IRQ configuration and handling*/
void GPIO_IRQInterruptConfig(IRQn_Type IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(IRQn_Type IRQNumber, NVIC_Priority IRQPriority);
void GPIO_IRQHandling(GPIO_Pin PinNumber); /*When an interrupts occurs the user application can call this function*/


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT		4  /*falling edge*/
#define GPIO_MODE_IT_RT		5  /*rising edge*/
#define GPIO_MODE_IT_FT_RT	6  /*falling edge rising edge*/

/*
 * @GPIO_OUTPUT_TYPES
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * @GPIO_PIN_SPEEDS
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH 	3

/*
 * @GPIO_PUPD_CONFIGURATIONS
 * GPIO pin pull up and pull down configurations
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/*
 * @GPIO_ALTERNATE_FUNCTIONS
 */
typedef enum {
    GPIO_AF0  = 0,
    GPIO_AF1  = 1,
    GPIO_AF2  = 2,
    GPIO_AF3  = 3,
    GPIO_AF4  = 4,
    GPIO_AF5  = 5,
    GPIO_AF6  = 6,
    GPIO_AF7  = 7,
    GPIO_AF8  = 8,
    GPIO_AF9  = 9,
    GPIO_AF10 = 10,
    GPIO_AF11 = 11,
    GPIO_AF12 = 12,
    GPIO_AF13 = 13,
    GPIO_AF14 = 14,
    GPIO_AF15 = 15
} GPIO_AlternateFunction;


#endif /* INC_STM32L476XX_GPIO_DRIVER_H_ */
