/*
 * Scheduler_Round_Robin.h
 *
 *  Created on: Nov 30, 2024
 *      Author: Elio Zepeda
 */

#ifndef INC_SCHEDULER_ROUND_ROBIN_H_
#define INC_SCHEDULER_ROUND_ROBIN_H_

#include "stm32l476xx.h"

#define MAX_TASKS			5

/*Stack Memory calculations*/
#define SIZE_TASK_STACK			1024U //1KByte
#define SIZE_SCHEDULER_STACK	1024U //1KByte

#define SRAM_START				0x20000000U
#define SIZE_SRAM				((96) * (1024))
#define SRAM_END				((SRAM_START) + (SIZE_SRAM))

#define T1_STACK_START			SRAM_END
#define T2_STACK_START			((SRAM_END) - (SIZE_TASK_STACK))
#define T3_STACK_START			((SRAM_END) - (2 * SIZE_TASK_STACK))
#define T4_STACK_START			((SRAM_END) - (3 * SIZE_TASK_STACK))
#define IDLE_STACK_START		((SRAM_END) - (4 * SIZE_TASK_STACK))
#define SCHEDULER_STACK_START	((SRAM_END) - (5 * SIZE_TASK_STACK))

/*SysTick Timer configuration Defines*/
#define TICK_HZ 				1000U //Tick Exception frequency
#define HSI_CLK 				16000000U //Internal MCU clock frequency
#define SYSTICK_TIM_CLK 		HSI_CLK	//Systick Timer Clock


/*Defines for context initial values*/
#define DUMMY_XPSR				0x1000000U	//T_Bit = 1
#define EXC_RETURN				0xFFFFFFFDU	//Return to thread mode and use PSP
/************************************/

#define TASK_RUNNING_STATE 		0x00
#define TASK_BLOCKED_STATE 		0xFF

#define INTERRUPT_DISABLE()		do{__asm volatile ("MOV R0,#0x1"); __asm volatile("MSR PRIMASK,R0"); } while(0)
#define  INTERRUPT_ENABLE()		do{__asm volatile ("MOV R0,#0x0"); __asm volatile("MSR PRIMASK,R0"); } while(0)

/* TASK CONTROL BLOCK STRUCTURE DEFINITION */
typedef struct
{
    uint32_t psp_value;         // The Process Stack Pointer (PSP) value for the task
    uint32_t block_count;       // The tick count when the task is unblocked
    uint8_t current_state;      // The current state of the task (e.g., RUNNING, BLOCKED)
    void (*task_handler)(void); // Pointer to the function (task handler) that the task executes
} TCB_t;

extern uint8_t current_task;
extern uint32_t g_tick_count;

__attribute__((naked)) void init_scheduler_stack(uint32_t scheduler_stack_start);
void init_systick_timer(uint32_t tick_hz);
void init_tasks_stack(void);
__attribute__((naked)) void switch_sp_to_psp(void);
uint32_t get_psp_value(void);
void update_global_tick_count(void);
void unblock_tasks(void);
void save_psp_value(uint32_t current_psp_value);
void update_next_task(void);
void task_delay(uint32_t tick_count);
void schedule(void);

void Systick_PriorityConfig(NVIC_Priority IRQPriority);

/*Tasks Handlers Prototypes*/
void task1_handler(void);
void task2_handler(void);
void task3_handler(void);
void task4_handler(void);
void idle_task(void);

#endif /* INC_SCHEDULER_ROUND_ROBIN_H_ */
