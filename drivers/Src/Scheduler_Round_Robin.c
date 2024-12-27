

#include "Scheduler_Round_Robin.h"

/*Creating control task block for each task*/
TCB_t user_tasks[MAX_TASKS];

/***************************************************************************************
 * @fn                      - init_scheduler_stack
 *
 * @brief                   - Initializes the scheduler stack pointer to the specified start address.
 *
 * @param scheduler_stack_start - Start address of the scheduler stack.
 *
 * @return                  - None
 */
__attribute__((naked)) void init_scheduler_stack(uint32_t scheduler_stack_start) //A naked function don´t have epilogue or prologue, we need to write it
{
	__asm volatile("MSR MSP,%0": : "r" (scheduler_stack_start) : ); //Copy start value to MSP
	__asm volatile ("BX LR"); //Write it to return since there is not epilogue
}


/***************************************************************************************
 * @fn                      - init_systick_timer
 *
 * @brief                   - Configures and initializes the SysTick timer with the specified tick frequency.
 *
 * @param tick_hz           - Desired tick frequency in Hz.
 *
 * @return                  - None
 */
void init_systick_timer(uint32_t tick_hz)
{

	uint32_t count_value = SYSTICK_TIM_CLK/tick_hz; //this is the reload or count value

	/*Clear the value of SysTick Reload Value Register*/
	*SYST_RVR &= ~(0xFFFFFFFF);

	/*Load the value into SysTick Reload Value Register*/
	*SYST_RVR |= (count_value-1); //Reload value should be N-1, Example: 16000-1

	/*Systick Settings in SysTick Control and Status Register*/
	/*Enabling Systick exception request*/
	*SYST_CSR |= (1 << TICKINT_BIT);
	/*Selecting processor clock*/
	*SYST_CSR |= (1 << CLKSOURCE_BIT);

	/*Enabling Sistick timer*/
	*SYST_CSR |= (1 << SYSTYCK_ENABLE_BIT);

}

/***************************************************************************************
 * @fn                      - init_tasks_stack
 *
 * @brief                   - Initializes the Process Stack Pointer (PSP) and state for all user tasks.
 *
 * @param                   - None
 *
 * @return                  - None
 */
void init_tasks_stack(void)
{

	/*Setting initial tasks state*/
	user_tasks[0].current_state = TASK_RUNNING_STATE;
	user_tasks[1].current_state = TASK_RUNNING_STATE;
	user_tasks[2].current_state = TASK_RUNNING_STATE;
	user_tasks[3].current_state = TASK_RUNNING_STATE;
	user_tasks[4].current_state = TASK_RUNNING_STATE;

	/*Setting initial PSP values for tasks*/
	user_tasks[0].psp_value = IDLE_STACK_START;
	user_tasks[1].psp_value = T1_STACK_START;
	user_tasks[2].psp_value = T2_STACK_START;
	user_tasks[3].psp_value = T3_STACK_START;
	user_tasks[4].psp_value = T4_STACK_START;

	user_tasks[0].task_handler = idle_task;
	user_tasks[1].task_handler = task1_handler;
	user_tasks[2].task_handler = task2_handler;
	user_tasks[3].task_handler = task3_handler;
	user_tasks[4].task_handler = task4_handler;

	uint32_t *pPSP;

	for(int i = 0 ; i<MAX_TASKS ; i++)
	{
		pPSP = (uint32_t *)user_tasks[i].psp_value; /*Pointer that points to the task PSP*/

		pPSP--; //decrementing stack position
		*pPSP = DUMMY_XPSR; //saving XPSR value 0X1000000

		pPSP--; //decrementing stack position
		*pPSP = (uint32_t)user_tasks[i].task_handler; //Saving Task Handler

		pPSP--; //decrementing stack position
		*pPSP = EXC_RETURN; //Saving LR with special value

		for(int i = 0 ; i<13 ; i++)
		{
			pPSP--; //decrementing stack position
			*pPSP = 0; //Initializing with 0 General register x
		}

		user_tasks[i].psp_value = (uint32_t)pPSP; //Saving current PSP position of the task
	}
}

/***************************************************************************************
 * @fn                      - switch_sp_to_psp
 *
 * @brief                   - Switches the stack pointer from MSP (Main Stack Pointer) to PSP (Process Stack Pointer).
 *
 * @param                   - None
 *
 * @return                  - None
 */
__attribute__((naked)) void switch_sp_to_psp(void)
{
	/*Initialize the PSP with TASK1 stack start address*/
	//Get the value of PSP of current task
	__asm volatile ("PUSH {LR}"); //Saving LR before calling get_psp_value to be able to go back to the main function (where switch_sp_to_psp is called)
	__asm volatile ("BL get_psp_value"); //Call get_psp_value function and return back, After calling this function the Return value is stored in R0 according to the Procedure Call Standard
	__asm volatile ("MSR PSP,R0"); //Copy R0 value (current Task PSP) to PSP
	__asm volatile ("POP {LR}"); //Restoring LR value


	/*change SP to PSP using CONTROL register*/
	__asm volatile ("MOV R0, #0X02"); //Saving 0x02 in R0 because CONTROL REGISTER Bit 2 configures PS as PSP
	__asm volatile ("MSR CONTROL, R0"); //Copying 0x02 to CONTROL REGISTER
	__asm volatile ("BX LR"); //Returning to main
}

/***************************************************************************************
 * @fn                      - get_psp_value
 *
 * @brief                   - Retrieves the current Process Stack Pointer (PSP) value for the active task.
 *
 * @param                   - None
 *
 * @return                  - Current PSP value.
 */
uint32_t get_psp_value(void)
{
	return user_tasks[current_task].psp_value;
}


/***************************************************************************************
 * @fn                      - SysTick_Handler
 *
 * @brief                   - SysTick timer interrupt handler. Updates tick count, unblocks tasks, and pends the PendSV exception.
 *
 * @param                   - None
 *
 * @return                  - None
 */
void SysTick_Handler(void)
{

	update_global_tick_count();
	unblock_tasks();
	//pend the pendsv exception
	*ICSR |= (1 << PENDSVSET_BIT);
}

/***************************************************************************************
 * @fn                      - update_global_tick_count
 *
 * @brief                   - Increments the global tick count.
 *
 * @param                   - None
 *
 * @return                  - None
 */
void update_global_tick_count(void)
{
	g_tick_count++;
}

/***************************************************************************************
 * @fn                      - unblock_tasks
 *
 * @brief                   - Unblocks tasks whose block count matches the global tick count.
 *
 * @param                   - None
 *
 * @return                  - None
 */
void unblock_tasks(void)
{
	for(int i = 1 ; i < MAX_TASKS ; i++) //starting from 1 because idle task is always in running state
	{
		 /*Check if the task is not running and if it is blocked*/
		if(user_tasks[i].current_state != TASK_RUNNING_STATE)
		{
			/*Check if the task has reached the unblock time*/
			if(user_tasks[i].block_count == g_tick_count)
			{
				/*Unblock the task and set its state to running*/
				user_tasks[i].current_state = TASK_RUNNING_STATE;
			}
		}
	}
}

/***************************************************************************************
 * @fn                      - PendSV_Handler
 *
 * @brief                   - PendSV interrupt handler. Saves the context of the current task and restores the context of the next task.
 *
 * @param                   - None
 *
 * @return                  - None
 */
__attribute__((naked)) void PendSV_Handler(void)
{
	/*Save the context of current task*/

	//1. Get current running task´s PSP value
	__asm volatile("MRS R0, PSP");
	//2. Using that PSP value store SF2( R4 - R11 )
	__asm volatile("STMDB R0!,{R4-R11}");

	__asm volatile ("PUSH {LR}"); //Saving LR before calling save_psp_value to be able to return from PendSV_Handler

	//3. Save the current value of PSP
	__asm volatile("BL save_psp_value"); //Is not necessary to put the PSP value as a parameter because R0 already has that value,
										 //When save_psp_value is called, R0 value is passed as parameter, this because to the Procedure Call Standard

	/*Retrieve the context of the next task*/

	//1. Decide Next task to run
	__asm volatile ("BL update_next_task");

	//2. Get its past PSP value
	__asm volatile ("BL get_psp_value"); //psp value is returned in R0 by standard

	//3. Using that PSP value retreive SF2 (R4 - R11)
	__asm volatile ("LDMIA R0!,{R4-R11}");

	//4. Update PSP and Exit
	__asm volatile ("MSR PSP, R0");

	//5. Restore LR value (0xfffffffd) from stack
	__asm volatile ("POP {LR}");


	/* LR = 0xfffffffd
	 * The processor knows it must use the PSP (Process Stack Pointer) to restore the context
	 * and indicates that the exception will return to Thread mode (task execution).
	 *
	 * The processor uses the PSP to read the values stored in the stack frame.
	 * This includes:
	 * - Registers R0-R3, R12.
	 * - LR (previously stored link register).
	 * - PC (task's return address).
	 * - xPSR (program status register).
	 */
	__asm volatile ("BX LR"); //Returning
}

/***************************************************************************************
 * @fn                      - save_psp_value
 *
 * @brief                   - Saves the current PSP value for the active task.
 *
 * @param current_psp_value - PSP value to save.
 *
 * @return                  - None
 */
void save_psp_value(uint32_t current_psp_value)
{
	user_tasks[current_task].psp_value = current_psp_value;
}

/***************************************************************************************
 * @fn                      - update_next_task
 *
 * @brief                   - Updates the `current_task` to the next runnable task.
 *
 * @param                   - None
 *
 * @return                  - None
 */
void update_next_task(void)
{
	int state = TASK_BLOCKED_STATE;

	for(int i = 0 ; i < MAX_TASKS ; i++)
	{
		/* Move to the next task in the round-robin queue */
		current_task++;

		if(current_task == MAX_TASKS)
		{
			current_task = 0;
		}

		/* Check the state of the next task */
		state = user_tasks[current_task].current_state;

		/* Select the task if it is runnable and not the idle task (task 0) */
		if( (state == TASK_RUNNING_STATE) && (current_task != 0) )
		{
			break;
		}
	}

	/* Go to the idle task if no other task is runnable */
	if(state != TASK_RUNNING_STATE)
	{
		current_task = 0;
	}

}

/***************************************************************************************
 * @fn                      - task_delay
 *
 * @brief                   - Blocks the current task for a specified number of ticks.
 *
 * @param tick_count        - Number of ticks to block the task.
 *
 * @return                  - None
 */
void task_delay(uint32_t tick_count)
{
	/*Disable interrupts to prevent task switches while modifying task state*/
	INTERRUPT_DISABLE();

	/*if not IDLE Task*/
	if(current_task != 0)
	{
		/*Set the delay (in tick counts), 1 tick = 1ms*/
		user_tasks[current_task].block_count = g_tick_count + tick_count;
		/*Change the current task state to BLOCKED*/
		user_tasks[current_task].current_state = TASK_BLOCKED_STATE;
		/*pend the pensv exception to do a task switch*/
		schedule();
	}

	/*Re-enable interrupts to allow task switching again*/
	INTERRUPT_ENABLE();
}

/***************************************************************************************
 * @fn                      - schedule
 *
 * @brief                   - Triggers the PendSV exception to perform task switching.
 *
 * @param                   - None
 *
 * @return                  - None
 */
void schedule(void)
{
	//pend the pendsv exception
	*ICSR |= (1 << PENDSVSET_BIT);
}

/***************************************************************************************
 * @fn                      - Systick_PriorityConfig
 *
 * @brief                   - Configures Systick priority.
 *
 * @param                   - None
 *
 * @return                  - None
 */
void Systick_PriorityConfig(NVIC_Priority IRQPriority)
{
	*SCB_SHPR3 = (*SCB_SHPR3 & 0x00FFFFFF) | (IRQPriority << SYSTICK_PRI_BIT);
}
