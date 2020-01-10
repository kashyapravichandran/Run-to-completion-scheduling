#include "RTCS.h"
#include <stdint.h>
#include "gpio_defs.h"

volatile uint32_t RTCS_Num_Ticks=0;
volatile RTCS_TASK_ENTRY RTCS_Task_Table[RTCS_NUM_TASKS];

void RTCS_Init(uint32_t freq) {
	int i;

	for (i=0; i<RTCS_NUM_TASKS; i++) {
		RTCS_Task_Table[i].Task = (void *)0;
		RTCS_Task_Table[i].Period = 0;
		RTCS_Task_Table[i].TicksToNextRelease = 0;
		RTCS_Task_Table[i].ReleasesPending = 0;
		RTCS_Task_Table[i].Enabled = 0;
	}
	
	Init_LPTMR(freq);
	Start_LPTMR();
}

void RTCS_Run_Scheduler(void){
	int i;
	SET_BIT(DEBUG_RTCS_SCHED);

	while (1) {
		// start at top of table to find highest priority ready task

		for (i=0; i<RTCS_NUM_TASKS; i++) {

			if ((RTCS_Task_Table[i].Task != 0) && (RTCS_Task_Table[i].Enabled) &&
				(RTCS_Task_Table[i].ReleasesPending > 0)) {
				RTCS_Task_Table[i].ReleasesPending--;
				CLEAR_BIT(DEBUG_RTCS_SCHED);
				RTCS_Task_Table[i].Task();
				SET_BIT(DEBUG_RTCS_SCHED);
				break; // exit from for loop, to start at top of table again
			}
		}
	}
}

void RTCS_Timer_Tick(void){
	int i;
	
	SET_BIT(DEBUG_RTCS_TICK);
	RTCS_Num_Ticks++;
	
	for (i=0; i<RTCS_NUM_TASKS; i++) {
		if ((RTCS_Task_Table[i].Task != 0) && (RTCS_Task_Table[i].Enabled) && 
			(RTCS_Task_Table[i].TicksToNextRelease > 0)) {
			if (--RTCS_Task_Table[i].TicksToNextRelease == 0) { // decrement TicksToNextRelease, if 0 then time to run
				RTCS_Task_Table[i].ReleasesPending++;
				RTCS_Task_Table[i].TicksToNextRelease = RTCS_Task_Table[i].Period;
			}
		}
	}
	CLEAR_BIT(DEBUG_RTCS_TICK);
}

int RTCS_Add_Task(void (*task)(void), uint32_t priority, uint32_t period) {
	if (priority < RTCS_NUM_TASKS) {
		RTCS_Task_Table[priority].Task = task;
		RTCS_Task_Table[priority].Period = period;
		RTCS_Task_Table[priority].TicksToNextRelease = period;
		RTCS_Task_Table[priority].ReleasesPending = 1;
		RTCS_Task_Table[priority].Enabled = 1;
		return 1;
	} 
	return 0;
}

// Returns task priority (slot) 0 to max. -1 on failure.
int RTCS_Find_Task_Priority(void (*task)(void)) {
	uint32_t i;
	
	for (i=0; i<RTCS_NUM_TASKS; i++) {
		if (RTCS_Task_Table[i].Task == task) {
				return i;
		}
	}
	return -1;
}

int RTCS_Enable_Task_i(int i, uint32_t enable) {
	if (i < RTCS_NUM_TASKS) {
			RTCS_Task_Table[i].Enabled = enable;
			return 1;
	}
	return 0;
}

int RTCS_Enable_Task(void (*task)(void), uint32_t enable) {
	int i;

	i = RTCS_Find_Task_Priority(task);
	if ((i >= 0) && (i<RTCS_NUM_TASKS)) {
		return RTCS_Enable_Task_i(i, enable);
	}
	return 0;
}


int RTCS_Set_Task_Period_i(int i, uint16_t period, int release_now){
	
	if ((i >= 0) && (i<RTCS_NUM_TASKS)) {
		RTCS_Task_Table[i].Period = period;
		RTCS_Task_Table[i].TicksToNextRelease = period;
		if (release_now)
			RTCS_Task_Table[i].ReleasesPending++;
		return 1;
	}
	return 0;
}

int RTCS_Set_Task_Period(void (*task)(void), uint16_t period, int release_now){
	int i;
	
	i = RTCS_Find_Task_Priority(task);
	if (i >= 0) {
		return RTCS_Set_Task_Period_i(i, period, release_now);
	}
	return 0;
}

int RTCS_Release_Task_i(int i) {
	if (i < RTCS_NUM_TASKS) {
		if (RTCS_Task_Table[i].Task != 0) {
			RTCS_Task_Table[i].ReleasesPending++;
			return 1;
		}
	}
	return 0;
}

int RTCS_Release_Task(void (*task)(void)) {
	int i;

	i = RTCS_Find_Task_Priority(task);
	if ((i >= 0) && (i<RTCS_NUM_TASKS)) {
			RTCS_Task_Table[i].ReleasesPending++;
			return 1;
	}
	return 0;
}

int RTCS_Clear_Task_Releases(void (*task)(void)) {
	int i;

	i = RTCS_Find_Task_Priority(task);
	if ((i >= 0) && (i<RTCS_NUM_TASKS)) {
			RTCS_Task_Table[i].ReleasesPending = 0;
			return 1;
	}
	return 0;
}

int RTCS_Decrement(void (*task) (void)){
	int i;
	i=RTCS_Find_Task_Priority(task);
	if(i>=0){
			RTCS_Task_Table[i].ReleasesPending = RTCS_Task_Table[i].ReleasesPending-1;
			return 1;
	}
	return 0;
}
		


