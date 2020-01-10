#ifndef RTCS_H
#define RTCS_H

#include <stdint.h>
#include "timers.h"

#define RTCS_NUM_TASKS (10)
#define SCHED_FREQ_HZ (500)
#define TICKS(f) (SCHED_FREQ_HZ/f)
extern volatile uint32_t RTCS_Num_Ticks;

typedef struct {
	void (*Task)(void);
	uint32_t Period;
	uint32_t TicksToNextRelease;
	int ReleasesPending;
	char Enabled;
} RTCS_TASK_ENTRY;

extern volatile RTCS_TASK_ENTRY RTCS_Task_Table[RTCS_NUM_TASKS];

/* 
Runs the scheduler and never returns. Must call RTCS_Add_Task at least once before 
this call.
*/
void RTCS_Run_Scheduler(void);

/*
Initializes task table. Configures LPTimer to generate interrupt at freq Hz.
Maximum freq is 500 Hz.
*/
void RTCS_Init(uint32_t freq);

/* 
Updates TicksToNextRelease (and possibly ReleasesPending) for each active, non-null task.
Must be called from Tick ISR.
*/
void RTCS_Timer_Tick(void);

/*
Adds task to table, overwriting what was previously in priority slot. 
Sets TimeToNextRelease to period, ReleasesPending to 1, and enables task.
*/
int RTCS_Add_Task(void (*task)(void), uint32_t priority, uint32_t period);

/* 
Look up priority/position in table of given task. 
Returns -1 for error. 
*/
int RTCS_Find_Task_Priority(void (*task)(void));

/* 
Update given task's Enable flag based on enable parameter 
*/
int RTCS_Enable_Task_i(int i, uint32_t enable);
int RTCS_Enable_Task(void (*task)(void), uint32_t enable);

/* 
Increment number of releases for given task 
*/
int RTCS_Release_Task_i(int i);
int RTCS_Release_Task(void (*task)(void));

/* 
Zero out number of pending releases for given task
*/
int RTCS_Clear_Task_Releases(void (*task)(void));

/*
Update task period, and reload TicksToNextRelease with period.
If release_now is > 0, increment releases pending.
*/
int RTCS_Set_Task_Period(void (*task)(void), uint16_t period, int release_now);
int RTCS_Set_Task_Period_i(int i, uint16_t period, int release_now);

int RTCS_Decrement(void (*task)(void));

#endif // RTCS_H
