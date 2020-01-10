#ifndef TIMERS_H
#define TIMERS_H
#include "MKL25Z4.h"

#define FREQ_LPO (1000)

void Init_PIT(unsigned period);
void Start_PIT(void);
void Stop_PIT(void);

void Init_PWM(void);
void Set_PWM_Values(uint16_t perc1, uint16_t perc2);

void Init_LPTMR(uint32_t freq);
void Start_LPTMR(void);
void Stop_LPTMR(void);


extern volatile unsigned PIT_interrupt_counter;
extern volatile unsigned LCD_update_requested;

#endif
// *******************************ARM University Program Copyright © ARM Ltd 2013*************************************   
