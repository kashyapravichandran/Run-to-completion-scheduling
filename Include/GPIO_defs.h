#ifndef GPIO_DEFS_H
#define GPIO_DEFS_H

#define MASK(x) (1UL << (x))

#define CONFIG1_POS (3) // on port E
#define CONFIG2_POS (4) // on port E
#define CONFIG3_POS (5) // on port E

#define SET_BIT(x) {PTB->PSOR = MASK(x);}
#define CLEAR_BIT(x) {PTB->PCOR = MASK(x);}

// Debug Signals on port B
#define DBG_0 0
#define DBG_1 1
#define DBG_2 2
#define DBG_3 3
#define DBG_4 8
#define DBG_5 9
#define DBG_6 10
#define DBG_7 11

#define DEBUG_RTCS_TICK (DBG_0)	
#define DEBUG_RTCS_SCHED (DBG_1)	
#define DEBUG_TASK_MOTION_SENSOR (DBG_2) 		
#define DEBUG_I2C_CODE (DBG_3)				
#define DEBUG_MSG_ON_BUS (DBG_4)				
#define DEBUG_I2C_BUSY_WAIT (DBG_5)				

#endif
