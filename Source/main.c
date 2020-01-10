/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <MKL25Z4.H>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "gpio_defs.h"
#include "LEDs.h"
#include "i2c.h"
#include "mma8451.h"
#include "delay.h"
#include "RTCS.h"

#define FLASH_DELAY 2
#define ACC_SENSITIVITY 90

#define TASK_MOTION_SENSOR_FREQ_HZ (100) 
#define TASK_I2C_SERVER_FSM_FREQ_HZ (500)

void TASK_MOTION_SENSOR_FSM(void);
void TASK_I2C_SERVER_FSM(void);

#define Cond_Exe 0 // change it to 1 for periodic execeution and keep it zero for event based triggering. 

int flag=0;
enum {S1,S2,S3,S4,S5,S6,S7,S8} next_state_task=S1;
//I2C_MESSAGE_T g_I2C_Msg;
int return_variable=0;
void Init_Debug_Signals(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

	PORTB->PCR[DBG_0] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_1] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_2] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_3] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_4] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_5] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_6] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_7] |= PORT_PCR_MUX(1);          

	PTB->PDDR |= MASK(DBG_0) | MASK(DBG_1) | MASK(DBG_2) | MASK(DBG_3) | MASK(DBG_4) | MASK(DBG_5) | MASK(DBG_6) | MASK(DBG_7);
	PTB->PCOR = MASK(DBG_0) | MASK(DBG_1) | MASK(DBG_2) | MASK(DBG_3) | MASK(DBG_4) | MASK(DBG_5) | MASK(DBG_6) | MASK(DBG_7);

}


void Init_Config_Signals(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	PORTE->PCR[CONFIG1_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG1_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;          
	PTE->PDDR &= ~MASK(CONFIG1_POS);

	PORTE->PCR[CONFIG2_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG2_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG2_POS);

	PORTE->PCR[CONFIG3_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG3_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG3_POS);
}

void Task_Motion_Sensor(void) {
	
	static int16_t prev_acc_X=0, prev_acc_Y=0, prev_acc_Z=0;
	int16_t acc_X=0, acc_Y=0, acc_Z=0;
	uint8_t rf, gf, bf;
	SET_BIT(DEBUG_TASK_MOTION_SENSOR);
	read_full_xyz(&acc_X, &acc_Y, &acc_Z);

	rf = abs(prev_acc_X - acc_X) > ACC_SENSITIVITY ? 1 : 0;
	gf = abs(prev_acc_Y - acc_Y) > ACC_SENSITIVITY ? 1 : 0;
	bf = abs(prev_acc_Z - acc_Z) > ACC_SENSITIVITY ? 1 : 0;

	Control_RGB_LEDs(rf, gf, bf);
	Delay(FLASH_DELAY);
	Control_RGB_LEDs(0, 0, 0);							
	Delay(FLASH_DELAY*2);		

	prev_acc_X = acc_X;
	prev_acc_Y = acc_Y;
	prev_acc_Z = acc_Z;
	CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);
}

void TASK_I2C_SERVER_FSM(void){
	static enum{S1,S2,S3,S4,S5,S6,S7,Wait} next_state=S1,prev_state=S1;
	static uint8_t dummy, num_bytes_read=0, is_last_read=0, num_bytes_written=0;;
	SET_BIT(DEBUG_I2C_CODE);
	if(g_I2C_Msg.Command!=NONE)
	switch(next_state){
		case S1:
						I2C_TRAN;													//	set to transmit mode							
						
						I2C_M_START;											//	send start	
						I2C0->D = g_I2C_Msg.Dev_adx;
						next_state=Wait;
						prev_state=S1;
						#if !Cond_Exe
							RTCS_Add_Task(TASK_I2C_SERVER_FSM,1, TICKS(TASK_I2C_SERVER_FSM_FREQ_HZ));
						#endif
						break;


		case S2:		I2C0->D = g_I2C_Msg.Reg_adx;
						next_state=Wait;
						prev_state=S2;
						#if !Cond_Exe
							RTCS_Add_Task(TASK_I2C_SERVER_FSM,1, TICKS(TASK_I2C_SERVER_FSM_FREQ_HZ));
						#endif
						break;
						//wait again
						
		case S3: 		g_I2C_Msg.Status=READING;
						I2C_M_RSTART;
						I2C0->D=g_I2C_Msg.Dev_adx | 0x01;
						next_state=Wait;
						prev_state=S3;
						SET_BIT(DEBUG_MSG_ON_BUS);	
						#if !Cond_Exe
							RTCS_Add_Task(TASK_I2C_SERVER_FSM,1, TICKS(TASK_I2C_SERVER_FSM_FREQ_HZ));
						#endif
						break;

		case S4: 		I2C_REC;
						if(num_bytes_read < g_I2C_Msg.Data_count) 
						{
							is_last_read = (num_bytes_read == g_I2C_Msg.Data_count-1)? 1: 0;
							if (is_last_read){
							NACK;													// tell HW to send NACK after read							
							} else {
							ACK;													// tell HW to send ACK after read								
							}
							dummy = I2C0->D;								//	dummy read										
							next_state=Wait;
							prev_state=S4;
						}
						#if !Cond_Exe
							RTCS_Add_Task(TASK_I2C_SERVER_FSM,1, TICKS(TASK_I2C_SERVER_FSM_FREQ_HZ));
						#endif
						break;

		case S5: 		if(num_bytes_read < g_I2C_Msg.Data_count) 
						{
							if (is_last_read){
								I2C_M_STOP;										//	send stop										
								CLEAR_BIT(DEBUG_MSG_ON_BUS);
								g_I2C_Msg.Status=READ_COMPLETE;
								g_I2C_Msg.Command=NONE;
								next_state=S1;
								#if !Cond_Exe 
								RTCS_Add_Task(TASK_MOTION_SENSOR_FSM, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
						#endif 
							}
							else
							{	
								next_state=S4;
								RTCS_Add_Task(TASK_I2C_SERVER_FSM,1, TICKS(TASK_I2C_SERVER_FSM_FREQ_HZ));
							}
							g_I2C_Msg.Data[num_bytes_read++] = I2C0->D; //	read data										
							
						}
						if(is_last_read)
							num_bytes_read=0;
						
						break;


		case S6:		
						SET_BIT(DEBUG_MSG_ON_BUS);	
						if(num_bytes_written<g_I2C_Msg.Data_count) // write
						{
						I2C0->D = g_I2C_Msg.Data[num_bytes_written++];
						next_state=Wait;
						prev_state=S6;
							
						}
						else 
							next_state=S7;
						
						break;
						
		case S7:
						I2C_M_STOP;
						CLEAR_BIT(DEBUG_MSG_ON_BUS);
						next_state=S1;
						g_I2C_Msg.Status=WRITE_COMPLETE;
						g_I2C_Msg.Command=NONE;	
						num_bytes_written=0;
						#if !Cond_Exe
							RTCS_Add_Task(TASK_I2C_SERVER_FSM,1, TICKS(TASK_I2C_SERVER_FSM_FREQ_HZ));
						#endif
						break;
		case Wait:		
						SET_BIT(DEBUG_I2C_BUSY_WAIT);
						if (( I2C0->S & I2C_S_IICIF_MASK ) == 0 );
						else
						{

							I2C0->S |= I2C_S_IICIF_MASK;
							switch(prev_state){
								case S1: next_state=S2;
									     break;
								case S2:
										if(g_I2C_Msg.Command==READ)
										{
											next_state=S3;
											g_I2C_Msg.Status=READING;
										}
										else if (g_I2C_Msg.Command==WRITE)
										{
											next_state=S6;
											g_I2C_Msg.Status=WRITING;
										}	
									     break;

								case S3:next_state=S4;
									     break;

								case S4:next_state=S5;
											  RTCS_Add_Task(TASK_I2C_SERVER_FSM,1, TICKS(TASK_I2C_SERVER_FSM_FREQ_HZ));
									     break;

								case S6:next_state=S6;
									     break;

								default: break;
									}
								}
								CLEAR_BIT(DEBUG_I2C_BUSY_WAIT);
								#if !Cond_Exe
									RTCS_Add_Task(TASK_I2C_SERVER_FSM,1, TICKS(TASK_I2C_SERVER_FSM_FREQ_HZ));
								#endif
								break;
									
		default:
						next_state=S1;
						break;
	
}					
		CLEAR_BIT(DEBUG_I2C_CODE);	

}
void TASK_MOTION_SENSOR_FSM(void){
	
	static int16_t prev_acc_X=0, prev_acc_Y=0, prev_acc_Z=0;
	static int16_t acc_X=0, acc_Y=0, acc_Z=0;
	static uint8_t rf, gf, bf; int k;
	int i;
	//static uint8_t data[6];
	static int16_t temp[3];
	SET_BIT(DEBUG_TASK_MOTION_SENSOR);
	flag++;
	switch(next_state_task)
	{
		case S1: 
						//check for device
						// need to add message thing 
						//update RTCS
						if(g_I2C_Msg.Status==IDLE)
						{
							g_I2C_Msg.Command=READ;
							g_I2C_Msg.Dev_adx=MMA_ADDR;
							g_I2C_Msg.Reg_adx=REG_WHOAMI;
							g_I2C_Msg.Data_count=1;
							//call RTCS  
							
							#if !Cond_Exe
							RTCS_Add_Task(TASK_I2C_SERVER_FSM,1, TICKS(TASK_I2C_SERVER_FSM_FREQ_HZ));
							#endif
							
							next_state_task=S2;
							break;
						}
						next_state_task=S1;
						#if !Cond_Exe 
								RTCS_Add_Task(TASK_MOTION_SENSOR_FSM, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
						#endif 
						break;
		case S2:
						if(g_I2C_Msg.Status==READ_COMPLETE)
						{
							if (g_I2C_Msg.Data[0] != WHOAMI)	{
							next_state_task=S4;
							return_variable=0;
							g_I2C_Msg.Status=IDLE;
							g_I2C_Msg.Command=NONE;
							Control_RGB_LEDs(1,0,0);
							//maybe have a global variable
							next_state_task=S2;
								#if !Cond_Exe 
								RTCS_Add_Task(TASK_MOTION_SENSOR_FSM, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
						#endif 
							break;
							//return 0; // error code
						}
							else
								{ g_I2C_Msg.Status=IDLE;
								next_state_task=S3;
								}
						}
						#if !Cond_Exe 
								RTCS_Add_Task(TASK_MOTION_SENSOR_FSM, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
						#endif 
							break;
						
			
							// can break here maybe
		case S3:	//set active mode, 14 bit samples, 2g full scale, low noise and 800 Hz ODR 
						 if(g_I2C_Msg.Status==IDLE)
						{
							g_I2C_Msg.Command=WRITE;
							g_I2C_Msg.Dev_adx=MMA_ADDR;
							g_I2C_Msg.Reg_adx=REG_CTRL1;
							g_I2C_Msg.Data_count=1;
							// write data over here
							g_I2C_Msg.Data[0]=0x05;
							next_state_task=S4;
							return_variable=1;
							#if !Cond_Exe
							RTCS_Add_Task(TASK_I2C_SERVER_FSM,1, TICKS(TASK_I2C_SERVER_FSM_FREQ_HZ));
							#endif
							//call RTCS
							break;
						}
						else
						next_state_task=S3;
						#if !Cond_Exe 
								RTCS_Add_Task(TASK_MOTION_SENSOR_FSM, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
						#endif 
						break;
						// call server
		case S4:
						//set global variable
						if(g_I2C_Msg.Status==WRITE_COMPLETE)
						{
							g_I2C_Msg.Status=IDLE;
							g_I2C_Msg.Command=NONE;
							next_state_task=S5;
							Control_RGB_LEDs(0,0,0);
							#if !Cond_Exe 
								RTCS_Add_Task(TASK_MOTION_SENSOR_FSM, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
						#endif 
							break;
						}
						next_state_task=S4;
						#if !Cond_Exe 
								RTCS_Add_Task(TASK_MOTION_SENSOR_FSM, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
						#endif 
						break;
						//return 1;
		case S5:
						
						//function call over here 
						if(g_I2C_Msg.Status==IDLE)
						{
							g_I2C_Msg.Command=READ;
							g_I2C_Msg.Dev_adx=MMA_ADDR;
							g_I2C_Msg.Reg_adx=REG_XHI;
							g_I2C_Msg.Data_count=6;
							for(k=0;k<6;k++)
							g_I2C_Msg.Data[k]=0;
							//call RTCS  
							next_state_task=S6;
							#if !Cond_Exe
							RTCS_Add_Task(TASK_I2C_SERVER_FSM,1, TICKS(TASK_I2C_SERVER_FSM_FREQ_HZ));
							#endif
							break;
						}
						#if !Cond_Exe 
								RTCS_Add_Task(TASK_MOTION_SENSOR_FSM, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
						#endif 
						next_state_task=S5;
						break;
		case S6: 
						if(g_I2C_Msg.Status==READ_COMPLETE)
						{
							g_I2C_Msg.Status=IDLE;
							g_I2C_Msg.Command=NONE;
							for ( i=0; i<3; i++ ) {
							temp[i] = (int16_t) ((g_I2C_Msg.Data[2*i]<<8) | g_I2C_Msg.Data[2*i+1]);
							}

						// Align for 14 bits
						acc_X = temp[0]/4;
						acc_Y = temp[1]/4;
						acc_Z = temp[2]/4;
						rf = abs(prev_acc_X - acc_X) > ACC_SENSITIVITY ? 1 : 0;
						gf = abs(prev_acc_Y - acc_Y) > ACC_SENSITIVITY ? 1 : 0;
						bf = abs(prev_acc_Z - acc_Z) > ACC_SENSITIVITY ? 1 : 0;
						//if(bf&&!rf&&!gf)
							//	bf=0;
						Control_RGB_LEDs(rf, gf, bf);
						#if !Cond_Exe 
								RTCS_Add_Task(TASK_MOTION_SENSOR_FSM, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ));
								RTCS_Clear_Task_Releases(TASK_MOTION_SENSOR_FSM);
								RTCS_Decrement(TASK_MOTION_SENSOR_FSM);	// Run periodically
						#endif 
							next_state_task=S7;
							break;
						}
						#if !Cond_Exe 
								RTCS_Add_Task(TASK_MOTION_SENSOR_FSM, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
						#endif 
						next_state_task=S6;
						break;
						
			
	case S7:	Control_RGB_LEDs(0, 0, 0);	
						next_state_task=S8;
						#if !Cond_Exe 
								
								RTCS_Add_Task(TASK_MOTION_SENSOR_FSM, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ));
								RTCS_Clear_Task_Releases(TASK_MOTION_SENSOR_FSM);
								RTCS_Decrement(TASK_MOTION_SENSOR_FSM);
								RTCS_Decrement(TASK_MOTION_SENSOR_FSM);								// Run periodically
						#endif
						break;
	case S8:	
						prev_acc_X = acc_X;
						prev_acc_Y = acc_Y;
						prev_acc_Z = acc_Z;
						next_state_task=S5;
						#if !Cond_Exe 
								RTCS_Add_Task(TASK_MOTION_SENSOR_FSM, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
						#endif 	 
						break;
						
		default:
					next_state_task=S5;
					break;
	}
	
	CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);
}
	/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	
	Init_RGB_LEDs();
	Init_Debug_Signals();
	Init_Config_Signals();
	
	Control_RGB_LEDs(1, 1, 0);								/* yellow: starting up */
	i2c_init();																/* init i2c	*/
	Delay(200);
	next_state_task=S1;
	Control_RGB_LEDs(0, 0, 0);							
	RTCS_Init(SCHED_FREQ_HZ);
	if(!(PTE->PDIR & (MASK(CONFIG1_POS))))
	{
			Control_RGB_LEDs(0,1,0);
		Delay(200);
		Control_RGB_LEDs(0, 0, 0);
	#if Cond_Exe
		RTCS_Add_Task(TASK_MOTION_SENSOR_FSM, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
		RTCS_Add_Task(TASK_I2C_SERVER_FSM,1, TICKS(TASK_I2C_SERVER_FSM_FREQ_HZ));
	#else 
		RTCS_Add_Task(TASK_MOTION_SENSOR_FSM, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
	#endif 
	}
	else
	{
	Control_RGB_LEDs(1,0,0);
		Delay(200);
		if (!init_mma()) {												/* init mma peripheral */
		Control_RGB_LEDs(1, 0, 0);							/* Light red error LED */
		while (1)																/* not able to initialize mma */
			;
	}
	Control_RGB_LEDs(0, 0, 0);							
	
	RTCS_Add_Task(Task_Motion_Sensor, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
	}
	RTCS_Run_Scheduler();
	
}
		