#include <stdint.h>

#define I2C_M_START 	I2C0->C1 |= I2C_C1_MST_MASK
#define I2C_M_STOP  	I2C0->C1 &= ~I2C_C1_MST_MASK
#define I2C_M_RSTART 	I2C0->C1 |= I2C_C1_RSTA_MASK

#define I2C_TRAN			I2C0->C1 |= I2C_C1_TX_MASK
#define I2C_REC				I2C0->C1 &= ~I2C_C1_TX_MASK

#define BUSY_ACK 	    while(I2C0->S & 0x01)
#define TRANS_COMP		while(!(I2C0->S & 0x80))
// #define I2C_WAIT			while((I2C0->S & I2C_S_IICIF_MASK)==0) {} \
                                 I2C0->S |= I2C_S_IICIF_MASK;
#define I2C_WAIT 			i2c_wait();

#define NACK 	        I2C0->C1 |= I2C_C1_TXAK_MASK
#define ACK           I2C0->C1 &= ~I2C_C1_TXAK_MASK

#define LOCK_DETECT_THRESHOLD (2000)

#define MAX_I2C_DATA_BYTES 32

typedef enum {IDLE, READING, READ_COMPLETE, WRITING, WRITE_COMPLETE} I2C_STATUS_T;
typedef enum {NONE, READ, WRITE} I2C_COMMAND_T;

typedef struct {
	void (* Client)(void); // To let server release client when work is done
	I2C_COMMAND_T Command;
	I2C_STATUS_T Status;
	uint8_t Dev_adx;
	uint8_t Reg_adx;
	uint8_t Data_count;
	uint8_t Data[MAX_I2C_DATA_BYTES];
} I2C_MESSAGE_T;


void i2c_init(void);
void Task_I2C_Server(void);

int i2c_read_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count);
int i2c_write_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count);

void i2c_wait( void );

extern volatile I2C_MESSAGE_T g_I2C_Msg;




