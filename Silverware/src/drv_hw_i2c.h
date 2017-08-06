

#include <inttypes.h>


typedef void(*idle_callback)(void);


void hw_i2c_init( void);
int hw_i2c_readdata( int reg, int *data, int size, idle_callback idle_cb );
int hw_i2c_readreg( int reg );
void hw_i2c_writereg( int reg ,int data);
			











