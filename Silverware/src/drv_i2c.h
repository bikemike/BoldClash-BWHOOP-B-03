





typedef void(*idle_callback)(void);

void i2c_init( void);
int i2c_readdata( int reg, int *data, int size, idle_callback idle_cb);
int i2c_readreg( int reg );
void i2c_writereg( int reg ,int data);
			











