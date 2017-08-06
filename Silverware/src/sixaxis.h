typedef void(*idle_callback)(void);

void sixaxis_init( void);
int sixaxis_check( void);
void sixaxis_read( int gyro_only, idle_callback idle_cb);
void gyro_cal( void);

void acc_cal(void);




