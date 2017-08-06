typedef void(*idle_callback)(void);

void control(idle_callback idle_cb);
float clip_ff(float motorin, int number);
float motorfilter( float motorin ,int number);
























