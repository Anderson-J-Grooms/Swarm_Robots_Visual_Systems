#ifndef ENCODERCONTROL
#define ENCODERCONTROL

int set_motor(int pin, double motor_setting);
void init_control();
int get_state(int pin);
void clean_up_control();

#endif