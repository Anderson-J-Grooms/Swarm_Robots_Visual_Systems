#include <stdio.h>
#include <stdlib.h>
#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <time.h>
#include <encodercontrol_export.h>

#define CHIP 1
#define IN_PIN0 25
#define IN_PIN1 17

/**
 * @brief Set the motor PWM setting. This should be called from the control loop to allow for pid_control to update the setting.
 * 
 * @param wheel The wheel to set
 * @return int Returns 0 on success and 1 if failure
 */
int set_motor(int pin, double motor_setting)
{
    return rc_servo_send_pulse_normalized(pin, motor_setting);
}

void init_control()
{
    rc_gpio_init(CHIP, IN_PIN0, GPIOHANDLE_REQUEST_INPUT);
    rc_gpio_init(CHIP, IN_PIN1, GPIOHANDLE_REQUEST_INPUT); 
    if(rc_servo_init() == -1)
    {
         printf("Error: Failed to initialized Servos.");
         return 1;
    }
    rc_servo_power_rail_en(1);
}

int get_state(int pin)
{
    return rc_gpio_get_value(CHIP, pin ? IN_PIN1 : IN_PIN0);
}

void clean_up_control()
{
    rc_servo_cleanup();
}
