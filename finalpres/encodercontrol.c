#include <stdio.h>
#include <stdlib.h>
#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <time.h>

enum state 
{
    LOW,
    HIGH
};

struct wheel_control
{
    int pin;                        // Pin to send motor speed to

    clock_t time_0;                 // Past Time
    clock_t time_1;                 // Current Time

    int16_t threshold;

    enum state current_state;       // Current State of Optical Sensor [High, LOW]

    double p, i, d;                  // PID values for the controller

    double i_error;                  // Incremental error for creating the integral of the error curve

    double prev_error;               // Previous error to get instantaneous change for derivative error;

    double m_speed;                  // Measured Speed. Gets updated by get_speed().
    double i_speed;                  // Ideal Speed. Gets set by g.

    double motor_setting;           // Setting the PWM for the motor. Gets set by the PID controller.

    double error;                   // The error is the differenct between the ideal and measured speeds; err = i_speed - m_speed.
};

/**
 * @brief A convenience function for converting a speed in cm/s to the frequency for setting the PWM on the motor
 * 
 * @param speed The speed we are trying to match to a frequency
 * 
 * @param pin If we are writing to the left wheel we need to flip the sign
 * 
 * @return The frequency to be sent to the motor
 */
double convert_speed(double speed, int pin)
{
    const double max_speed = 3.0; // Max speed is unknown.

    const double max_freq = 1.5;

    if(pin == 1)
        speed *= -1;

    return ((speed / max_speed) * max_freq)
}

/**
 * @brief A control algorithm implementing a PID controller. The functions takes in the constants for the PID variables and 
 * pointers to the final speeds after error correction.
 * 
 * @param wheel pointer to the struct containing control data.
 */
void pid_control(struct wheel_control *wheel) 
{
    // PID control
    double proportional = wheel->p * wheel->error;
    double intregral = wheel->i * wheel->i_error;
    double derivative = wheel->d * ((wheel->error - wheel->prev_error) / ((wheel->time_1 - wheel->time_0) / CLOCKS_PER_SEC));

    wheel->motor_setting += convert_speed(proportional + intregral + derivative);

    // set motor to new speed
    printf("Wheel Motor current setting %f\n", wheel->motor_setting);
    rc_servo_send_pulse_normalized(wheel->pin, wheel->motor_setting);
}

void get_speed(struct wheel_control *wheel)
{
    const double distance = 1.5; // 1.5 cm is how far the wheel travels

    double delta_clock = wheel->time_1 - wheel->time_0;
    double seconds = delta_clock / CLOCKS_PER_SEC;

    wheel->m_speed = distance / seconds;  // Unit of speed is cm/s 
}

void get_error(struct wheel_control *wheel)
{
    wheel->prev_error = wheel->error;
    wheel->i_error += wheel->error * (wheel->time_1 - wheel->time_0) / CLOCKS_PER_SEC;
    wheel->error = wheel->i_speed - wheel->m_speed; // Error is equal to the difference between the ideal and measured speeds
}

/**
 * @brief Read the values from the ADC. This function will need to be called at a "Sampling Rate" by the caller function to ensure
 * the changes from the encoder are accurate.
 * 
 * ADC sample time **may be** 125 ns
 *
 * @param left_wheel pointer to the struct containing control data.
 * 
 * @param left_wheel pointer to the struct containing control data.
 * 
 * @return True if there was a change and false if else.
 */
bool read_ADC(struct wheel_control *left_wheel, struct wheel_control *right_wheel)
{
    // Move to global scope
    // Analog input pins
    int AIN0 = 3;
    int AIN1 = 4;

    bool change = false; // Return so we know if there were any changes.

    // Read values from ADC
    enum state leftReading = (rc_adc_read_raw(AIN0) >= left_wheel->threshold) ? HIGH : LOW;
    enum state rightReading = (rc_adc_read_raw(AIN1) >= right_wheel->threshold) ? HIGH : LOW;
    // Compare to previous values
    // if the new values read from the adc are on the opposite side of the threshold increment the count for that wheel
    if (leftReading != left_wheel->current_state)
    {
        left_wheel->time_0 = left_wheel->time_1;
        left_wheel->time_1 = clock();

        left_wheel->current_state = leftReading;

        get_speed(left_wheel); // Calculates speed
        get_error(left_wheel); // updates error

        change = true;
    }

    if (rightReading != right_wheel->current_state)
    {
        right_wheel->time_0 = right_wheel->time_1;
        right_wheel->time_1 = clock();

        right_wheel->current_state = rightReading;

        get_speed(right_wheel);
        get_error(right_wheel);

        change = true;
    }

    return change;
}

void init_pid(struct wheel_control *left_wheel, struct wheel_control *right_wheel)
{
    // These values are not correct nor good attempts at being close
    left_wheel->p = 0.9;
    left_wheel->i = 0.4;
    left_wheel->d = 0.2;

    left_wheel->p = 1.0;
    left_wheel->i = 0.6;
    left_wheel->d = 0.2;

    left_wheel->threshold = 2200;
    right_wheel->threshold = 2200;

    left_wheel->pin = 1;
    right_wheel->pin = 8;
}

void set_speed(double speed, struct wheel_control *wheel)
{
    clock_t time = clock();
    wheel->time_1 = time;

    // Get PWM setting from speed
    double motor_setting = convert_speed(speed, wheel->pin);

    rc_servo_send_pulse_normalized(wheel->pin, motor_setting);

    wheel->i_speed = speed;
    wheel->m_speed = speed;
    wheel->motor_setting = motor_setting;
}



int main(int argc, char *argv[]) 
{
    struct wheel_control left_wheel;
    struct wheel_control right_wheel;

    // We need to ensure that the structs have the data for each respective wheel.
    init_pid(&left_wheel, &right_wheel); // Sets the pid values for each wheel and the time_0 for each wheel;
    rc_servo_init();
    rc_adc_init();

    long long int time;
    if (argc > 1) 
    {
	    time = atoi(argv[1]);
    }
    else 
    {
	    time = 100000000000; // 100,000,000,000 == 100 seconds give or take
    }

    // We need an initial speed to set the motors to and a goal to calculate error from
    set_speed(2, &left_wheel);
    set_speed(2, &right_wheel);
    
    // Poll The adc and wait for a change
    while(time > 0)
    {
        if(read_ADC(&left_wheel, &right_wheel)) // Reads the ADC and updates the state and time of that change
        {
            pid_control(&left_wheel); // Update motor PWM setting
            pid_control(&right_wheel);
        }
	    time--;
    }
}

/**
 * @brief TODO
 * 1. The encoders need to be remounted and we have to find their respective thresholds again.
 * 
 * 2. The wheels need to be set to full speed and their top speeds recalculated.
 * 
 * 3. The Threshold and top speed values need to be set in the convert_speed() and read_ADC() functions.
 * 
 * 4. Radius of the wheel needs to be measured and the length of travel calculated. Most likely the problem 
 *  I was having with the speed is due to the wheel not being a good code wheel and we can make it more uniform. 
 * 
 */

