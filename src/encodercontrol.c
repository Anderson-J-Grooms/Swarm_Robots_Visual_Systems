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

    int16_t threshold;              // Threshold in the middle of the two readings from the wheel encoders.

    enum state current_state;       // Current State of Optical Sensor [High, LOW]

    double p, i, d;                 // PID values for the controller

    double i_error;                 // Incremental error for creating the integral of the error curve

    double prev_error;              // Previous error to get instantaneous change for derivative error;

    double m_speed;                 // Measured Speed. Gets updated by get_speed().
    double i_speed;                 // Ideal Speed. Gets set by set_speed().

    double motor_setting;           // Setting the PWM for the motor. Gets set by the PID controller.

    double error;                   // The error is the difference between the ideal and measured speeds; err = i_speed - m_speed.
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

    // Pin 1 is connected to the right wheel which is facing the oposite direction
    // so we need to flip the speed so it goes in the correct direction.
    if(pin == 1) 
        speed *= -1;

    return ((speed / max_speed) * max_freq);
}

/**
 * @brief Set the motor PWM setting
 * 
 * @param wheel The wheel to set
 * @return int Returns 0 on success and 1 if failure
 */
int set_motor(struct wheel_control *wheel)
{
    return rc_servo_send_pulse_normalized(wheel->pin, wheel->motor_setting);
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

    wheel->motor_setting += convert_speed(proportional + intregral + derivative, wheel->pin);
}

/**
 * @brief Get the speed from the current delta time and the measured distance that it travels.
 * The distance is contant and based off of two state changes from the wheel encoder.
 * 
 * @param wheel The wheel for calculating the speed.
 */
void get_speed(struct wheel_control *wheel)
{
    const double distance = 2.17; // 2.17 cm is how far the wheel travels in a single state

    double delta_clock = wheel->time_1 - wheel->time_0;
    double seconds = delta_clock / CLOCKS_PER_SEC;

    wheel->m_speed = distance / seconds;  // Unit of speed is cm/s 
}

/**
 * @brief Get the error from the current measured speed and the ideal speed set in set_speed().
 * 
 * @param wheel The wheel for calculating the error.
 */
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
 * @return True if the speed and error were updated and false if they were not.
 */
bool read_ADC(struct wheel_control *left_wheel, struct wheel_control *right_wheel)
{
    // Analog input pins
    int AIN0 = 3;
    int AIN1 = 4;

    bool update = false; // Return so we know if there were any changes.

    // Read values from ADC
    enum state leftReading = (rc_adc_read_raw(AIN0) >= left_wheel->threshold) ? HIGH : LOW;
    enum state rightReading = (rc_adc_read_raw(AIN1) >= right_wheel->threshold) ? HIGH : LOW;
    // Compare to previous values
    // if the new values read from the adc are on the opposite side of the threshold and we have gone one unit in distance
    // update the speed and error
    if (leftReading != left_wheel->current_state)
    { 
	    // Debounce in a way
        rc_usleep(10);
	    leftReading = (rc_adc_read_raw(AIN0) >= left_wheel->threshold) ? HIGH : LOW;
	
        if (leftReading != left_wheel->current_state)
	    {
		    left_wheel->current_state = leftReading;

        	left_wheel->time_0 = left_wheel->time_1;
       		left_wheel->time_1 = clock();

        	get_speed(left_wheel); // Calculates speed
        	get_error(left_wheel); // updates error
        	update = true;
	    }
    }

    if (rightReading != right_wheel->current_state)
    {
     	// Debounce in a way
        rc_usleep(10);
	    rightReading = (rc_adc_read_raw(AIN1) >= right_wheel->threshold) ? HIGH : LOW;
	
        if (rightReading != right_wheel->current_state)
	    {
		    right_wheel->current_state = rightReading;
        
        	right_wheel->time_0 = right_wheel->time_1;
        	right_wheel->time_1 = clock();

        	get_speed(right_wheel); // Calculates speed
        	get_error(right_wheel); // updates error
        	update = true;
       	}
     }

    return update;
}

/**
 * @brief A special function for intializing all the controller values.
 * 
 * @param left_wheel Pointer to the control struct for the left wheel.
 * @param right_wheel Pointer to the control struct for the right wheel.
 */
void init_pid(struct wheel_control *left_wheel, struct wheel_control *right_wheel)
{
    // Analog input pins
    int AIN0 = 3;
    int AIN1 = 4;
    
    // These values are not correct nor good attempts at being close
    left_wheel->p = 0.9;
    left_wheel->i = 0.4;
    left_wheel->d = 0.2;

    right_wheel->p = 1.0;
    right_wheel->i = 0.6;
    right_wheel->d = 0.2;

    // This is the value that will determine whether the encoder is sending a high or low signal
    left_wheel->threshold = 2700;
    right_wheel->threshold = 2700;

    // The integral of the area before we start is zero
    left_wheel->i_error = 0.0;
    right_wheel->i_error = 0.0;

    // Pins for writing the PWM setting the motors
    left_wheel->pin = 8;
    right_wheel->pin = 1;

    // Current states of the wheel encoders for initializing the variables
    left_wheel->current_state = (rc_adc_read_raw(AIN0) >= left_wheel->threshold) ? HIGH : LOW;
    right_wheel->current_state = (rc_adc_read_raw(AIN1) >= right_wheel->threshold) ? HIGH : LOW;
}

/**
 * @brief Initialize the ideal and measured speed to whatever speed we want for a given wheel.
 * This method will also start the motor and should be followed by set_motor() to keep it running.
 * 
 * @param speed Speed in cm/s to be converted to a PWM frequency for the motor.
 * @param wheel The wheel for setting the speed.
 */
void set_speed(double speed, struct wheel_control *wheel)
{
    // Since we are starting the motor we need to record this as the current time.
    clock_t time = clock();
    wheel->time_0 = time; // time_0 should never be accessed before it gets set to the correct value, but we initialize to be safe.
    wheel->time_1 = time;

    // Get PWM setting from speed
    double motor_setting = convert_speed(speed, wheel->pin);

    printf("Pin: %d, Setting: %f\n", wheel->pin, motor_setting);
    printf("Success: %d\n", rc_servo_send_pulse_normalized(wheel->pin, motor_setting));

    wheel->i_speed = speed;
    wheel->m_speed = speed;
    wheel->motor_setting = motor_setting;
}

/**
 * @brief Function useful for debugging which dumps a few of the useful parameters to standard output.
 * 
 * @param wheel Wheel to get information from.
 */
void output_data(struct wheel_control *wheel)
{
    printf("\n======================\n");

    printf("%s Wheel:\n", (wheel->pin == 1 ? "Left" : "Right"));
    printf("Measured Speed: %f\n", wheel->m_speed);
    printf("Ideal Speed: %f\n", wheel->i_speed);
    printf("Motor Setting: %f\n", wheel->motor_setting);
    printf("Timing:\n\ttime[0] = %ld, time[1] = %ld\n\tdelta_t = %ld", wheel->time_0, wheel->time_1, wheel->time_1 - wheel->time_0);

    printf("\n======================\n");
}



int main(int argc, char *argv[]) 
{
    // Analog input pins
    int AIN0 = 3;
    int AIN1 = 4;
    
    // Create the structs for controlling the motors
    struct wheel_control left_wheel;
    struct wheel_control right_wheel;

    // Initialize the ADC and SERVO
    rc_adc_init();
    rc_servo_init();
    rc_servo_power_rail_en(1);

    // We need to ensure that the structs have the data for each respective wheel.
    init_pid(&left_wheel, &right_wheel); // Sets the pid values for each wheel and the time_0 for each wheel;

    // Set time for the loop to run
    //    long long int time;
    //    if (argc > 1) 
    //    {
    //	    time = atoi(argv[1]);
    //    }
    //    else 
    //    {
    //	    time = 100000000000; // 100,000,000,000 == 100 seconds give or take
    //    }
    //
    // We need an initial speed to set the motors to and a goal to calculate error from
    // set_speed(2, &left_wheel);
    // set_speed(2, &right_wheel);
    for(;;)
    {
        read_ADC(&left_wheel, &right_wheel);
	    printf("Left Wheel Reading %d, State: %d", rc_adc_read_raw(AIN0), left_wheel.current_state);
        printf(" Right Wheel Reading %d, State: %d\n", rc_adc_read_raw(AIN1), right_wheel.current_state);
       // set_motor(&left_wheel);
       // set_motor(&right_wheel);
       // rc_usleep(1000000/50);    
    }
    
    // CONFIGURATION LOOP
    //for(;;)
    //{
        // Use to find threshold and ensure encoders are labeled properly in code
        // printf("Left Wheel Reading: %d\nRight Wheel Reading: %d\n", rc_adc_read_raw(AIN0), rc_adc_read_raw(AIN1));

        // Needs the Threshold set first
        // if(readADC(&left_wheel, &right_wheel))
        // {
        //     output_data(&left_wheel);
        //     output_data(&right_wheel);
        // }
    //}

    // CONTROL LOOP
    // // Poll The adc and wait for a change
    // while(time > 0)
    // {
    //     if(read_ADC(&left_wheel, &right_wheel)) // Reads the ADC and updates the state and time of that change
    //     {
    //         pid_control(&left_wheel); // Update motor PWM setting
    //         pid_control(&right_wheel);
    //     }

    //     output_data(&left_wheel);
    //     output_data(&right_wheel);
	//     time--;
    // }
    rc_servo_cleanup();
}

/**
 * @brief TODO
 * 
 * 1. Radius of the wheel needs to be measured and the length of travel calculated. we can make it more uniform. 
 *  Diameter of wheel: 6.9 cm
 *  Circumference of wheel: 6.9cm * pi = 21.7 cm
 *  Distance Traveled in one state (defined by the transition in the wheel encoder readings): 21.7cm / 10 = 2.17 cm
 * 
 * 2. The encoders need to be remounted and we have to find their respective thresholds again.
 * 
 * 3. The wheels need to be set to full speed and their top speeds recalculated. The wheels may need to keep track of their top speeds. This will be important for maxing out the slow wheel and not the fast wheel. 
 *  They will need to have different ranges for outputing the motor_setting in convert_speed().
 * 
 * 4. The Threshold and top speed values need to be set in the convert_speed() and read_ADC() functions.
 * 
 * 5. Debug it when nothing works.
 */
