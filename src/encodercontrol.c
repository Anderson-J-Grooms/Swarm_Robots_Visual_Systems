#include <stdio.h>
#include <stdlib.h>
#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include <rc/adc.h>
#include <rc/time.h>
/**
 * @brief A control algorithm implementing a PID controller. The functions takes in the constants for the PID variables and 
 * pointers to the final speeds after error correction.
 * 
 * @param error Integer value representing the difference in the motor wheels. If error is positive then the left wheel is 
 * moving faster than the right wheel. The opposite is true if the error is negative.
 * @param p Proportional value
 * @param i Intergal value
 * @param d differential value
 * @param leftSpeedIn Speed of the left motor before error correction
 * @param rightSpeedIn Speed of the right motor before error correction
 * @param leftSpeedOut Speed of the Left motor after error correction
 * @param rightSpeedOut Speed of the Left motor after error correction
 */
void controlAlgorithm(const int *error, const int16_t p, const int16_t i, const int16_t d, const float *leftSpeedIn, const float *rightSpeedIn, int *leftSpeedOut, int *rightSpeedOut) 
{
    // Very Simple Control
    // This assumes that the max (p*error) will be 75. New range [1.5,0]
    float p_delta = (p * &error) / 50;
    leftSpeedOut -= (&leftSpeedIn > 0.0 && &leftSpeedIn <= 1.5f) ? p_delta : 0;
    rightSpeedOut +=  (&rightSpeedIn > 0.0 && &rightSpeedIn <= 1.5f) ? p_delta : 0;
}


enum state 
{
    LOW,
    HIGH
};
/**
 * @brief Read the values from the ADC. This function will need to be called at a "Sampling Rate" by the caller function to ensure
 * the changes from the encoder are accurate.
 * 
 * ADC sample time **may be** 125 ns
 *
 * @param leftWheelCount pointer to the count of the right encoder changes
 * 
 * @param rightWheelCount pointer to the count of the right encoder changes
 */
void readADC(int *leftWheelCount, int *rightWheelCount, enum state *leftState, enum state *rightState)
{
    // Move to global scope
    // Analog input pins
    int AIN0 = 3;
    int AIN1 = 4;
    // Threshold reading from BeagleBone
    long threshold = 3500; // ***Needs Testing***

    // Read values from ADC
    enum state leftReading = (rc_adc_read_raw(AIN0) >= threshold) ? HIGH : LOW;
    enum state rightReading = (rc_adc_read_raw(AIN1) >= threshold) ? HIGH : LOW;

    // Compare to previous values
    // if the new values read from the adc are on the opposite side of the threshold increment the count for that wheel
    if (leftReading != &leftState)
    {
        leftWheelCount++;
        leftState = &leftReading;
    }
    if (rightReading != &rightState)
    {
        rightWheelCount++;
        rightState = &rightReading;
    }
}

int main(int argc, char *argv[]) 
{
    int time;
    if (argc > 1) {
	time = argv[1];
    }
    else {
	time = 100;
    }
    //int time = (argc > 1) ? argv[1] : 100;
    enum state leftState = LOW, rightState = LOW;
    int leftWheelCount = 0, rightWheelCount = 0;
    while(time > 0)
    {
        readADC(&leftWheelCount, &rightWheelCount, &leftState, &rightState);
        printf("Left Wheel Count:  %i \n Right Wheel Count: %i", leftWheelCount, rightWheelCount);
        time--;
    }
}
