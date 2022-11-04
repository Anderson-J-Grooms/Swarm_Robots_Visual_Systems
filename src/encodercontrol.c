#include <stdio.h>
#include <stdlib.h>
#include <rc_usefulincludes.h>
#include <roboticscape.h>

/**
 * @brief A control algorithm implementing a PID controller. The functions takes in the constants for the PID variables and 
 * pointers to the final speeds after error correction.
 * 
 * @param p Proportional value
 * @param i Intergal value
 * @param d differential value
 * @param leftSpeedIn Speed of the left motor before error correction
 * @param rightSpeedIn Speed of the right motor before error correction
 * @param leftSpeedOut Speed of the Left motor after error correction
 * @param rightSpeedOut Speed of the Left motor after error correction
 */
void controlAlgorithm(int error, const int16_t p, const int16_t i, const int16_t d, const int leftSpeedIn, const int rightSpeedIn, int *leftSpeedOut, int *rightSpeedOut) 
{

}


enum state {
    LOW,
    HIGH
}

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
void readADC(int *leftWheelCount, int *rightWheelCount, state *leftState, state *rightState)
{
    // Move to global scopett
    // Analog input pins
    int AIN0 = 3;
    int AIN1 = 4;
    // Threshold reading from BeagleBone
    long threshold = 750; // Need testing

    // Read values from ADC
    state leftReading = [](){return rc_adc_read_raw(AIN0) >= threshold;};
    state rightReading = [](){return rc_adc_read_raw(AIN1) >= threshold;};

    // Compare to previous values
    // if the new values read from the adc are on the opposite side of the threshold increment the count for that wheel
    if (leftReading != leftState)
    {
        leftWheelCount++;
        leftState = &leftReading;
    }
    if (rightReading != rightState)
    {
        rightWheelCount++;
        rightState = &rightReading;
    }
}