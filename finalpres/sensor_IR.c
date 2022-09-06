/*
Copyright (c) 2022 Peter Jamieson (jamieson.peter@gmail.com)
and Bryan Van Scoy

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <string.h>
#include "sensor_IR.h"

/* globals */
// Number of sample
const int iteration = 10;
/*-------------------------------------------------------------------------
* (function: )
*-----------------------------------------------------------------------*/
//void* sensor_function_ULTRASONIC(sensor_t *sensor, agent_t *agent, double current_time)
//{
//	printf("Ultrasonic called\n");
//	return NULL;
//}

double sum_array(double array[], int num_elements)
{
	double sum = 0;
	for (int i = 0; i<num_elements; i++)
	{
		sum = sum + array[i];
	}
	return(sum);
}

void normalize_array(double array[], int num_elements)
{
	double sum = sum_array(array, num_elements);
	for (int i = 0; i<num_elements; i++)
	{
		array[i] = array[i] / sum;
	}
}

double* make_prediction(double reading, int restart) {
	// Variable to notify reading and 
	static int sampleCount = 0;
	static int init = 1;
	/* Storing finite number of state */
	double state[591];
	double num = 1;
	for (int i = 0; i < 591; i++) {
		state[i] = num;
		num += 0.1;
	}
	/* Array to store prediction probability */
	double en = 1.0 / (double)state_size;
	static double prob_arr[591];
	if ((init == 1) || (sampleCount >= iteration) || (restart == 1)) {
		sampleCount = 0;
		init = 0;
		for (int i = 0; i < state_size; i++) {
			prob_arr[i] = en;
		}
	}
	/* ===== Start Predicting ======== */
	static double mean_k1 = 11955.224610613135;
	static double mean_k2 = -0.9738407600162202;
	static double std_k1 = 5.574431613205327;
	static double std_k2 = 0.14072513618767238;

	for (int i = 0; i < state_size; i++) {
		double mean = mean_k1 * (pow(state[i], mean_k2));
		double std = std_k1 * pow(M_E, (std_k2*state[i]));
		double variance = pow((std), 2.0);
		double prob = ((1.0 / (std * sqrt(2.0 * M_PI))) * (pow(M_E, -1.0 * ((pow((reading - mean), 2.0)) / (variance * 2.0)))));
		prob_arr[i] = prob * prob_arr[i];
	}
	normalize_array(prob_arr, state_size);
	sampleCount++;
	return prob_arr;
}

double randnorm(double mu, double sigma)
{
	double U1, U2, W, mult;
	static double X1, X2;
	static int call = 0;

	if (call == 1)
	{
		call = !call;
		return (mu + sigma * (double)X2);
	}

	do
	{
		U1 = -1 + ((double)rand() / RAND_MAX) * 2;
		U2 = -1 + ((double)rand() / RAND_MAX) * 2;
		W = pow(U1, 2) + pow(U2, 2);
	} while (W >= 1 || W == 0);

	mult = sqrt((-2 * log(W)) / W);
	X1 = U1 * mult;
	X2 = U2 * mult;

	call = !call;

	return (mu + sigma * (double)X1);
}

double generate_data(double distance) {
	// Constant to calculate to mean and standard deviation from the given distance
	static double mean_k1 = 11955.224610613135;
	static double mean_k2 = -0.9738407600162202;
	static double std_k1 = 5.574431613205327;
	static double std_k2 = 0.14072513618767238;

	// Calculate the mean and std for the Gausian Distribution
	double mean = mean_k1 * (pow(distance, mean_k2));
	double std = std_k1 * pow(M_E, (std_k2*distance));

	// Generate the random numbers that follows the Gausian Distribution
	return randnorm(mean, std);
}

/*Geneate a set of data that follows a distribution with mean and standard deviation calculated from the distance*/
double* generate_data_set(double distance, int numberSample) {
	// Constant to calculate to mean and standard deviation from the given distance
	static double mean_a = 0.9581686069037303;
	static double mean_b = 0.7625882833237847;
	static double std_a = 0.009280269184555247;
	static double std_b = 0.48783070995817385;

	// Calculate the mean and std for the Gausian Distribution
	double mean = mean_a * distance + mean_b;
	double std = std_a * distance + std_b;

	// Generate the random numbers that follows the Gausian Distribution
	double *returnArr = malloc(sizeof(double) * numberSample);
	for (int i = 0; i < numberSample; i++) {
		returnArr[i] = randnorm(mean, std);
	}

	return returnArr;
}

void printArr(double arr[], int size) {
	printf("[");
	for (int i = 0; i < size; i++) {
		printf("%.2f ", arr[i]);
	}
	printf("]\n");
}
