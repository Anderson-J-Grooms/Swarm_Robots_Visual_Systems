#include <rc_usefulincludes.h>
// main roboticscape API header
#include "udp.h"
#include <roboticscape.h>
#include <time.h>
#include <stdlib.h>
#include <rc/motor.h> 
#include "sensor_IR.h"		// Comment this out and uncomment the ultrasonic header file
// #include "sensor_ULTRASONIC.h"	// Comment this out and uncomment the ir header file

int pin1 = 1;
int pin2 =  8;
int frequency = 50;
char ip_target1[] = "172.25.53.121";
int msg_signal;
int ang_offset;

// Time
int echo = 0, previousEcho = 0, lowHigh = 0, highLow = 0;
long long int startTime = 0, stopTime = 0, difference = 0;
double distance = 0;
int count = 0;
// Ultrasonic Setup
int echoPin = 2;
int echoChip = 3;
int trigPin = 1;
int trigChip = 3;
// Setup timer v2
struct timeval timer_usec;
long long int timestamp_usec_start;
long long int timestamp_usec_end;

void forward(int pin1, int pin2, int frequency) {
    rc_servo_power_rail_en(1);
    rc_servo_send_pulse_normalized(pin1, -1);
    //rc_usleep(100000/30);
    rc_servo_send_pulse_normalized(pin2, 1);
    rc_usleep(2000);
    //rc_usleep(20000/frequency);
}
void stop(int pin1, int pin2, int frequency) {
    rc_servo_send_pulse_normalized(pin1, 0);
    rc_servo_send_pulse_normalized(pin2, 0);
    //rc_servo_power_rail_en(1);
}
void turn_right(int pin1, int pin2, int frequency) {
    rc_servo_power_rail_en(1);
    rc_servo_send_pulse_normalized(pin1, 1);
    rc_servo_send_pulse_normalized(pin2, 1);
    rc_usleep(100000/frequency);
}
void turn_left(int pin1, int pin2, int frequency) {
    rc_servo_power_rail_en(1);
    rc_servo_send_pulse_normalized(pin1, -1);
    rc_servo_send_pulse_normalized(pin2, -1);
    rc_usleep(100000/frequency);
}
double ts(double s) {
	double mu = 221.212 * s + 20.8466;
	double sigma = 3;
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
double fs(double s) {
	double mu = 22.864 * s + 1.3405;
	double sigma = 0.15;
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
double as(double s) {
        double mu = 4.38 * s + 1.3874;
        double sigma = 2;	
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
uint64_t angletos(double ang) {
	uint64_t re = (uint64_t)((ang - 20.84)*1000000000/237.5288752);
	//printf("%lld\n", re);
	return re;
}
double ir() {
	double measureParam_k1 = 11955.224610613135;
	double measureParam_k2 = -0.9738407600162202;
	long data =  rc_adc_read_raw(0);
	double returnValue = pow((data/measureParam_k1),(1/(measureParam_k2)));
	return returnValue;
}
double ir_bayesian() {
	const int iteration = 10;
	double *prob;
	for (int i = 0; i < iteration; i++) {
		long data =  rc_adc_read_raw(0);
		prob = make_prediction(data, 0);
	}
	int maxState = 0;
	for (int i = 0; i < state_size; i++) {
		if (prob[i] > prob[maxState]) {
			maxState = i;
		}
	}
	double state[591];
	double num = 1;
	for (int i = 0; i < 591; i++){
		state[i] = num;
		num += 0.1;
	}
       	//printf("predicted %f\n", state[maxState]);	
	return state[maxState];
}
void sttime(int ms) {
	uint64_t t = rc_nanos_since_epoch() + ms*1000000;
	while (rc_nanos_since_epoch() < t) {
		stop(1,8,50);
	}
}
void fwtime(int ms) {
	uint64_t t = rc_nanos_since_epoch() + ms*1000000;
	while (rc_nanos_since_epoch() < t) {
		forward(1,8,50);
	}
}
void trtime(int ms) {
	uint64_t t = rc_nanos_since_epoch() + ms*1000000;
	while (rc_nanos_since_epoch() < t) {
		turn_right(1,8,50);
	}
}
int main (int argc, char *argv[]) {
    // always initialize cape library first
    udp_init();
    rc_adc_init();
    uint8_t signal = 0;
    mavlink_message_t msg;
  
    if (rc_servo_init()) {
            fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?");
            return -1;
    }
    
    rc_set_state(RUNNING);
    stop(1,8,50);
    while (rc_get_state() != EXITING) {
        if (rc_get_state() == RUNNING) {
	    uint64_t target;
	    uint64_t a = angletos(115);
	    uint64_t fives = rc_nanos_since_epoch() + 5000000000;
	    double reading;
		if (read_from_socket(ip_target1, &msg)) {
			uint64_t start = rc_nanos_since_epoch();
			//printf("%lld", start);
				reading = ir_bayesian();
				//reading = ir();
				while (reading > 10) {
					fwtime(100);
					reading = ir_bayesian();
					//reading = ir();
				}
				//printf("see obstacle %f\n", ir());
				//stop(1, 8, 50);
				//printf("turn 90\n");
				//target = rc_nanos_since_epoch() + 400000000;
				//printf("%lld", angletos(90));
				//while (rc_nanos_since_epoch() < target) {
				//	turn_right(1, 8, 50);
					//printf("turn\n");
				//}
				//printf("%lld", rc_nanos_since_epoch() - target);
				//sttime(50);
				stop(1,8,50);
				trtime(350);
				//printf("%lld", a);
				//break;
				//while (ir_bayesian() > 10 && rc_nanos_since_epoch() < fives) {
				//	forward(1, 8, 50);
				//}
				//sttime(50);
				stop(1,8,50);
				reading = ir_bayesian();
				//reading = ir();
				while (reading > 10) {
					fwtime(100);
					//reading = ir();
					reading = ir_bayesian();
				}
				//target = rc_nanos_since_epoch() + 450000000;
				//while (rc_nanos_since_epoch() < target) {
				//	turn_right(1, 8, 50);
				//}
				//sttime(50);
				stop(1,8,50);
				trtime(350);
				//stop(1, 8, 50);
				//sttime(50);
				stop(1,8,50);
				//target = rc_nanos_since_epoch() + 1000000000;
				//while (rc_nanos_since_epoch() < target && rc_nanos_since_epoch() < fives) {
				//	forward(1, 8, 50);
				//}
				fwtime(1000);
		//	}
			printf("%lld", rc_nanos_since_epoch() - start);
			//break;
	    	}
	}
    }
    rc_adc_cleanup();
}
