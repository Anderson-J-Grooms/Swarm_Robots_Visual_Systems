#ifndef SENSOR_IR_H   
#define SENSOR_IR_H    /* prevents the file from being included twice. */
/* Including a header file twice causes all kinds */
/* of interesting problems.*/
#define state_size  591

double sum_array(double array[], int num_elements);
void normalize_array(double array[], int num_elements);
double* make_prediction(double reading, int restart);
double randnorm(double mu, double sigma);
double generate_data(double distance);
double* generate_data_set(double distance, int numberSample);
void printArr(double arr[], int size);


#endif /* SENSOR_IR_H */