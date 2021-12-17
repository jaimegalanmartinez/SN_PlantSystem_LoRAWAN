#include "mbed.h"

#ifndef _LOG_VALUES_SENSORS_H_
#define _LOG_VALUES_SENSORS_H_
/*
//Struct that stores all previous values in 1 hour to be processed later.
struct Log{
	//Temp
	float temperature_max;
	float temperature_min;
	float temperature_avg;
	float temperature_sum;
	//Humidity
	float humidity_max;
	float humidity_min;
	float humidity_avg;
	float humidity_sum;
	//Light
	float light_max;
	float light_min;
	float light_avg;
	float light_sum;
	//Moisture
	float moisture_max;
	float moisture_min;
	float moisture_avg;
	float moisture_sum;
	//Count colours
	uint16_t count_color_red;
	uint16_t count_color_blue;
	uint16_t count_color_green;
	//Accel
	float accel_x_max;
	float accel_x_min;
	float accel_y_max;
	float accel_y_min;
	float accel_z_max;
	float accel_z_min;
	//Num samples
	uint16_t num_samples_measures;
};
*/
/*
Initialize the log_values variable 
*/
//void initLog(Log *log_values);
/*
Updates the log_values variable with the new values recorded
*/
//void updateLog(Log *log_values,float temperature, float humidity, float light, float moisture, char dominant_color, float accel_values[]);
/**
* Calculate average sensors data
*/
//void calculate_average_sensors_data(Log *log_values);
/**
* Calculate dominant color from  logs colors count
*/
//char calculate_dominant_color_from_logs(Log log_values);

#endif