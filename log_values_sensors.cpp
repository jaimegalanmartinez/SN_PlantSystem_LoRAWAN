#include "mbed.h"
#include "log_values_sensors.h"
/*EPC PlantSystem project
* @Authors: Jaime Galan Martinez
*						Victor Aranda Lopez
*/

/*
Initialize the log_values variable 
*/
void initLog(Log *log_values){
	log_values->temperature_max = -60;
	log_values->temperature_min = 200;
	log_values->temperature_avg = 0;
	log_values->temperature_sum = 0;
	
	log_values->humidity_max = 0;
	log_values->humidity_min = 100;
	log_values->humidity_avg = 0;
	log_values->humidity_sum = 0;
	
	log_values->light_max = 0;
	log_values->light_min = 100;
	log_values->light_avg = 0;
	log_values->light_sum = 0;
	
	log_values->moisture_max = 0;
	log_values->moisture_min = 100;
	log_values->moisture_avg = 0;
	log_values->moisture_sum = 0;
	
	log_values->count_color_red = 0;
	log_values->count_color_green = 0;
	log_values->count_color_blue = 0;
		
  log_values->accel_x_max = -99;
	log_values->accel_x_min = 99;
	log_values->accel_y_max = -99;
	log_values->accel_y_min = 99;
	log_values->accel_z_max = -99;
	log_values->accel_z_min = 99;
	
	log_values->num_samples_measures = 0;
}

/*
Updates the log_values variable with the new values recorded
*/
void updateLog(Log *log_values,float temperature, float humidity, float light, float moisture, char dominant_color, float accel_values[]){
	if(log_values->temperature_max < temperature)
		log_values->temperature_max = temperature;
	if(log_values->temperature_min > temperature)
		log_values->temperature_min = temperature;
	log_values->temperature_sum += temperature;
	
	
	if(log_values->humidity_max < humidity)
		log_values->humidity_max = humidity;
	if(log_values->humidity_min > humidity)
		log_values->humidity_min = humidity;
	log_values->humidity_sum += humidity;
	
	if(log_values->moisture_max < moisture)
		log_values->moisture_max = moisture;
	if(log_values->moisture_min > moisture)
		log_values->moisture_min = moisture;
	log_values->moisture_sum += moisture;
	
	if(log_values->light_max < light)
		log_values->light_max = light;
	if(log_values->light_min > light)
		log_values->light_min = light;
	log_values->light_sum += light;
	
	log_values->num_samples_measures++;
	
	if(dominant_color == 'R')
		log_values->count_color_red++;
	else if(dominant_color == 'G')
		log_values->count_color_green++;
	else if(dominant_color == 'B')
		log_values->count_color_blue++;
	
	if(log_values->accel_x_max < accel_values[0])
		log_values->accel_x_max = accel_values[0];
	if(log_values->accel_x_min > accel_values[0])
		log_values->accel_x_min = accel_values[0];
	if(log_values->accel_y_max < accel_values[1])
		log_values->accel_y_max = accel_values[1];
	if(log_values->accel_y_min > accel_values[1])
		log_values->accel_y_min = accel_values[1];
	if(log_values->accel_z_max < accel_values[2])
		log_values->accel_z_max = accel_values[2];
	if(log_values->accel_z_min > accel_values[2])
		log_values->accel_z_min = accel_values[2];
	
}

/**
* Calculate average sensors data
*/
void calculate_average_sensors_data(Log *log_values){
	log_values->temperature_avg = log_values->temperature_sum / log_values->num_samples_measures;
	log_values->humidity_avg = log_values->humidity_sum / log_values->num_samples_measures;
	log_values->light_avg = log_values->light_sum / log_values->num_samples_measures;
	log_values->moisture_avg = log_values->moisture_sum / log_values->num_samples_measures;

}
/**
* Calculate dominant color from  logs colors count
*/
char calculate_dominant_color_from_logs(Log log_values){
	char dominant_color_final = 'N';
	if(log_values.count_color_red>log_values.count_color_green && log_values.count_color_red> log_values.count_color_blue)
								dominant_color_final='R';
	else if(log_values.count_color_green>log_values.count_color_blue && log_values.count_color_green> log_values.count_color_red) 
								dominant_color_final='G';
	else if(log_values.count_color_blue>log_values.count_color_red && log_values.count_color_blue > log_values.count_color_green)
								dominant_color_final='B';
	else
		dominant_color_final = 'N';
	
	return dominant_color_final;
	
}