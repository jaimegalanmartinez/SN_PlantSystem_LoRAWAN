#include "mbed.h"
#include "hardware.h"
/*Sensor Networks PlantSystem project
* @Authors: Jaime Galan Martinez
*						Victor Aranda Lopez
*/

#ifndef _ACCELEROMETER_ADVANCED_H_
#define _ACCELEROMETER_ADVANCED_H_

typedef struct {
	uint8_t count_single_taps;
	uint8_t count_plant_freefalls;
} PlantEvents;

typedef struct {
		float temperature;
		float humidity;
		float light;
		float moisture;
		float accel_values[3];
	  int rgb_readings[4];
		char dominant_color;
		uint8_t count_plant_falls;
		PlantEvents plantEvents;
} mail_t_advanced;

enum PlantOrientation{UP,DOWN};

typedef struct {
		uint8_t count_plant_falls;
		PlantOrientation previousState;
} PlantOrientationLog;

/**
* Updates the plant orientation based in accelerometer values.
* If plant was up
* 	and If absolute value Z is not greater than X or Y - plant orientation = DOWN
* If plant was down
* 	If Z is greater than X or Y - plant orientation = UP
*/
void updatePlantOrientation ( PlantOrientationLog *log, float accel_values[3]);

#endif



