#include "mbed.h"
#include "hardware.h"
/*Sensor Networks PlantSystem project
* @Authors: Jaime Galan Martinez
*						Victor Aranda Lopez
*/

#ifndef _ACCELEROMETER_ADVANCED_H_
#define _ACCELEROMETER_ADVANCED_H_

/*
Stores the count of single taps and freefalls
*/
typedef struct {
	uint8_t count_single_taps;
	uint8_t count_plant_freefalls;
} PlantEvents;
//2 possible states of the orientation od the plant
enum PlantOrientation{UP,DOWN};
/*
Stores the count of plant falls. Also, uses the previousState to
determine if a new freefall has occured.
*/
typedef struct {
		uint8_t count_plant_falls;
		PlantOrientation previousState;
} PlantOrientationLog;

////////////////////////////////////
//Structures for advanced mode
////////////////////////////////////
static PlantOrientationLog plantLog;//Stores the status of the plant regarding the advanced mode
static PlantEvents plantEvents;

////////////////////////////////////
//Functions
////////////////////////////////////
/**
Initializes everything regarding advanced mode:
    Initializes the plantLog and plantEvents structures.
    Configures the SingleTap, Freefall, and interruption of the accelerometer.
*/
void initAdvancedMode(void);

/**
* Updates the plant orientation based in accelerometer values.
* If plant was up
* 	and If absolute value Z is not greater than X or Y - plant orientation = DOWN
* If plant was down
* 	If Z is greater than X or Y - plant orientation = UP
*/
void updatePlantOrientation ( PlantOrientationLog *log, float accel_values[3]);

#endif



