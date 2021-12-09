#include "mbed.h"
#include "hardware.h"
#include "accelerometer_advanced.h"


void updatePlantOrientation ( PlantOrientationLog *log, float accel_values[3])
{
	if(log->previousState==UP){//If plant was up
		if( !(abs(accel_values[2]) > abs(accel_values[1])) || !(abs(accel_values[2]) > abs(accel_values[0]))){//If Z is not greater than X or Y
			log->count_plant_falls++;
			log->previousState=DOWN;
		}
	}else{//If plant was down
		if( (abs(accel_values[2]) > abs(accel_values[1])) && (abs(accel_values[2]) > abs(accel_values[0]))){//If Z is greater than X or Y
			log->previousState=UP;
		}
	}
}
