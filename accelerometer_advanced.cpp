#include "mbed.h"
#include "hardware.h"
#include "functions.h"
#include "accelerometer_advanced.h"

void initAdvancedMode(void){
    //Accelerometer advanced
    plantLog.count_plant_falls=0; //uint8_t
    plantLog.previousState=UP;
    //Accel interrupt INT1 (Single Tap)
    plantEvents.count_single_taps = 0; //uint8_t
    plantEvents.count_plant_freefalls = 0; //uint8_t
    accel_sensor.setupSingleTap();
    accel_sensor.initFreeFall();
    accel_interruptTap.rise(&ISR_accelTap);
}

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
