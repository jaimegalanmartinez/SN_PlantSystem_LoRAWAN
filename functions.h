/*Sensor Networks PlantSystem project
* @Authors: Jaime Galan Martinez
*						Victor Aranda Lopez
* This file contains various funtions but without an specific functionality to be included in a separate file
*/

#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include "mbed.h"
#include "accelerometer_advanced.h"
#include "hardware.h"

#define ON  1
#define OFF 0


extern bool is_accel_interruptTap;
////////////////////////////////////
//Interrupt service routines(ISRs)
////////////////////////////////////
//It is call when pin INT1 of the accelerometer has a rise flank 
void ISR_accelTap();

////////////////////////////////////
//Functions
////////////////////////////////////
/** Function checkRange_and_set_RGB_color
	@description Write frame with sensor values, accelerometer advanced parameters and GPS location, GPS time and date.
	//FRAME: Temp (2bytes), Humidity (2bytes), Light (2bytes), Moisture (2bytes),  //8 bytes
		//			 Dominant colour (1 byte),AccelX(2 bytes), AccelY (2 bytes), AccelZ (2 bytes), // 7 bytes
		//			 Count plant falls (1 byte), Freefalls (1 byte), Single Taps (1 byte) //3 bytes
		//       GPS: Latitude (4bytes), Longitude (4bytes) //8 bytes
		//       GPS: Hour (1 byte), Minutes (1 byte), Day (1 byte) Month (1 byte) //4 bytes
	@params temperature, humidity, light, moisture, dominantColor, accel_x, accel_y, accel_z, plantLog, plantEvents, latitude, longitude, hour, minutes, day, month
**/
uint16_t writeFrameInBuffer(uint8_t buffer[30], short int temp, unsigned short humidity, unsigned short light,unsigned short moisture,
	char dominantColor, short int accel_x, short int accel_y, short int accel_z, PlantOrientationLog plantLog, PlantEvents plantEvents, 
	float latitude, float longitude, uint8_t hour, uint8_t minutes, uint8_t day, uint8_t month);


/**
* Set the dominant color of the plant based on the readings provided by the color sensor
* @param int rgb_readings[4]
*	@return dominant_color ('R'= red, 'G'= green, 'B' = blue, 'N' = none)
*/
char set_dominant_color(int rgb_readings[4]);

#endif