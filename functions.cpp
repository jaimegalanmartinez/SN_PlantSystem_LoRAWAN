#include "functions.h"

bool is_accel_interruptTap = false;

/*
* Interrupt Service Routine for INT1 accelerometer used for single taps
*/
void ISR_accelTap(){
	is_accel_interruptTap = true;
}


uint16_t writeFrameInBuffer(uint8_t buffer[30], short int temp, unsigned short humidity, unsigned short light,unsigned short moisture,
	char dominantColor, short int accel_x, short int accel_y, short int accel_z, PlantOrientationLog plantLog, PlantEvents plantEvents, 
	float latitude, float longitude, uint8_t hour, uint8_t minutes, uint8_t day, uint8_t month){
		
	uint16_t packet_len;
	
	memcpy(buffer, &temp, sizeof(short int));
	packet_len = (sizeof(short int));
	memcpy(buffer + packet_len, &humidity, sizeof(short int));
	packet_len += (sizeof(short int));
	memcpy(buffer + packet_len, &light, sizeof(short int));
	packet_len += (sizeof(short int));
	memcpy(buffer + packet_len, &moisture, sizeof(short int));
	packet_len += (sizeof(short int));
	memcpy(buffer + packet_len, &dominantColor, sizeof(char));
	packet_len += (sizeof(char));
	memcpy(buffer + packet_len, &accel_x, sizeof(short int));
	packet_len += (sizeof(short int));
	memcpy(buffer + packet_len, &accel_y, sizeof(short int));
	packet_len += (sizeof(short int));
	memcpy(buffer + packet_len, &accel_z, sizeof(short int));
	packet_len += (sizeof(short int));
	//Accelerometer advanced
	memcpy(buffer + packet_len, &plantLog.count_plant_falls, sizeof(uint8_t));
	packet_len += (sizeof(uint8_t));
	memcpy(buffer + packet_len, &plantEvents.count_plant_freefalls, sizeof(uint8_t));
	packet_len += (sizeof(uint8_t));
	memcpy(buffer + packet_len, &plantEvents.count_single_taps, sizeof(uint8_t));
	packet_len += (sizeof(uint8_t));
	//GPS
	memcpy(buffer + packet_len, &latitude, sizeof(float));
	packet_len += (sizeof(float));
	memcpy(buffer + packet_len, &longitude, sizeof(float));
	packet_len += (sizeof(float));
	memcpy(buffer + packet_len, &hour, sizeof(uint8_t));
	packet_len += (sizeof(uint8_t));
	memcpy(buffer + packet_len, &minutes, sizeof(uint8_t));
	packet_len += (sizeof(uint8_t));
	memcpy(buffer + packet_len, &day, sizeof(uint8_t));
	packet_len += (sizeof(uint8_t));
	memcpy(buffer + packet_len, &month, sizeof(uint8_t));
	packet_len += (sizeof(uint8_t));
	
	return packet_len;
}
	
/**
* Set the dominant color of the plant based on the readings provided by the color sensor
* @param int rgb_readings[4]
*	@return dominant_color ('R'= red, 'G'= green, 'B' = blue, 'N' = none)
*/
char set_dominant_color(int rgb_readings[4]){
	char dominant_color = 'N';
	if(rgb_readings[1]>rgb_readings[2] && rgb_readings[1]>=rgb_readings[3]){//If max=RED
		dominant_color='R'; //red
		
	}else if(rgb_readings[2]>rgb_readings[1] && rgb_readings[2]>rgb_readings[3]){//If max=Green
		dominant_color='G'; //green
		
	}else if(rgb_readings[3]>rgb_readings[1] && rgb_readings[3]>rgb_readings[2]){//If max=Blue
		dominant_color='B';	//blue	
		
	}else{
		dominant_color='N'; //nothing
	}
	return dominant_color;
}
