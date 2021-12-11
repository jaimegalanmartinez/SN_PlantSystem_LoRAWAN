#include "functions.h"

volatile bool user_button_flag = false;
bool half_hour_flag = false;
bool is_accel_interruptTap = false;

//Initial mode
Mode mode = TEST;

//Mailbox for communicate the measure thread with the main thread
Mail<mail_t, MAIL_QUEUE_SIZE> sensor_data_mail_box;
//Mailbox for communicate the main thread with the output thread
Mail<mail_t, MAIL_QUEUE_SIZE> print_mail_box;
//Mailbox for communicate sensor_data logs the main thread with the output thread
Mail<mail_t_logs, MAIL_QUEUE_SIZE> print_logs_mail_box;
//Mailbox for communicate sensor_data, plant orientation and plant events to the main thread with the output thread in advanced mode
Mail<mail_t_advanced, MAIL_QUEUE_SIZE> print_mail_box_advanced;

EventFlags event_flags;
Mutex serial_mutex; //Mutex for serial communication

/*
* Interrupt Service Routine, ticker
*/
void half_hour_irq()
{
		half_hour_flag = true;
}
/*
* Interrupt Service Routine, 
* Detects if the user button is pressed to change system mode (TEST, NORMAL, ADVANCED)
*/
void user_button()
{
		user_button_flag = true;
}
/*
* Interrupt Service Routine for INT1 accelerometer used for single taps
*/
void ISR_accelTap(){
	is_accel_interruptTap = true;
}

/** Function checkRange_and_set_RGB_color
	@description Check sensor data ranges and set the RGB color.
	@params temperature, humidity, light_value, moisture_value, accel_values and dominantColor
	Normal ranges:
	Temperature: -10 - +50 C
	Humidity: 25-75%
	Ambient light: 0-100%
	Soil humidity: 0-100%
	Acceleration: Z axis > X axis && Z axis > Y axis

	Testing ranges:
	Temperature: 15-22 C
	Humidity: 30-50%
	Ambient light: 20-75%
	Soil humidity: 10-75%
	Acceleration: Z axis > X axis && Z axis > Y axis

	Color codes when out of range:
	Temperature: LED off
	Humidity: White (RED + GREEN + BLUE)
	Ambient light: Margenta (RED + BLUE)
	Soil humidity: Cian (GREEN + BLUE)
	Color: yellow (RED + GREEN)
	Accelerometer: red (RED)

	Color when is in range:
	No errors: Green - Detecting plant
*/
void checkRange_and_set_RGB_color(float temperature,float humidity,float light_value_f,float moisture_value_f,float accel_values [],char dominantColor){
	if(temperature > TEMPERATURE_MAX || temperature < TEMPERATURE_MIN)
		RGB_LED=0b000;
	else	if(humidity > HUMIDITY_MAX || humidity < HUMIDITY_MIN){
		RGB_LED=0b111;
	}else if(light_value_f > LIGHT_MAX || light_value_f < LIGHT_MIN){
		RGB_LED=0b101;
	}else if(moisture_value_f > MOISTURE_MAX || moisture_value_f < MOISTURE_MIN){
		RGB_LED=0b110;
	}else if(dominantColor != 'G'){
		RGB_LED=0b011;
	}else if( !(abs(accel_values[2]) > abs(accel_values[1])) || !(abs(accel_values[2]) > abs(accel_values[0]))){
		RGB_LED=0b001;
	}else if(dominantColor == 'G'){//If no errors=GREEN, detecting plant
		RGB_LED=0b010;
	}
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
/**
* With the dominant color we set the RGB led color
*
* RGB_LED ('R'= 0b001, 'G'= 0b010, 'B' = 0b100, 'N' = 0b000)
*/
void set_color_RGB_led(char dominant_color){
	if(dominant_color == 'R'){//If max=RED
		RGB_LED=0b001;
	}else if(dominant_color == 'G'){//If max=Green
		RGB_LED=0b010;
	}else if(dominant_color == 'B'){//If max=Blue
		RGB_LED=0b100;	
	}else if(dominant_color == 'N'){ ////If max=None
		RGB_LED=0b000;
	}
}


/**
* Function put_sensor_data_on_Mailbox
* Description: Read sensors data, put in on sensor mailbox 
* and activate event flag EV_FLAG_READ_SENSORS to inform the main thread. 
* Executed each depending on the mode: SENSORS_READ_CADENCY_TEST (2s),SENSORS_READ_CADENCY_NORMAL (30s), SENSORS_READ_CADENCY_ADVANCED(2s) in the measure_thread
*/
void put_sensor_data_on_Mailbox(void)
{
		int rgb_readings[4]; // declare a 4 element array to store RGB sensor readings
		float accel_values [3];
		mail_t *mail_data_sensors = sensor_data_mail_box.try_calloc();
	
		//Temperature and humidity
		tempHumSensor.get_data();
		int32_t temperature = tempHumSensor.get_temperature();//Value is multiplied by 1000
		uint32_t humidity = tempHumSensor.get_humidity();//Value is multiplied by 1000
	  //Temp
		mail_data_sensors->temperature = temperature/1000.0;
	  //Humidity
		mail_data_sensors->humidity = humidity/1000.0;
		//Light sensor
		mail_data_sensors->light = lightSensor.read_u16()*100.0/65536.0;
		//Moisture sensor
		mail_data_sensors->moisture = moistureSensor.read_u16()*100.0/65536.0;
		//RGB sensor, and calculares the dominant color
		rgb_sensor.getAllColors(rgb_readings); // read the sensor to get red, green, and blue color data along with overall brightness
		mail_data_sensors->rgb_readings[0] = rgb_readings[0];		//Clear
		mail_data_sensors->rgb_readings[1] = rgb_readings[1];		//Red
		mail_data_sensors->rgb_readings[2] = rgb_readings[2];		//Green
		mail_data_sensors->rgb_readings[3] = rgb_readings[3];		//Blue
		mail_data_sensors->dominant_color = set_dominant_color(rgb_readings);
		//Acceleration sensor
		accel_sensor.getAccAllAxis(accel_values);
		mail_data_sensors->accel_values[0] = accel_values[0]; // Accel x
		mail_data_sensors->accel_values[1] = accel_values[1]; // Accel y
		mail_data_sensors->accel_values[2] = accel_values[2]; // Accel z
		//Send mail
    sensor_data_mail_box.put(mail_data_sensors);
		event_flags.set(EV_FLAG_READ_SENSORS);		
}

/////////////////////////////////////////////////////////////////////////////////////
/**
* Function put_sensor_data_to_print_mail, executed by the main thread.
* @param mail_t *mail_data_sensor 
* Description:  Get the mail data from sensor mailbox, put in on print mailbox 
* and activate event flag EV_FLAG_PRINT_INFO to print the sensors data and the dominant color.
*/
void put_sensor_data_to_print_mail(mail_t *mail_data_sens){
	
		mail_t *mail_data_print = print_mail_box.try_calloc();
		//Temp
		mail_data_print->temperature = mail_data_sens->temperature;
		//Humidity
		mail_data_print->humidity = mail_data_sens->humidity;
		//Light
		mail_data_print->light = mail_data_sens->light;
		//Moisture
		mail_data_print->moisture = mail_data_sens->moisture;
		//RGB
		mail_data_print->rgb_readings[0] = mail_data_sens->rgb_readings[0];		//Clear
		mail_data_print->rgb_readings[1] = mail_data_sens->rgb_readings[1];		//Red
		mail_data_print->rgb_readings[2] = mail_data_sens->rgb_readings[2];		//Green
		mail_data_print->rgb_readings[3] = mail_data_sens->rgb_readings[3];		//Blue
		mail_data_print->dominant_color = mail_data_sens->dominant_color;
		//Accelerometer
		mail_data_print->accel_values[0] = mail_data_sens->accel_values[0]; // Accel x
		mail_data_print->accel_values[1] = mail_data_sens->accel_values[1]; // Accel y
		mail_data_print->accel_values[2] = mail_data_sens->accel_values[2]; // Accel z
		
		print_mail_box.put(mail_data_print);
		event_flags.set(EV_FLAG_PRINT_INFO);	
}

/**
* Function put_log_sensor_data_to_print_mail_logs
* @param Log log_values Stores teh max, min, avg values of recorded measurements
* @param char dominant_color has a 'R','G','B' or 'N' depending of the dominant color of that hour
* Description:  Gets the log values of the main thread and dominant color of that hour and put them in on print mailbox
* Finally, it activates event flag EV_FLAG_PRINT_INFO_LOGS to print the log.
*/
void put_log_sensor_data_to_print_mail_logs(Log log_hour_values, char dominant_color){
	
		mail_t_logs *mail_data_print = print_logs_mail_box.try_calloc();
		//Temp
		mail_data_print->log_values.temperature_max = log_hour_values.temperature_max;
		mail_data_print->log_values.temperature_min = log_hour_values.temperature_min;
		mail_data_print->log_values.temperature_avg = log_hour_values.temperature_avg;
		//Humidity
		mail_data_print->log_values.humidity_max = log_hour_values.humidity_max;
		mail_data_print->log_values.humidity_min = log_hour_values.humidity_min;
		mail_data_print->log_values.humidity_avg = log_hour_values.humidity_avg;
		//Light
		mail_data_print->log_values.light_max = log_hour_values.light_max;
		mail_data_print->log_values.light_min = log_hour_values.light_min;
		mail_data_print->log_values.light_avg = log_hour_values.light_avg;
		//Moisture
		mail_data_print->log_values.moisture_max = log_hour_values.moisture_max;
		mail_data_print->log_values.moisture_min = log_hour_values.moisture_min;
		mail_data_print->log_values.moisture_avg = log_hour_values.moisture_avg;
		//Dominant color
		mail_data_print->dominant_color = dominant_color;
		//Accelerometer
		mail_data_print->log_values.accel_x_max = log_hour_values.accel_x_max; // Accel x
		mail_data_print->log_values.accel_x_min = log_hour_values.accel_x_min;
		mail_data_print->log_values.accel_y_max = log_hour_values.accel_y_max; // Accel y
		mail_data_print->log_values.accel_y_min = log_hour_values.accel_y_min;
		mail_data_print->log_values.accel_z_max = log_hour_values.accel_z_max; // Accel z
		mail_data_print->log_values.accel_z_min = log_hour_values.accel_z_min;
																																				
		print_logs_mail_box.put(mail_data_print);
		event_flags.set(EV_FLAG_PRINT_INFO_LOGS);	
}
/** 
* Function put_sensor_data_to_print_mail_advanced
* @param PlantOrientationLog advancedLog, mail_t * mail_data_sens, PlantEvents * plantEvents 
* Description:  Gets the sensors data, plant count falls, plant single taps count, and plant freefall count
* of the main thread and put them in on print mailbox
* Finally, it activates event flag EV_FLAG_PRINT_INFO_ADVANCED to print the ADVANCED mode data.
*/

void put_sensor_data_to_print_mail_advanced(PlantOrientationLog * advancedLog, mail_t *mail_data_sens, PlantEvents* plantEvents){
	
		mail_t_advanced *mail_data_print_advanced = print_mail_box_advanced.try_calloc();
		//Temp
		mail_data_print_advanced->temperature = mail_data_sens->temperature;
		//Humidity
		mail_data_print_advanced->humidity = mail_data_sens->humidity;
		//Light
		mail_data_print_advanced->light = mail_data_sens->light;
		//Moisture
		mail_data_print_advanced->moisture = mail_data_sens->moisture;
		//RGB
		mail_data_print_advanced->rgb_readings[0] = mail_data_sens->rgb_readings[0];		//Clear
		mail_data_print_advanced->rgb_readings[1] = mail_data_sens->rgb_readings[1];		//Red
		mail_data_print_advanced->rgb_readings[2] = mail_data_sens->rgb_readings[2];		//Green
		mail_data_print_advanced->rgb_readings[3] = mail_data_sens->rgb_readings[3];		//Blue
		mail_data_print_advanced->dominant_color = mail_data_sens->dominant_color;
		//Accelerometer
		mail_data_print_advanced->accel_values[0] = mail_data_sens->accel_values[0]; // Accel x
		mail_data_print_advanced->accel_values[1] = mail_data_sens->accel_values[1]; // Accel y
		mail_data_print_advanced->accel_values[2] = mail_data_sens->accel_values[2]; // Accel z
		//Plant orientaton ADVANCED
		mail_data_print_advanced->count_plant_falls = advancedLog->count_plant_falls;
		//Plant FreeFalls
		mail_data_print_advanced->plantEvents.count_plant_freefalls = plantEvents->count_plant_freefalls;
		//Plant Single tap
		mail_data_print_advanced->plantEvents.count_single_taps = plantEvents->count_single_taps;
		
		print_mail_box_advanced.put(mail_data_print_advanced);
		event_flags.set(EV_FLAG_PRINT_INFO_ADVANCED);	
}
/*
 * Converts the 'R','G','B' or 'N' char to strings to later print the dominant color in serial
 */
char const* get_str_dominant_color(char dominant_color){
	char const *color_dominant_detected;
	if(dominant_color == 'R'){
		color_dominant_detected = "RED";
				
	}else if (dominant_color == 'G'){
		color_dominant_detected = "GREEN";
				
	}else if (dominant_color == 'B'){
		color_dominant_detected = "BLUE";
				
	}else if (dominant_color == 'N'){
		color_dominant_detected = "NONE";
	}
	return color_dominant_detected;
}


///////////////////////////////////////////////////////////////////////////////////
//MEASURE THREAD 
///////////////////////////////////////////////////////////////////////////////////
/**
* Task executed by the measure_thread
* @description Init temp/hum sensor and rgb_sensor, 
* reads sensors data and send it to main thread in a mailbox
* Depending on the mode, the thread sleeps with the sensor read cadency specified
* Test - 2s
* Normal - 30s
* Advanced - 2s
*/
void measure_sensors(void){
	//Init
	if(!tempHumSensor.check()){
			serial_mutex.lock();
			printf("Temperature and humidity sensor error");
			serial_mutex.unlock();
		}
	//Turn on color sensor
	//Get ENABLE register		
	rgb_sensor.enablePowerAndRGBC();
	rgb_sensor.enableWait();
	rgb_sensor.setWaitTime(1500);
		
	while(true) {
		if(mode == TEST || mode == NORMAL|| mode == ADVANCED){//If TEST, NORMAL or ADVANCED gets measurements
			put_sensor_data_on_Mailbox();
		}
		if(mode == TEST){
			ThisThread::sleep_for(SENSORS_READ_CADENCY_TEST);
		}else if (mode == NORMAL){
			ThisThread::sleep_for(SENSORS_READ_CADENCY_NORMAL);
		} else if (mode == ADVANCED){
			ThisThread::sleep_for(SENSORS_READ_CADENCY_ADVANCED);
		}	
	}
}

///////////////////////////////////////////////////////////////////////////////////
//OUTPUT THREAD 
///////////////////////////////////////////////////////////////////////////////////
/**
* Function GPS_and_print_info_system
* Description: Task executed by the output thread to print system info and to manage GPS readings:
* When it receives the event flag EV_FLAG_PRINT_INFO, retrieves from print mailbox
* the values to print.
* Additionaly, if it receives EV_FLAG_PRINT_INFO_LOGS, retrieves from print_logs_mail_box (mailbox the log and dominant colorn to print)
* Additionaly, if it receives EV_FLAG_PRINT_INFO_ADVANCED, retrieves from print_mail_box_advanced the values to print in ADVANCED mode
* 
*/ 
void GPS_and_print_info_system(void){
	uint32_t flags_read_serial_th;
	//Init GPS
	GPS_sensor.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //these commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
  GPS_sensor.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS_sensor.sendCommand(PGCMD_ANTENNA);
  
	while(true){
		//GPS
		char c = GPS_sensor.read();
		//If a NMEA message is received
		if (GPS_sensor.newNMEAreceived()) {
			if (!GPS_sensor.parse(GPS_sensor.lastNMEA()))  // this also sets the newNMEAreceived() flag to false
			// we can fail to parse a sentence in which case we should just wait for another
					;
		}
		//Reads if there is anything to print
		flags_read_serial_th = event_flags.wait_any(EV_FLAG_PRINT_INFO | EV_FLAG_PRINT_INFO_LOGS | EV_FLAG_PRINT_INFO_ADVANCED,0);//Wait for flag to send the information
		
		if(flags_read_serial_th == EV_FLAG_PRINT_INFO){
			mail_t *mail_data_info = (mail_t *) print_mail_box.try_get();//Get sensors value
			if (mail_data_info != NULL){
				//Modify hour to Madrid latitude
			uint8_t local_time_hour = GPS_sensor.hour + 1; //UTC+1 (Madrid Winter time)
			if(local_time_hour > 23) local_time_hour = 0;
			char const *dominant_color = get_str_dominant_color(mail_data_info->dominant_color);
			serial_mutex.lock();
			printf("GPS: #Sats: %d, Lat(UTC): %f, Long(UTC): %f, Altitude: %.0fm, GPS_time: %d:%d:%d GPS_date: %d/%d/%d\n",GPS_sensor.satellites,GPS_sensor.latitude/100,
				GPS_sensor.longitude/100,GPS_sensor.altitude,local_time_hour,GPS_sensor.minute,GPS_sensor.seconds, GPS_sensor.day, GPS_sensor.month, GPS_sensor.year);
			printf("TEMP/HUM: Temperature: %.1f C, Relative Humidity: %.1f%%\n",mail_data_info->temperature,mail_data_info->humidity);
			printf("ACCELEROMETER: X_axis=%.2fg, Y_axis=%.2fg, Z_axis=%.2fg\n",mail_data_info->accel_values[0],mail_data_info->accel_values[1],mail_data_info->accel_values[2]);
			printf("LIGHT: %.1f%%\n",mail_data_info->light);
			printf("SOIL MOISTURE: %.1f%%\n",mail_data_info->moisture);
			printf("COLOR SENSOR: Clear=%d, Red=%d, Green=%d, Blue=%d   -- Dominant color: %s\n\n",mail_data_info->rgb_readings[0],mail_data_info->rgb_readings[1],mail_data_info->rgb_readings[2],mail_data_info->rgb_readings[3],dominant_color);      
			serial_mutex.unlock();
			print_mail_box.free(mail_data_info);
			event_flags.clear(EV_FLAG_PRINT_INFO);
			}				
		}
		if(flags_read_serial_th == EV_FLAG_PRINT_INFO_LOGS){
			mail_t_logs *mail_data_logs = (mail_t_logs *) print_logs_mail_box.try_get();
			if (mail_data_logs != NULL){
				char const *dominant_color = get_str_dominant_color(mail_data_logs->dominant_color);
				serial_mutex.lock();
				printf("\n\n------------SUMMARY VALUES 1 HOUR---------\n");
				printf("TEMP: Max: %.1f C, Min: %.1f C, Avg %.1f C\n",mail_data_logs->log_values.temperature_max,mail_data_logs->log_values.temperature_min,mail_data_logs->log_values.temperature_avg);
				printf("HUM: Max: %.1f%%, Min: %.1f%%, Avg %.1f%%\n",mail_data_logs->log_values.humidity_max,mail_data_logs->log_values.humidity_min,mail_data_logs->log_values.humidity_avg);
				printf("LIGHT: Max: %.1f%%, Min: %.1f%%, Avg %.1f%%\n",mail_data_logs->log_values.light_max,mail_data_logs->log_values.light_min,mail_data_logs->log_values.light_avg);
				printf("MOISTURE: Max: %.1f%%, Min: %.1f%%, Avg %.1f%%\n",mail_data_logs->log_values.moisture_max,mail_data_logs->log_values.moisture_min,mail_data_logs->log_values.moisture_avg);
				printf("COLOR: Dominant color: %s\n",dominant_color);
				printf("ACCELEROMETER: X: Max: %.1f, Min: %.1f. Y: Max: %.1f, Min: %.1f. Z: Max: %.1f, Min: %.1f.\n",mail_data_logs->log_values.accel_x_max,mail_data_logs->log_values.accel_x_min,mail_data_logs->log_values.accel_y_max,mail_data_logs->log_values.accel_y_min,mail_data_logs->log_values.accel_z_max,mail_data_logs->log_values.accel_z_min);
				printf("----------------------------------------------\n\n");
				serial_mutex.unlock();
				print_logs_mail_box.free(mail_data_logs);
				event_flags.clear(EV_FLAG_PRINT_INFO_LOGS);
			}
		}
		if(flags_read_serial_th == EV_FLAG_PRINT_INFO_ADVANCED){
			mail_t_advanced *mail_data_info = (mail_t_advanced *) print_mail_box_advanced.try_get();//Get sensors value
			if (mail_data_info != NULL){
				//Modify hour to Madrid latitude
			uint8_t local_time_hour = GPS_sensor.hour + 1; //UTC+1 (Madrid Winter time)
			if(local_time_hour > 23) local_time_hour = 0;
			char const *dominant_color = get_str_dominant_color(mail_data_info->dominant_color);
			serial_mutex.lock();
			printf("GPS: #Sats: %d, Lat(UTC): %f, Long(UTC): %f, Altitude: %.0fm, GPS_time: %d:%d:%d GPS_date: %d/%d/%d\n",GPS_sensor.satellites,GPS_sensor.latitude/100,
				GPS_sensor.longitude/100,GPS_sensor.altitude,local_time_hour,GPS_sensor.minute,GPS_sensor.seconds, GPS_sensor.day, GPS_sensor.month, GPS_sensor.year);
			printf("TEMP/HUM: Temperature: %.1f C, Relative Humidity: %.1f%%\n",mail_data_info->temperature,mail_data_info->humidity);
			printf("ACCELEROMETER: X_axis=%.2fg, Y_axis=%.2fg, Z_axis=%.2fg\n",mail_data_info->accel_values[0],mail_data_info->accel_values[1],mail_data_info->accel_values[2]);
			printf("LIGHT: %.1f%%\n",mail_data_info->light);
			printf("SOIL MOISTURE: %.1f%%\n",mail_data_info->moisture);
			printf("COLOR SENSOR: Clear=%d, Red=%d, Green=%d, Blue=%d   -- Dominant color: %s\n",mail_data_info->rgb_readings[0],mail_data_info->rgb_readings[1],mail_data_info->rgb_readings[2],mail_data_info->rgb_readings[3],dominant_color);      
			printf("COUNT PLANT FALLS: %d\n",mail_data_info->count_plant_falls);
			printf("COUNT FREEFALLS: %d\n",mail_data_info->plantEvents.count_plant_freefalls);
			printf("SINGLE TAPS: %d\n\n", mail_data_info -> plantEvents.count_single_taps);
			serial_mutex.unlock();
			print_mail_box_advanced.free(mail_data_info);
			event_flags.clear(EV_FLAG_PRINT_INFO_ADVANCED);
			}				
		}
	}	
}