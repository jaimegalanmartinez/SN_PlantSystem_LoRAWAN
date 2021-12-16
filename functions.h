/*Sensor Networks PlantSystem project
* @Authors: Jaime Galan Martinez
*						Victor Aranda Lopez
*/

#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include "mbed.h"
#include "accelerometer_advanced.h"
#include "log_values_sensors.h"
#include "hardware.h"

//#define PERIOD_HALF_HOUR  1800000000 //1800000000 Normal Operation: 1800000000 (30min) -> 1h for logs Testing: 5000000 (5s) -> 10s for logs
#define ON  1
#define OFF 0
#define MAIL_QUEUE_SIZE 1
//Event flags used to inform about messages to read 
/*#define EV_FLAG_READ_SENSORS (1UL << 0) // 00000000000000000000000000000001
#define EV_FLAG_PRINT_INFO (1UL << 9)
#define EV_FLAG_PRINT_INFO_ADVANCED (1UL << 2)
#define EV_FLAG_PRINT_INFO_LOGS (1UL << 1)*/
#define EV_FLAG_READ_GPS (1UL << 3)
//Sensors read cadency for each mode
/*#define SENSORS_READ_CADENCY_TEST 2000ms
#define SENSORS_READ_CADENCY_NORMAL 30000ms //30000ms (30s) normal operation, Testing = 2000ms
#define SENSORS_READ_CADENCY_ADVANCED 2000ms */
//Stack size for threads
#define STACK_SIZE_OUTPUT_THREAD 1500
//#define STACK_SIZE_MEASURE_THREAD 512
//#define STACK_SIZE_STATE_MACHINE_THREAD 4096

//Ranges for sensor data
/*#define TEMPERATURE_MAX 22
#define TEMPERATURE_MIN 15
#define HUMIDITY_MAX 50
#define HUMIDITY_MIN 30
#define LIGHT_MAX 75
#define LIGHT_MIN 20
#define MOISTURE_MAX 75
#define MOISTURE_MIN 10*/

/*
 * Structure to send the values measured by the sensor thread to the main thread and
 * also to send this same values to the output thread from the main thread 
 */
 /*typedef struct {
		float temperature;
		float humidity;
		float light;
		float moisture;
		float accel_values[3];
	  int rgb_readings[4];
		char dominant_color;
} mail_t;*/
/*
 * Structure to send the values stored and calculated in the log of main thread 
 * to the output thread
 */
/*typedef struct {
		Log log_values;
		char dominant_color;
} mail_t_logs;*/

/*
 * Structure to send the measured values and the GPS to the send_message thread so that 
 * they are senit to LoRaWAN.
 */
 /*typedef struct {
		float latitude;
		float longitude;
		float altitude;
		uint8_t local_time_hour;
		uint8_t minute;
		uint8_t seconds;
} mail_t_gps;
 */
/*extern Mail<mail_t, MAIL_QUEUE_SIZE> sensor_data_mail_box, print_mail_box;
extern Mail<mail_t_logs, MAIL_QUEUE_SIZE> print_logs_mail_box;
extern Mail<mail_t_advanced, MAIL_QUEUE_SIZE> print_mail_box_advanced;*/
//extern Mail<mail_t_gps, MAIL_QUEUE_SIZE> gps_mail_box;
extern EventFlags event_flags;

//Threads
//extern Thread measure_thread;//Measures all elements except the GPS
extern Thread output_thread;//Prints the relevant data to the serial port (printf) and controls also the GPS
//extern Thread state_machine_thread;//Prints the relevant data to the serial port (printf) and controls also the GPS


extern volatile bool user_button_flag;
extern bool half_hour_flag, is_accel_interruptTap;

//Possible states of the state machine
enum Mode{TEST,NORMAL,ADVANCED};
/////////////////////////////////////////////
//Global variables
extern Mode mode;

//Interrupt service routines(ISRs)
void half_hour_irq();
void user_button();
void ISR_accelTap();

//void state_machine(void);
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
//void checkRange_and_set_RGB_color(float temperature,float humidity,float light_value_f,float moisture_value_f,float accel_values [],char dominantColor);

/**
* Set the dominant color of the plant based on the readings provided by the color sensor
* @param int rgb_readings[4]
*	@return dominant_color ('R'= red, 'G'= green, 'B' = blue, 'N' = none)
*/
//char set_dominant_color(int rgb_readings[4]);

/**
* With the dominant color we set the RGB led color
*
* RGB_LED ('R'= 0b001, 'G'= 0b010, 'B' = 0b100, 'N' = 0b000)
*/
//void set_color_RGB_led(char dominant_color);

/**
* Function put_sensor_data_on_Mailbox
* Description: Read sensors data, put in on sensor mailbox 
* and activate event flag EV_FLAG_READ_SENSORS to inform the main thread. 
* Executed each depending on the mode: SENSORS_READ_CADENCY_TEST (2s),SENSORS_READ_CADENCY_NORMAL (30s), SENSORS_READ_CADENCY_ADVANCED(2s) in the measure_thread
*/
//void put_sensor_data_on_Mailbox(void);

/**
* Function put_sensor_data_to_print_mail, executed by the main thread.
* @param mail_t *mail_data_sensor 
* Description:  Get the mail data from sensor mailbox, put in on print mailbox 
* and activate event flag EV_FLAG_PRINT_INFO to print the sensors data and the dominant color.
*/
//void put_sensor_data_to_print_mail(mail_t *mail_data_sens);

/**
* Function put_log_sensor_data_to_print_mail_logs executed by the main thread
* @param Log log_values Stores teh max, min, avg values of recorded measurements
* @param char dominant_color has a 'R','G','B' or 'N' depending of the dominant color of that hour
* Description:  Gets the log values of the main thread and dominant color of that hour and put them in on print mailbox
* Finally, it activates event flag EV_FLAG_PRINT_INFO_LOGS to print the log.
*/
//void put_log_sensor_data_to_print_mail_logs(Log log_hour_values, char dominant_color);

/** 
* Function put_sensor_data_to_print_mail_advanced executed by the main thread
* @param PlantOrientationLog advancedLog, mail_t * mail_data_sens, PlantEvents * plantEvents 
* Description:  Gets the sensors data, plant count falls, plant single taps count, and plant freefall count
* of the main thread and put them in on print mailbox
* Finally, it activates event flag EV_FLAG_PRINT_INFO_ADVANCED to print the ADVANCED mode data.
*/
//void put_sensor_data_to_print_mail_advanced(PlantOrientationLog * advancedLog, mail_t *mail_data_sens, PlantEvents* plantEvents);

/*
 * Converts the 'R','G','B' or 'N' char to strings to later print the dominant color in serial
 */
//char const* get_str_dominant_color(char dominant_color);

//Thread tasks
/**
* Task executed by the measure_thread
* @description Init temp/hum sensor and rgb_sensor, 
* reads sensors data and send it to main thread in a mailbox
* Depending on the mode, the thread sleeps with the sensor read cadency specified
* Test - 2s
* Normal - 30s
* Advanced - 2s
*/
//void measure_sensors(void);

/**
* GPS_and_print_info_system
* Description: Task executed by the output thread to print system info and to manage GPS readings:
* When it receives the event flag EV_FLAG_PRINT_INFO, retrieves from print mailbox
* the values to print.
* Additionaly, if it receives EV_FLAG_PRINT_INFO_LOGS, retrieves from print_logs_mail_box (mailbox the log and dominant colorn to print)
* Additionaly, if it receives EV_FLAG_PRINT_INFO_ADVANCED, retrieves from print_mail_box_advanced the values to print in ADVANCED mode
* 
*/ 
//void GPS_and_print_info_system(void);

void read_GPS();
#endif