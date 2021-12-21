/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 /*Sensor Networks PlantSystem project
* @Authors: Jaime Galan Martinez (Group J)
*			Victor Aranda Lopez (Group C)
*/

#include <stdio.h>

#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "trace_helper.h"
#include "lora_radio_helper.h"

#include "mbed.h"
#include "functions.h"
using namespace events;


// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t rx_buffer[30];

/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        10000

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3

/**
 * Dummy pin for dummy sensor
 */
//#define PC_9                            0

/**
 * Dummy sensor class object
 */
//DS1820  ds1820(PC_9);

/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it the radio object from lora_radio_helper.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;

// Jaime Galan   DEV_EUI = {0x7A, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94} //Group J
// Victor Aranda DEV_EUI = {0x73, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94} //Group C
static uint8_t DEV_EUI[] = { 0x7A, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94};
static uint8_t APP_EUI[] = { 0x70, 0xb3, 0xd5, 0x7e, 0xd0, 0x00, 0xfc, 0xda };
//Same value for sw application receiving the information
static uint8_t APP_KEY[] = { 0xf3,0x1c,0x2e,0x8b,0xc6,0x71,0x28,0x1d,0x51,0x16,0xf0,0x8f,0xf0,0xb7,0x92,0x8f };
/**
 * Entry point for application
 */
int main(void)
{	 
		initAdvancedMode();
	
    // setup tracing
    setup_trace();

    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }

    printf("\r\n Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
            != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    printf("\r\n CONFIRMED message retries : %d \r\n",
           CONFIRMED_MSG_RETRY_COUNTER);

    // Enable adaptive data rate
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");
    lorawan_connect_t connect_params;
		connect_params.connect_type = LORAWAN_CONNECTION_OTAA;
    connect_params.connection_u.otaa.dev_eui = DEV_EUI;
    connect_params.connection_u.otaa.app_eui = APP_EUI;
    connect_params.connection_u.otaa.app_key = APP_KEY;
    connect_params.connection_u.otaa.nb_trials = 3;
		
    retcode = lorawan.connect(connect_params);

    if (retcode == LORAWAN_STATUS_OK ||
            retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");
		
    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();

    return 0;
}

/**
 * Sends a message to the Network Server
 */
static void send_message()
{
    uint16_t packet_len;
    int16_t retcode;
	
    //Default values to send
    float temp = 25.2; //0x 41 c9 99 9a - in buffer -> 9A99 C941
    float humidity = 45.2; //0x4234cccd -> CDCC 3442
    float light = 10.2; //0x41233333
    float moisture = 32.1; //0x42006666
    float accel_values [3]; //[0]=X, [1]=Y, [2]=Z
    int rgb_readings[4];
    char dominantColor = 'G';
    float latitude = 40.3903; //0x42218fab
    float longitude = -3.62702; //0xc0682118
    uint8_t hour = 11;
    uint8_t minutes = 25;
    uint8_t day = 17;
    uint8_t month = 12;
    
    //Temperature and humidity
    tempHumSensor.get_data();
    temp = tempHumSensor.get_temperature()/1000.0;//Value is multiplied by 1000
    humidity = tempHumSensor.get_humidity()/1000.0;//Value is multiplied by 1000
    //Light sensor
    light = lightSensor.read_u16()*100.0/65536.0;
    //Moisture sensor
    moisture = moistureSensor.read_u16()*100.0/65536.0;
    //Colour sensor
    rgb_sensor.getAllColors(rgb_readings); // read the sensor to get red, green, and blue color data along with overall brightness
    dominantColor = set_dominant_color(rgb_readings); //in hex -> 'N' = 4E , 'R' = 52 , 'G' = 47 , 'B' = 42
    //Accelerometer
    accel_sensor.getAccAllAxis(accel_values);
    //GPS
    uint8_t read_gps_obtained = false;
    while(!read_gps_obtained){
        char c = GPS_sensor.read();
        //If a NMEA message is received
        if (GPS_sensor.newNMEAreceived()) {
            if (!GPS_sensor.parse(GPS_sensor.lastNMEA())){  // this also sets the newNMEAreceived() flag to false
                ;// we can fail to parse a sentence in which case we should just wait for another          
            }else{
                uint8_t local_time_hour = GPS_sensor.hour + 1; //UTC+1 (Madrid Winter time)
                if(local_time_hour > 23) local_time_hour = 0;
                
                latitude = GPS_sensor.latitude/100;
                longitude = GPS_sensor.longitude/100;
                if(GPS_sensor.lon =='W') longitude=longitude*-1.0;
                if(GPS_sensor.lat =='S') latitude=latitude*-1.0;
                hour = local_time_hour;
                minutes = GPS_sensor.minute;
                day = GPS_sensor.day;
                month = GPS_sensor.month;

                read_gps_obtained=true;	
                    
            }	//else
        } //if GPS NMEA received
    } //while GPS

    //Advanced mode values
        //Count plant falls
    updatePlantOrientation(&plantLog, accel_values);
        //Count single taps
    if(is_accel_interruptTap) { //Single Tap
        is_accel_interruptTap = false;
        plantEvents.count_single_taps++;	
    }
        //Detects free fall
    if(accel_sensor.getFF()){
        plantEvents.count_plant_freefalls++;
    }    

    //Conversions to reduce the size of the data sent
    short int tempI = temp * 100; //2520 		         -> 0x09D8 -in buffer -> D809
    unsigned short int humidityI = humidity * 100; //4520 -> 0x11A8 -in buffer -> A811
    unsigned short int lightI = light * 100; //1020 			-> 0x03FC -in buffer -> FC03
    unsigned short int moistureI = moisture * 100.0; //3209 -> 0x0C89 -in buffer -> 890C
    short int accel_x = accel_values[0] * -1000; // 0.0087890625 -> -8
    short int accel_y = accel_values[1] * -1000; // 0.125488281  -> -125
    short int accel_z = accel_values[2] * -1000; // 1.0078125    -> -1007

    //Summary
    printf("Temp: %d, Hum: %u, Light: %u, Moisture: %u, Dominant colour: %c\n", tempI, humidityI, lightI, moistureI, dominantColor);
    printf("AccelS X: %d AccelS Y: %d, AccelS Z: %d\n", accel_x, accel_y, accel_z);
    printf("COUNT PLANT FALLS: %d\n",plantLog.count_plant_falls);
    printf("COUNT FREEFALLS: %d\n",plantEvents.count_plant_freefalls);
    printf("SINGLE TAPS: %d\n", plantEvents.count_single_taps);
    printf("GPS: Lat(UTC): %d, Long(UTC): %d \n",(int)latitude,(int)longitude);
    printf("Hour: %d:%d Date: %d/%d,\n", hour, minutes, day, month);
    
    //Frame created
    //FRAME: Temp (2bytes), Humidity (2bytes), Light (2bytes), Moisture (2bytes),  //8 bytes
    //			 Dominant colour (1byte),AccelX(2 bytes), AccelY (2bytes), AccelZ (2bytes), // 7 bytes
    //			 Count plant falls (1 byte), Freefalls (1 byte), Single Taps (1 byte) //3 bytes
    //       GPS: Latitude (4bytes), Longitude (4bytes) //8 bytes
    //       GPS: Hour (1 byte), Minutes (1 byte), Day (1 byte) Month (1 byte) //4 bytes
    //FRAME BUFFER TX: D809 A811 FC03 890C 47 0800 7D00 EF03 00 00 00 AB8F2142 182168C0 0B 19 11 0c //30 bytes
    packet_len = writeFrameInBuffer(tx_buffer, tempI, humidityI, lightI, moistureI, dominantColor, accel_x, accel_y, accel_z,
                                                                    plantLog, plantEvents, latitude, longitude, hour, minutes, day, month);
    
    //PRINT FRAME TO SEND 
    for(uint8_t i = 0; i < sizeof(tx_buffer); i ++){
        printf("%02x", tx_buffer[i]);
    }

    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,
                           MSG_UNCONFIRMED_FLAG);

    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
        : printf("\r\n send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            //retry in 3 seconds
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                ev_queue.call_in(3000, send_message);
            }
        }
        return;
    }

    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

/**
 * Receive a message from the Network Server
 */
static void receive_message()
{
    uint8_t port;
    int flags;
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }
    /* 
      Possible commands: OFF, Red, Green
    */
    if(strncmp((char *)rx_buffer, "OFF" , sizeof(retcode)) == 0){ //OFF -> 4F 46 46
        printf("Led off");
				RGB_LED=0b000; //LED OFF
    }else if (strncmp((char *)rx_buffer, "Green" ,sizeof(retcode)) == 0){ //Green -> 47 72 65 65 6e
        printf("Led green");
				RGB_LED=0b010; //LED Green
    }else if (strncmp((char *)rx_buffer, "Red" , sizeof(retcode)) == 0){ //Red -> 52 65 64
        printf("Led red"); 
				RGB_LED=0b001; //LED Red
    }

    printf(" RX Data on port %u (%d bytes): ", port, retcode);
    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x ", rx_buffer[i]);
    }
    printf("\r\n");
    
    memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event)
{
    switch (event) {
        case CONNECTED:
            printf("\r\n Connection - Successful \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                ev_queue.call_every(TX_TIMER, send_message);
            }

            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("\r\n Disconnected Successfully \r\n");
            break;
        case TX_DONE:
            printf("\r\n Message Sent to Network Server \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case RX_DONE:
            printf("\r\n Received message from Network Server \r\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("\r\n Error in reception - Code = %d \r\n", event);
            break;
        case JOIN_FAILURE:
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        case UPLINK_REQUIRED:
            printf("\r\n Uplink required by NS \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

// EOF
