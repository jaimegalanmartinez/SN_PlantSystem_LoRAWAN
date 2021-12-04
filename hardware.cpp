#include "mbed.h"
#include "TCS3472_I2C/TCS3472_I2C.h"
#include "Accel_MMA8451Q/MMA8451Q.h"
#include "Si7021/Si7021.h"
#include "GPS/MBed_Adafruit_GPS.h"

#define MMA8451Q_SENSOR_ADDR	0x1C

MMA8451Q accel_sensor(PB_9, PB_8,MMA8451Q_SENSOR_ADDR << 1);
TCS3472_I2C rgb_sensor (PB_9, PB_8);
Si7021 tempHumSensor(PB_9, PB_8);
Adafruit_GPS GPS_sensor(new BufferedSerial(PA_9, PA_10, 9600));

DigitalOut TestMode_LED(LED1);
DigitalOut NormalMode_LED(LED2);
DigitalOut AdvancedMode_LED(LED3);
InterruptIn button(PB_2);
InterruptIn accel_interruptTap(PB_15);//INT1
AnalogIn moistureSensor(PA_0);
AnalogIn lightSensor(PA_4);
BusOut RGB_LED(PH_0,PH_1,PB_13);
              //LSB      MSB
Ticker halfHourTicker;
