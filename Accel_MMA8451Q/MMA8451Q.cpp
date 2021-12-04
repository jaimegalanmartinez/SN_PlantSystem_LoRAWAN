#include "MMA8451Q.h"
 
#define REG_WHO_AM_I      0x0D
/*: CTRL_REG1 register (read/write)
Bit 7 Bit 6 Bit 5 Bit 4 Bit 3 Bit 2 Bit 1 Bit 0
ASLP_RATE1 ASLP_RATE0 DR2 DR1 DR0 LNOISE F_READ ACTIVE
*/
#define REG_CTRL_REG_1    0x2A
#define REG_OUT_X_MSB     0x01
#define REG_OUT_Y_MSB     0x03
#define REG_OUT_Z_MSB     0x05
 
#define FF_MT_CFG      0x15
#define FF_MT_SRC      0x16
#define FF_MT_THS      0x17
#define FF_MT_COUNT    0x18
#define FF_MT_CFG      0x15
#define CTRL_REG4      0x2D
#define CTRL_REG5      0x2E
#define UINT14_MAX        16383

/*Configures the event flag for the tap detection interrupt function for enabling/disabling single and double pulse on
 *each of the axes
					Tap Enable
					Bit 7 Bit 6 Bit 5 Bit 4 Bit 3 Bit 2 Bit 1 Bit 0
					DPA ELE ZDPEFE ZSPEFE YDPEFE YSPEFE XDPEFE XSPEFE
Single Tap 0 1 0 1 0 1 0 1 -> 0x55 ALL AXES
Double Tap 0 1 1 0 1 0 1 0 -> 0x6A
Both S & D 0 1 1 1 1 1 1 1 ->
When configured for a single tap event, an interrupt is generated when the input acceleration on the selected axis exceeds
the programmed threshold, and returns below it within a time window defined by PULSE_TMLT. If the ELE bit (bit 6) of the
PULSE_CFG (Reg 0x21) register is not set, the interrupt is kept high for the duration of the Latency window PULSE_LTCY (Reg
0x27). The latency window is a user-definable period of delay after each pulse. This latency window applies either for single pulse
or double pulse
 If the ELE bit is set, the source register values will remain static until the PULSE_SRC (Reg 0x22) register is read
*/
#define REG_PULSE_CFG	0x21 //ELE BIT 6

//With bit ELE activated
#define SINGLE_TAP 0x55 //in all axes 
#define SINGLE_TAP_X 0x41
#define SINGLE_TAP_Y 0x44
#define SINGLE_TAP_Z 0x50
#define DOUBLE_TAP 0x6A // in all axes
#define BOTH_TAPS  0x7F
//Without bit ELE
#define SINGLE_TAP_NO_ELE 0x15 //in all axes
#define NO_TAP_DETECTION 0x00
/*Threshold values to trigger tap event
 *The threshold values range from 0 to 127 (7 bits expressed as an absolute value) with
 *steps of 0.063g/LSB at a fixed 8g acceleration range
*/
#define REG_PULSE_THSX 0x23 //threshold_value_x
#define REG_PULSE_THSY 0x24 //threshold_value_y
#define REG_PULSE_THSZ 0x25 //threshold_value_z

/*The Pulse Latency Timer Register is the time duration that the tap event can be read from the source register to detect the X,
 *Y, and Z for single pulse or double pulse events without the latch enabled. The duration of any specified latency time is valid each
 *time a single or double pulse occurs.
*/
#define REG_PULSE_LTCY 0x27//Latency time to hold the event conditions

#define REG_PULSE_TMLT 0x26 //time window, time limit register
/*When the low pass filter is enabled the time step doubles.
* The filter should help eliminate additional ringing after the tap signature is detected.
*/
#define REG_LOW_PASS_FILTER 0x0F

/*Bit 7 Bit 6 Bit 5 Bit 4 Bit 3 Bit 2 Bit 1 Bit 0
 EA AxZ AxY AxX DPE PolZ PolY PolX
 */
#define REG_PULSE_SRC 0x22


//Pulse detection interrupt enabled in CTRL_REG4 (Bit 3 activated)
#define INT_EN_PULSE 0x08
//Enable free fall and pulse detection in CTRL_REG4 (Bit 3 and Bit2 activated)
#define INT_EN_TAP_FREE_FALL 0x0C

//Set tap to INT1 (CTRL_REG5 BIT3 INT_CFG_PULSE = 1 -> INT1 
//																INT_CFG_PULSE = 0 -> INT2
#define ENABLE_INT1  0x08
#define ENABLE_INT2  0x00

#define REG_INT_SOURCE 0x0C //Interrupt status

MMA8451Q::MMA8451Q(PinName sda, PinName scl, int addr) : m_i2c(sda, scl), m_addr(addr) {
    // activate the peripheral
    uint8_t data[2] = {REG_CTRL_REG_1, 0x01};//0x01 but 0x19
    writeRegs(data, 2);
}
 
MMA8451Q::~MMA8451Q() { }
 
uint8_t MMA8451Q::getWhoAmI() {
    uint8_t who_am_i = 0;
    readRegs(REG_WHO_AM_I, &who_am_i, 1);
    return who_am_i;
}
 
float MMA8451Q::getAccX() {
    return (float(getAccAxis(REG_OUT_X_MSB))/4096.0);
}
 
float MMA8451Q::getAccY() {
    return (float(getAccAxis(REG_OUT_Y_MSB))/4096.0);
}
 
float MMA8451Q::getAccZ() {
    return (float(getAccAxis(REG_OUT_Z_MSB))/4096.0);
}
 
void MMA8451Q::getAccAllAxis(float * res) {
    res[0] = getAccX();
    res[1] = getAccY();
    res[2] = getAccZ();
}
 
int16_t MMA8451Q::getAccAxis(uint8_t addr) {
    int16_t acc;
    uint8_t res[2];
    readRegs(addr, res, 2);
 
    acc = (res[0] << 6) | (res[1] >> 2);
    if (acc > UINT14_MAX/2)
        acc -= UINT14_MAX;
 
    return acc;
}
 
void MMA8451Q::readRegs(int addr, uint8_t * data, int len) {
    char t[1] = {(char)addr};
    m_i2c.write(m_addr, t, 1, true);
    m_i2c.read(m_addr, (char *)data, len);
}
 
void MMA8451Q::writeRegs(uint8_t * data, int len) {
    m_i2c.write(m_addr, (char *)data, len);
}

uint8_t MMA8451Q::getThreshold_x_tap() {
	uint8_t threshold_x_read = 0;
	readRegs(REG_PULSE_THSX,&threshold_x_read, 1);
  return threshold_x_read;
}

uint8_t MMA8451Q::getThreshold_y_tap() {
  uint8_t threshold_y_read = 0;
	readRegs(REG_PULSE_THSY,&threshold_y_read, 1);
  return threshold_y_read;
}

uint8_t MMA8451Q::getThreshold_z_tap() {
   uint8_t threshold_z_read = 0;
	readRegs(REG_PULSE_THSZ,&threshold_z_read, 1);
  return threshold_z_read;
}

void MMA8451Q::setThreshold_x_tap(uint8_t threshold_value) {
	uint8_t data[2] = {REG_PULSE_THSX, threshold_value};
  writeRegs(data, 2);
}

void MMA8451Q::setThreshold_y_tap(uint8_t threshold_value) {
  uint8_t data[2] = {REG_PULSE_THSY, threshold_value};
  writeRegs(data, 2);
}

void MMA8451Q::setThreshold_z_tap(uint8_t threshold_value) {
  uint8_t data[2] = {REG_PULSE_THSZ, threshold_value};
  writeRegs(data, 2);
}

void MMA8451Q::setupPulseConfig(uint8_t data_value){
	uint8_t data[2] = {REG_PULSE_CFG, data_value};
	writeRegs(data, 2);
}


void MMA8451Q::setTimeLimitTapDetection(uint8_t data_value){
	uint8_t data[2] = {REG_PULSE_TMLT, data_value};
	writeRegs(data, 2);
}

void MMA8451Q::setLatencyTimeTap(uint8_t data_value){
	uint8_t data[2] = {REG_PULSE_LTCY, data_value};
	writeRegs(data, 2);
}

void MMA8451Q::enablePulseInterrupt(uint8_t data_value){
	uint8_t data[2] = {CTRL_REG4, data_value};
	writeRegs(data, 2);
}

void MMA8451Q::routePulseInterruptBlock(uint8_t data_value){
	uint8_t data[2] = {CTRL_REG5, data_value};
	writeRegs(data, 2);
}

// Single Tap Only: Normal Mode, No Low Pass Filter, 400 Hz ODR (Output data rate)
void MMA8451Q::setupSingleTap(){
	 //Set standby-mode and 400Hz
		uint8_t data_ctrl_reg[2] = {REG_CTRL_REG_1, 0x08};
		writeRegs(data_ctrl_reg, 2);
	//Enable X,Y,Z single pulse with ELE bit deactivated
		setupPulseConfig(SINGLE_TAP_NO_ELE);
	/*	Set Threshold 1.575g on X and 2.65g on Z
			Note: Each step is 0.063g per count
			1.575g/0.063g = 25 counts
			2.65g/0.063g = 42 counts
	*/
		setThreshold_x_tap(0x19); //Set X Threshold to 1.575g
		setThreshold_y_tap(0x19); //Set X Threshold to 1.575g//0x19
		setThreshold_z_tap(0x19); //Set Z Threshold to 2.65g//(0x2A
		//Set Time Limit for Tap Detection to 50 ms, Normal Mode, No LPF
		//Data Rate 400 Hz, time step is 0.625 ms
		//50 ms/0.625 ms = 80 counts  80 = 0x50
		setTimeLimitTapDetection(0x50);//data_value = time_tap_detection/time_step
		//Set Latency Time to 300 ms
		//Data Rate 400 Hz, time step is 1.25 ms
		//300 ms/1.25 ms = 240 counts
		setLatencyTimeTap(0xF0);//300ms (240 in HEX)
		//Route INT1 to System Interrupt
		enablePulseInterrupt(INT_EN_PULSE); //0x08 Enable Pulse Interrupt Block in System CTRL_REG4
		routePulseInterruptBlock(ENABLE_INT1); //0x08 Route Pulse Interrupt Block to INT1 hardware Pin
		//Put the device in ActiveMode
		uint8_t status_ctrl_reg1 = 0;
		readRegs(REG_CTRL_REG_1,&status_ctrl_reg1, 1);
		status_ctrl_reg1 |= 0x01; //Bitwise OR, setting bit 0 to 1 (change mode to ACTIVE)
		uint8_t data_ctrl[2] = {REG_CTRL_REG_1, status_ctrl_reg1}; //Put device in Active Mode
		writeRegs(data_ctrl, 2);
}

void MMA8451Q::disableSingleTap(){
	//Set standby-mode and 400Hz
	uint8_t data_ctrl_reg[2] = {REG_CTRL_REG_1, 0x08};
	writeRegs(data_ctrl_reg, 2);
	//Disable event flag for tap detection
	setupPulseConfig(NO_TAP_DETECTION);
	//Put the device in ActiveMode
	uint8_t status_ctrl_reg1 = 0;
	readRegs(REG_CTRL_REG_1,&status_ctrl_reg1, 1);
	status_ctrl_reg1 |= 0x01; //Bitwise OR, setting bit 0 to 1 (change mode to ACTIVE)
	uint8_t data_ctrl[2] = {REG_CTRL_REG_1, status_ctrl_reg1}; //Put device in Active Mode
	writeRegs(data_ctrl, 2);
}

uint8_t MMA8451Q::detect_interrupt_generated(){
	//READ System Interrupt Status Source Register 0x0C
	uint8_t interrupt_type = 0x00; //None interrupt
	uint8_t interrupt_status_data = 0;
	readRegs(REG_INT_SOURCE,&interrupt_status_data, 1);
	if (interrupt_status_data & 0x08){ //SRC_PULSE = 1
		//Interrupt was generated due to single and/or double pulse event
		interrupt_type = 0x01;
	}
	if (interrupt_status_data & 0x04){ //SRC_FF_MT = 1
		//Indicates that the freefall/motion function interrupt is active
		interrupt_type = 0x02;
	}
	if (interrupt_status_data & 0x0C){ //SRC_PULSE = 1 and SRC_FF_MT = 1
		interrupt_type = 0x03;
	}
	return interrupt_type;
}


char MMA8451Q::detectSingleTap(void){
	char isDetected = 'N'; //not detected
	uint8_t data_pulse_src = 0;
   readRegs(REG_PULSE_SRC, &data_pulse_src, 1);
	//Bit 7 Bit 6 Bit 5 Bit 4 Bit 3 Bit 2 Bit 1 Bit 0
	//EA AxZ AxY AxX DPE PolZ PolY PolX
	//EA active (bit 7)- one or more interrupt events have been generated
	//DPE (0: Single Pulse Event triggered interrupt; 1: Double Pulse Event triggered interrupt)
	//Pulse polarity positive
   if (data_pulse_src == 0b11000000 || data_pulse_src == 0b11000100){ //Single tap Z-axis (pulse event  that triggered positive || pulse event that triggered negative)
		 isDetected = 'Z';
	 } else if (data_pulse_src == 0b10100000 || data_pulse_src == 0b10100010){ // /Single tap Y-axis
		 isDetected = 'Y';
	 }else if (data_pulse_src == 0b10010000 || data_pulse_src == 0b10010001){ // /Single tap X-axis	
		 isDetected = 'X';
	 }
	return isDetected;	 
}


/*
// activate free fall
	uint8_t data0[2] = {REG_CTRL_REG_1, 0x20};//0x19
  writeRegs(data0, 2);
  uint8_t data[2] = {FF_MT_CFG, 0b10111000};
	writeRegs(data,2);
	uint8_t data2[2] = {FF_MT_THS, 0b00000011};
	writeRegs(data2,2);
	uint8_t data5[2] = {FF_MT_COUNT, 0x06};
	writeRegs(data5,2);
	uint8_t data4[2] = {CTRL_REG4, 0x04};
	writeRegs(data4,2);
	uint8_t data44[2] = {CTRL_REG5, 0x00};
	writeRegs(data44,2);
  uint8_t data6[2] = {REG_CTRL_REG_1, 0x01};//0x19
  writeRegs(data6, 2);
	*/
void MMA8451Q::initFreeFall(){
	// activate free fall
	//Set standby-mode and 400Hz
	uint8_t data_ctrl_reg[2] = {REG_CTRL_REG_1, 0x08};
	writeRegs(data_ctrl_reg, 2);
	
  uint8_t data[2] = {FF_MT_CFG, 0b10111000};
	writeRegs(data,2);
	uint8_t data2[2] = {FF_MT_THS, 0b00000111};
	writeRegs(data2,2);
	uint8_t data5[2] = {FF_MT_COUNT, 0x06};//6
	writeRegs(data5,2);
	//Route INT1 to System Interrupt
	enablePulseInterrupt(INT_EN_TAP_FREE_FALL); //0x0C Enable TAP and FREE FALL Interrupt Block in System CTRL_REG4
	routePulseInterruptBlock(ENABLE_INT1); //0x08 Route FF Interrupt Block to INT2 hardware Pin
	//Put the device in ActiveMode
	uint8_t status_ctrl_reg1 = 0;
	readRegs(REG_CTRL_REG_1,&status_ctrl_reg1, 1);
	status_ctrl_reg1 |= 0x01; //Bitwise OR, setting bit 0 to 1 (change mode to ACTIVE)
	uint8_t data_ctrl[2] = {REG_CTRL_REG_1, status_ctrl_reg1}; //Put device in Active Mode
	writeRegs(data_ctrl, 2);
}
void MMA8451Q::uninitFreeFall(){
		// activate free fall
		uint8_t data_ctrl_reg[2] = {REG_CTRL_REG_1, 0x08};
	writeRegs(data_ctrl_reg, 2);
	// deactivate free fall
  uint8_t data[2] = {FF_MT_CFG, 0b00000000};
	writeRegs(data,2);
	uint8_t data2[2] = {FF_MT_THS, 0b00000000};
	writeRegs(data2,2);
	
	uint8_t status_ctrl_reg1 = 0;
	readRegs(REG_CTRL_REG_1,&status_ctrl_reg1, 1);
	status_ctrl_reg1 |= 0x01; //Bitwise OR, setting bit 0 to 1 (change mode to ACTIVE)
	uint8_t data_ctrl[2] = {REG_CTRL_REG_1, status_ctrl_reg1}; //Put device in Active Mode
	writeRegs(data_ctrl, 2);
}
bool MMA8451Q::getFF(){
	// activate free fall
  uint8_t data[1] = {};
	readRegs(FF_MT_SRC,data,1);
	return (data[0] & 0b10000000);
	/*uint8_t data2[2] = {FF_MT_THS, 0b00000110};
	writeRegs(data2,2);
	uint8_t data3[2] = {FF_MT_COUNT, 0x06};
	writeRegs(data3,2);*/
}
