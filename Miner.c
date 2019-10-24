/*
 * Ver: Miner cut open demo
 * Author: YG Bit Robots
 * main.c
 */

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_flash.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"
#include "inc/hw_adc.h"
#include "inc/hw_nvic.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "utils/ustdlib.h"

#include <string.h>
#include <stdlib.h>

 //--------
#include "pinout.h"
#include "port_pin_map.h"
#include "CPU_init.h"

//---------
#include "Libraries/motors.h"
#include "Libraries/pitches.h"
#include "Libraries/serial_comm.h"
#include "Libraries/oneshot_timer.h"
#include "Libraries/Encoders.h"
#include "Libraries/ir_module.h"
#include "Libraries/lcd_SPI_drv.h"
#include "Libraries/Glcd.h"
#include "Libraries/Fonts/splash_screen.h"
//--------

//---------
#include "MPU6050/motion.h"


#define SENSORS_NR		6
#define PWM_FREQ		75000


///////////////////////////////////////////////////////////////////////////
//----------------------------------- PID ---------------------------------
///////////////////////////////////////////////////////////////////////////

#define WKP					5
#define WKD					80
#define WKI					0.01

#define DISTANCE			245			//150mm
//#define DISTANCE			163			//100mm
#define STEPK				3.26		//	1.63

//----------------  40mm Wheels / 0.36 / 6 / 115 / ---------------
#define LINE_TRESHOLD   25    //
#define SENSOR_TRESHOLD 10    //

//------------------  1000 rpm -------------
#define KP             54 //52
#define KD             36
#define KI             0

//-------------------------------------------
#define MAX_SPEED      70
#define ACQUIRE_SPEED  60
#define RAMP_SPEED     60

// Two speed for turn - for making right turn radius
#define TURN_SPEED_HIGH		115		// % from max speed
#define TURN_SPEED_LOW      25

//
#define TURN_ERROR_LEFT    100    //  100 minimum
#define TURN_ERROR_RIGHT   600    // 600 maximum
#define TIME_DELAY			10
#define MIN_DISTANCE 		8

//---------------------------------------------
#define CROSS_MASK          0b011110
#define LEFT_TURN_MASK      0b011100
#define RIGHT_TURN_MASK     0b001110
#define MERGE_RIGHT_MASK    0b001101
#define MERGE_LEFT_MASK     0b101100

//---------------- Sensors and position ----3-----
typedef enum {
	STOPPED, TURN_RIGHT, TURN_LEFT, TURN, FORWARD, STOPPING, DISCOVER, CROSS, SKIP, SKIP_LINK, TRANSLATE_ORDER, SLOWDOWN
} E_STATE;

#define CENTER_POS 	2

//------------  Robot 6 Leds -------------
const unsigned int sensors[SENSORS_NR] = { ADC_CTL_CH9, ADC_CTL_CH8, ADC_CTL_CH7, ADC_CTL_CH6, ADC_CTL_CH5, ADC_CTL_CH4 }; //left-right

unsigned int sensor_values[SENSORS_NR];
unsigned int sensor_calmin[SENSORS_NR];
unsigned int sensor_calmax[SENSORS_NR];
unsigned int sensor_denom[SENSORS_NR];
unsigned int left_cal, center_cal, right_cal, l_edge_cal, r_edge_cal;

//--------- Common ------------
E_STATE state;
E_STATE nextState;
int position;
static unsigned int last_pos = 350;
volatile unsigned char on_line;
signed int error;
signed int lastError;
signed int prev_error1;
signed int prev_error2;
signed int derivate;
signed long integral = 0;
unsigned int delta_i;
unsigned int delta_d;
signed int proportional, derivate;
signed int control_valueX;

//  Rotational error - angular error from Gyro  - W = omega
signed int errorW;
signed int errors_sumW;
signed int lastErrorW;
signed int proportionalW = 0;
signed long integralW = 0;
signed long derivateW = 0;
signed int control_valueW;
signed int speedW;
//--------------------
int lAngle;
signed int turn;
signed int cycle_counter;

//------------
signed int speed;
signed int left_pwm;
signed int right_pwm;
float speed_ramp;
float current_speed;
signed int speed_setpoint;
signed int speedX;
signed int distance;

//-----------------
//unsigned long fast_speed;
unsigned long starting_speed;
unsigned long stopping_speed;
unsigned long turn_speed;
unsigned long uturn_speed;
unsigned long ramp;
unsigned char rule;
char ramp_pass = 0;
unsigned char avoidance_step;

signed int left_pwm;
signed int right_pwm;
//------------------------------

const char sens_bitmap[6] = { 0x20, 0x10, 0x8, 0x04, 0x02, 0x01;
int line_map;
int discover_map;
int direction;

char slow_down_flag;
const char SET_SPD_CMD[] = { 'S' , 'P', 'D', 'S', 'E', 'T', '\0' };
const char SET_KP_CMD[] = { 'K' , 'P', 'S', 'E', 'T', '\0' };
const char SET_KD_CMD[] = { 'K' , 'D', 'S', 'E', 'T', '\0' };
const char SET_DIST_CMD[] = { 'D', 'I', 'S', 'T', '\0' };
const char SET_MAP_CMD[] = "M2";
const char START_CMD[] = { 'B', 'E', 'G', 'I', 'N', '\0' };
const char CALIBRATE_CMD[] = { 'C', 'A', 'L', 'I', 'B', 'R', 'A', 'T', 'E', '\0' };
const char POWER_SAVE_M[] = "EMERGENCY3";
const char RETURN_HOME_CMD[] = "REVERSEPATH";
const char

int path[322];

unsigned char path_length;
unsigned int uiDelta_send_info;
unsigned int uiDelta_get_container;
unsigned char string_length = 0;
char input_string[256];
unsigned char road;

ST_DEV_SETTINGS dev_settings

//--------------------- Macros definitions -----------
#define delay(ms); gulTickCountDown = ms; while (gulTickCountDown);

//------------------- Functions prototypes ---------------------
unsigned int read_position(void);
void callibrate(void);

void sharp_init(void);
unsigned int sharp_distance(void);
void display_values(void);
void get_string(void);
//--------------------------------------------------------------------------------------------------
//                    ADC read
//--------------------------------------------------------------------------------------------------

//*****************************************************************************
//
// Handles the SysTick timeout interrupt.
//
//*****************************************************************************
void SysTickIntHandler(void) {
	static unsigned long ulDelta;
	static char led_fl;
	// countdown timer
	if (gulTickCountDown)
		gulTickCountDown--;

	gulTimeTick++;

	//------------------ 1s  tasks -----------------
	if (++ulDelta == 3000) {	// update g_ulSysTickCount 100 times per second
		ulDelta = 0;

		if (!led_fl) {
			ROM_GPIOPinWrite(LED1_PORT, LED1, LED1);
			led_fl = 1;
		}
		else {
			ROM_GPIOPinWrite(LED1_PORT, LED1, 0);
			led_fl = 0;
		}

	}
}

//==============================================================================
int main(void) {
	char tmp_str[256];
	char tmp_char;
	int i;
	long tmp_long;

	// -------- CTCLOCK SEUP --------
	// Enable processor interrupts.
	//---------------------------------------------

	//-------------------- SETUP ------------------
	PWMSetupLeft(255, 1, PWM_FREQ);
	PWMSetupRight(255, 1, PWM_FREQ);

	// -------- Tone init --------

	//----------------- EEPROM init --------------------

	//------------------ LCD -----------------------

	//---------------  Resore saved state ----------
	//	if (dev_settings_chk()) {
	//		dev_factory_settings();
	//	}

		//------------  Encoders Init ----------------

	//---------- MPU6050 init -----------
	//MotionInit();

	//------------------------------------------
	//SerialPrint("MPU init\r\n");

	//-------- Sharp input init -------
	sharp_init();

	//--------- IR remote init ----------
	ir_init();

	//    delay(50000);
		//--------------
	uiDelta_send_info = 0;
	//---------------- Wait button or remote key pressed for callibrate  ------------------

	//    /*
	while (1) {
		if (pckg_received) {
			get_string();
			if (input_string[0] == CALIBRATE_CMD[0]) {
				tmp_char = 1;
				for (i = 1; i < strlen(input_string); i++) 
					if (input_string[i] == CALIBRATE_CMD[i])
						tmp_char++;
				}
				if (tmp_char == strlen(input_string)) {
					pckg_received = 0;
					break;
				}
			}
		}
	}

	callibrate();
	LcdErase();
	SerialPrint("R1 started");
	//    delay(5000);
	while (1) {
		brick_distance = sharp_distance();
		position = read_position();
		display_values();

		if (pckg_received == 1) {
			get_string();
			if (input_string[0] == START_CMD[0]) {
				tmp_char = 1;
				for (i = 1; i < strlen(input_string); i++) {
					if (input_string[i] == START_CMD[i])
						tmp_char++;
				}
				if (tmp_char == strlen(input_string)) {
					SerialPrint("R2 Setup");
					pckg_received = 0;
					break;
				}
			}
		}
		delay(100);
	}

	//----------
	state = WAIT_ORDER;
	lastError = 0;
	integral = 0;


	//==========================  Starting =====================


	speed = MAX_SPEED;
	//	rule = 1;			
	//	avoidance_step = 0;
	//=================================================================================

	//=================================================================================
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< !!!! >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	//=================================================================================

	path_length = 0;
	//====================================================================================
	//--------------------------------------- LOOP ---------------------------------------
	//=====================================================================================
	while (1) {

		position = read_position();
		error = position - 350;
		//--------- Liear PID ------------
		proportional = dev_settings.kp * error;
		proportional = proportional / 100;

		derivate = (error - lastError) * dev_settings.kd;
		derivate = derivate / 10;
		lastError = error;
		prev_error1 = error;


		control_valueX = proportional + derivate;
		if (control_valueX > 90)
			control_valueX = 90;
		if (control_valueX < -90)
			control_valueX = -90;

		//-------- Read sharp sensor -----
		distance = sharp_distance();

		switch (state) {
		case TRANSLATE_ORDER:
			left_pwm = 0;
			right_pwm = 0;

			display_values();

			if (pckg_received == 1) {
				get_string();
				tmp_char = 0;

				// ! check if cmd starts with and read !
				if (input_string[0] == RAND_CMD[0]) {
					while (input_string[tmp_char] != '\0' && input_string[tmp_char] == RAND_CMD[tmp_char++]);

					if (tmp_char == strlen(RAND_CMD)) {
						rand_func();
						input_string[0] = '\0';
						last_state = 0;
						state = TRANSLATE_ORDER;
					}

				}

				if (uiDelta_send_info-- == 0) {
					uiDelta_send_info = 1000;
					SerialPrint("ST2 available");
				}
			}

			delay(100);
			break;

		case FORWARD:

			if (distabce < MIN_DISTANCE) {
				MotorLeftSpeed(0);
				MotorRightSpeed(0);
				delay(2000);
			}

			left_pwm = speed - control_valueX;
			right_pwm = speed + control_valueX;

			//check event
			if ((line_map & 0b100000) && !(line_map & 0b010000) && ((line_map & 0b001100) >= 0b000100)) {
				cycle_counter = 0;
				state = DISCOVER;
			}
			else if ((line_map & 0b000001) && !(line_map & 0b000010) && ((line_map & 0b001100) >= 0b000100)) {
				cycle_counter = 0;
				state = DISCOVER;
			}
			else if ((on_line > 2) && ((line_map & 0b001100) == 0b001100)) {
				cycle_counter = 0;
				state = DISCOVER;
			}

			//----------------------------------------------------

			break;

		case DISCOVER:
			left_pwm = speed - control_valueX;
			right_pwm = speed + control_valueX;
			if (++cycle_counter > 25) {
				//display_measures();
				// check event slow down 
				if (((line_map & 0b100001) == 0b100001) && !(line_map & 0b010010) && ((line_map & 0b001100) >= 0b000100)) {
					//----- RAMP MARK--------
					if (slow_down_flag == 0) {
						speed = RAMP_SPEED;
						slow_down_flag = 1;
					}
					else {
						speed = MAX_SPEED;
						slow_down_flag = 0;
					}
					state = SKIP_LINK;
				}
				else if ((line_map & CROSS_MASK) == CROSS_MASK) { // check event cross line 
					if (slow_down_flag) {
						speed = MAX_SPEED;
						slow_down_flag = 0;
						state = SKIP_LINK;
					}
					cycle_counter = 0;
					state = SKIP;
					road += TURN_LEFT;
					road += TURN_RIGHT;
				}
				else if (line_map == LEFT_TURN_MASK) { // check event left turn 
					road += TURN_LEFT;
					state = SKIP;
					cycle_counter = 0;
				}
				else if (line_map == RIGHT_TURN_MASK) { //// check event right turn 
					road += TURN_RIGHT;
					//                        state = SKIP;
					cycle_counter = 0;
				}
				else if ((line_map & MERGE_RIGHT_MASK) >= 0b00000101) {		// check event merge
					cycle_counter = 0;
					state = SKIP;
				}
				else if ((line_map & MERGE_LEFT_MASK) >= 0b101000) { // check event merge 
					cycle_counter = 0;
					state = SKIP;
				}
				else {
					state = SKIP;
				}


				if (road != 0) {
					switch (path[path_length]) {
					case FORWARD:
						state = SKIP_LINK;
						path_length++;
						break;
					case TURN_LEFT:
						if (road & TURN_LEFT) {
							state = TURN_LEFT;
							path_length++;
						}
						break;
					case TURN_RIGHT:
						if (road & TURN_RIGHT) {
							state = TURN_RIGHT;
							path_length++;
						}
						break;
					}
					sprintf(tmp_str, "R2 Step #%c last step %c", path_length[], path[path_length])
					SerialPrint(tmp_str)
				}
				road = 0;

			}
			else {
				if (((line_map & 0b100001) == 0b100001) && !(line_map & 0b010010) && ((line_map & 0b001100) >= 0b000100)) {
					//----- RAMP MARK--------
					if (slow_down_flag == 0) {
						speed = RAMP_SPEED;
						slow_down_flag = 1;
					}
					else {
						speed = MAX_SPEED;
						slow_down_flag = 0;
					}
					state = SKIP_LINK;
				}
			}

			break;

		case TURN_LEFT:
			left_pwm = TURN_SPEED_LOW;
			right_pwm = TURN_SPEED_HIGH;
			if ((line_map == 0b001000) || (line_map == 0b000100) || (line_map == 0b001100)) {
				state = FORWARD;
				integral = 0;
			}
			break;
		case TURN_RIGHT:
			left_pwm = TURN_SPEED_HIGH;
			right_pwm = TURN_SPEED_LOW;
			if ((line_map == 0b001000) || (line_map == 0b000100) || (line_map == 0b001100)) {
				state = FORWARD;
				integral = 0;
			}
			break;

		case SKIP:
			if (++cycle_counter > 60) {
				if ((line_map & 0b111111) == 0b111111)
					state = STOPPED;
			}

			left_pwm = speed - control_valueX;
			right_pwm = speed + control_valueX;

			if ((line_map == 0b001000) || (line_map == 0b000100) || (line_map == 0b001100)) {
				state = FORWARD;
				integral = 0;
			}
			if (((line_map & 0b100001) == 0b100001) && !(line_map & 0b010010) && ((line_map & 0b001100) >= 0b000100)) {
				//----- RAMP MARK--------
				if (slow_down_flag == 0) {
					speed = RAMP_SPEED;
					slow_down_flag = 1;
				}
				else {
					speed = MAX_SPEED;
					slow_down_flag = 0;
				}
				state = SKIP_LINK;
			}

			break;
		case SKIP_LINK:
			left_pwm = speed - control_valueX;
			right_pwm = speed + control_valueX;

			if ((line_map == 0b001000) || (line_map == 0b000100) || (line_map == 0b001100)) {
				state = FORWARD;
				integral = 0;
			}
			break;

		case CROSS:
			// if line isnt crossed
			if (((line_map & 0b100000) == 0b100000) && ((line_map & 0b000001) == 0b000001)) {
				left_pwm = speed + control_valueW;
				right_pwm = speed - control_valueW;
				discover_map |= line_map;
			}
			else {
				state = DISCOVER;
			}
			break;

		case STOPPED:
			left_pwm = 0;
			right_pwm = 0;
			MotorLeftSpeed(left_pwm);
			MotorRightSpeed(right_pwm);
			delay(30000);
			state = TRANSLATE_ORDER;
			break;
		}

		if (pckg_received) {
			left_pwm = 0;
			right_pwm = 0;

			last_state = state;

			state = TRANSLATE_ORDER;
		}

		MotorLeftSpeed(left_pwm);
		MotorRightSpeed(right_pwm);
		delay(5);
	}
}

//==========================  CALIBRATE ==========================
void callibrate(void) {
	int i, sens;
	int center_pos;
	unsigned int tmp_value;

	for (i = 0; i < SENSORS_NR; i++)
		(sensor_calmin)[i] = 4095;      // Maximum ADC value

	for (i = 0; i < SENSORS_NR; i++) {
		(sensor_calmax)[i] = 0;         // Minimum ADC value
	}

	MotorLeftSpeed(ACQUIRE_SPEED);
	MotorRightSpeed(-ACQUIRE_SPEED);

	for (i = 0; i < 550; i++) {

		//----- Find minimum and maximum values for all sensors -----
		for (sens = 0; sens < SENSORS_NR; sens++) {
			ROM_GPIOPinWrite(PING_PORT, PING1, PING1);  //LEDS ON
			delay(4);

			tmp_value = analogRead(sensors[sens]);
			if (tmp_value < sensor_calmin[sens])
				sensor_calmin[sens] = tmp_value;
			if (tmp_value > sensor_calmax[sens])
				sensor_calmax[sens] = tmp_value;
		}

		if (i == 180) {
			MotorLeftSpeed(-ACQUIRE_SPEED);
			MotorRightSpeed(ACQUIRE_SPEED);
		}
		ROM_GPIOPinWrite(PING_PORT, PING1, 0);  //LEDS Off
		delay(4);
	}

	//-------   Calibrate denom --------
	for (sens = 0; sens < SENSORS_NR; sens++) {
		sensor_denom[sens] = (sensor_calmax[sens] - sensor_calmin[sens]) / 10;
	}

	MotorLeftSpeed(0);
	MotorRightSpeed(0);
	ROM_GPIOPinWrite(PING_PORT, PING1, 0);  //LEDS OFF
}

//============================= Read sensors  and scale =====================
unsigned int read_position(void) {
	//    unsigned char on_line;
	//static unsigned int last_pos = 350;
	signed char sens;
	unsigned int tmp_value;
	unsigned int sensors_sum;
	unsigned int pos;
	line_map = 0;
	pos = 0;
	sensors_sum = 0;
	on_line = 0;

	//-------------- Read  Odd sensors ------------
	ROM_GPIOPinWrite(PING_PORT, PING1, PING1);  //LEDS ON
	delay(4);

	for (sens = 0; sens < SENSORS_NR / 2; sens++) {
		tmp_value = analogRead(sensors[sens]);

		//--------- Validate ----------
		if (tmp_value < sensor_calmin[sens])
			tmp_value = sensor_calmin[sens];
		if (tmp_value > sensor_calmax[sens])
			tmp_value = sensor_calmax[sens];

		sensor_values[sens] = ((tmp_value - sensor_calmin[sens]) * 10) / sensor_denom[sens];

		//----------- Noise filtering ----------
		if (sensor_values[sens] < SENSOR_TRESHOLD) {
			sensor_values[sens] = 0;
		}

		sensors_sum += sensor_values[sens];
		pos += sensor_values[sens] * ((sens + 1) * 100);

		//--- line presens check ---
		if (sensor_values[sens] > LINE_TRESHOLD) {
			on_line += 1;
			line_map += sens_bitmap[sens];
		}
	}
	ROM_GPIOPinWrite(PING_PORT, PING1, 0);  //LEDS OFF
	delay(4);

	//-------------- Read Even sensors ------------
	ROM_GPIOPinWrite(PING_PORT, PING1, PING1);  //LEDS ON
	delay(4);
	for (sens = SENSORS_NR - 1; sens >= SENSORS_NR / 2; sens--) {
		tmp_value = analogRead(sensors[sens]);

		//--------- Validate ----------
		if (tmp_value < sensor_calmin[sens])
			tmp_value = sensor_calmin[sens];
		if (tmp_value > sensor_calmax[sens])
			tmp_value = sensor_calmax[sens];

		//-------- Calibrate ----------
		sensor_values[sens] = ((tmp_value - sensor_calmin[sens]) * 10) / sensor_denom[sens];

		//----------- Noise filtering ----------
		if (sensor_values[sens] < SENSOR_TRESHOLD) {
			sensor_values[sens] = 0;
		}

		sensors_sum += sensor_values[sens];
		pos += sensor_values[sens] * ((sens + 1) * 100);

		if (sensor_values[sens] > LINE_TRESHOLD) {
			on_line += 1;
			line_map += sens_bitmap[sens];
		}
	}

	ROM_GPIOPinWrite(PING_PORT, PING1, 0);  //LEDS OFF

	if (!on_line) {
		// If it last read to the left of center, return minimum value.
		if (last_pos < 250) {
			last_pos = 90;
			// If it last read to the right of center, return the max.
		}
		else if (last_pos > 450) {
			last_pos = (SENSORS_NR * 100) + 10;
		}
		else {
			last_pos = 350;   // center pos
		}
	}
	if (on_line > 2) {
		last_pos = 350;     //center pos
	}
	else
		last_pos = pos / sensors_sum;

	return last_pos;
}


void sharp_init(void) {
	/// Enable port PE5 for ADC AIN8 - ADC_CTL_CH8
	ROM_GPIOPinTypeADC(SHARP_PORT, SHARP_PIN);
	ROM_GPIOPadConfigSet(SHARP_PORT, SHARP_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_ANALOG);
	ROM_GPIOPinWrite(SHARP_PORT, SHARP_PIN, 0);
}

unsigned int sharp_distance(void) {
	uint32_t ulADC0_Value[1];

	return 16136 / ulADC0_Value[0]; // A41
//return 33513 / ulADC0_Value[0];			// A21
}

void display_values(void) {
	char tmp_str[64];
	usprintf(tmp_str, "Line %c %c %c %c %c %c", B_LINE_MAP(line_map));
	LcdTextXY(0, 1, BOLD_FNT, NORM_LINE, tmp_str);

	usprintf(tmp_str, "position  %d", position);
	LcdTextXY(0, 2, BOLD_FNT, NORM_LINE, tmp_str);

	usprintf(tmp_str, "Left %d %d %d ", sensor_values[0], sensor_values[1], sensor_values[2]);
	LcdTextXY(0, 3, BOLD_FNT, NORM_LINE, tmp_str);
	usprintf(tmp_str, "Right %d %d %d", sensor_values[5], sensor_values[4], sensor_values[3]);
	LcdTextXY(0, 4, BOLD_FNT, NORM_LINE, tmp_str);
	usprintf(tmp_str, "Distance  %d", distance);
	LcdTextXY(0, 5, BOLD_FNT, NORM_LINE, tmp_str);
	LcdUpdate();
}

void get_string(void) {
	char i = 0;
		//------------ Get str -------------
	if (pckg_received && (ulRDptr0 != ulWRptr0)) {
		i = 0;
		while ((ulRDptr0 != ulWRptr0)) {
			input_string[i] = rxbuff0[ulRDptr0];
			if (++ulRDptr0 > BUFFSIZE)
				ulRDptr0 = 0;
			i++;
		}
		input_string[i] = 0;
		//        tmp_str = inp_string_map;
		LcdTextXY(0, 0, BOLD_FNT, NORM_LINE, input_string);
		//----------- Echo on serial channel 0 --------
//        strcat(inp_string_map, "<<<");
		SerialPrint(input_string);
	}
}
