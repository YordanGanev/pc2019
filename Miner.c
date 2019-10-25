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



