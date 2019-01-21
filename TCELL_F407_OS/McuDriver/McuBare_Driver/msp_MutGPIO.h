#ifndef _MSP_MUTGPIO_H_
#define _MSP_MUTGPIO_H_

#include "sys_Platform.h"
#include "msp_GPIO.h"

typedef struct
{
	MSP_General_Gpio Red;
	MSP_General_Gpio Green;
	MSP_General_Gpio Blue;	
}MSP_TricolorGpio;

#endif
