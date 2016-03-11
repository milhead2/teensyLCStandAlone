#include <stdint.h>

#include "kinetis.h"
#include "core_pins.h" 
#include "assert.h"

#define LED  (1<<5)
#define CS   (1<<1) 
#define SET_LED(x)  do{ ((x) ? (GPIOC_PSOR = LED) : (GPIOC_PCOR = LED )); spinDelayUs(1); } while(0)
#define SET_CS(x)   do{ ((x) ? (GPIOC_PSOR = CS)  : (GPIOC_PCOR = CS  )); spinDelayUs(1); } while(0)


extern "C" int main(void)
{
    PORTC_PCR5 = (1<<6) | (1<<8);
    PORTD_PCR1 = (1<<6) | (1<<8);

    GPIOD_PDDR = CS;
    GPIOD_PDIR = 0;
    GPIOC_PDDR = LED;
    GPIOC_PDIR = 0;

	while (1) 
    {
        SET_LED(1);
        SET_CS(1);
		spinDelayMs(500);
        SET_LED(0);
   		spinDelayMs(700);

        SET_LED(1);
        SET_CS(0);

		spinDelayUs(50);
        SET_LED(0);
   		spinDelayUs(50);
        SET_LED(1);
		spinDelayUs(50);
        SET_LED(0);
   		spinDelayUs(50);
        SET_LED(1);
		spinDelayUs(50);
        SET_LED(0);
   		spinDelayUs(50);
        SET_LED(1);
		spinDelayUs(50);
        SET_LED(0);
   		spinDelayUs(50);
        SET_LED(1);
		spinDelayUs(50);
        SET_LED(0);
   		spinDelayUs(50);

	}
}

