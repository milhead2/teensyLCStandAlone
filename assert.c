#include "kinetis.h"
#include "core_pins.h" 
#include "assert.h"

#define LED  (1<<5)
#define CS   (1<<1)
#define CLK  (1<<0)
#define MOSI (1<<0)

#define SET_LED(x)  do{ ((x) ? (GPIOC_PSOR = LED) : (GPIOC_PCOR = LED )); spinDelayUs(1); } while(0)
#define SET_CS(x)   do{ ((x) ? (GPIOC_PSOR = CS)  : (GPIOC_PCOR = CS  )); spinDelayUs(1); } while(0)
#define SET_CLK(x)  do{ ((x) ? (GPIOC_PSOR = CLK) : (GPIOC_PCOR = CLK )); spinDelayUs(1); } while(0)
#define SET_MOSI(x) do{ ((x) ? (GPIOC_PSOR = MOSI): (GPIOC_PCOR = MOSI)); spinDelayUs(1); } while(0)

int isIntContext(void)
{
    int res = 0;
    __asm ("mrs    r0, iapsr\n\t"
           "mov    %[result], r0"
           : [result]"=r" (res) /* 'result' is output */
           :                    /* No input. */
           : "r0"               /* r0 was clobbered */
    );
    
    return (res & 0x000001ff);
}

void  
spinDelayUs(uint32_t us)
{
    while(us--)
    {
        __asm("    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n");
    }
}

void 
spinDelayMs(uint32_t ms)
{
    while(ms--)
    {
        spinDelayUs(1000);
    }
}

#define THROTTLE spinDelayUs(10)

uint8_t transfer(uint8_t out)
{
    uint8_t count, in = 0;

    SET_CLK(0); 
    THROTTLE;
    for (count = 0; count < 8; count++)
    {
        in <<= 1;
        SET_MOSI(out & 0x80);   // set Output bit
        THROTTLE;
        SET_CLK(1);             // Clock Rising Edge
        THROTTLE;
        in += 0;            // normally read MISO here.
        SET_CLK(0);             // Clock Rising Edge
        THROTTLE;
        out <<= 1;          // shift read bit
    }
    SET_MOSI(0); 
    THROTTLE;

    return (in);
}


void 
_assert_failed (const char *assertion, const char *file, unsigned int line)
{
    // Not alot that we can do at the current time so simply blink the 
    // LED rapidly
    //
    // Normally an IO would display:
    //   Assertion failed: expression, file filename, line line number 
	
    if ( ! isIntContext() )
    {
        // disable interrupts and task switching
    }
    
        
    // Denergize any outputs
    
    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    
    //
    // Do a dummy read to insert a few cycles after enabling the peripheral.
    //

    //
    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //

    // turn off all the LED's
    //
    
    PORTC_PCR5 = (1<<6) | (1<<8);
    PORTD_PCR1 = (1<<6) | (1<<8);
    PORTC_PCR0 = (1<<6) | (1<<8);
    PORTB_PCR0 = (1<<6) | (1<<8);

    GPIOB_PDDR = MOSI;
    GPIOB_PDIR = 0;
    GPIOD_PDDR = CS;
    GPIOD_PDIR = 0;
    GPIOC_PDDR = LED | CLK;
    GPIOC_PDIR = 0;

    SET_CLK(0); 
    SET_CS(1);

    //
    // Loop forever.
    //
    while (1) 
    {
        SET_LED(1);
        spinDelayMs(50);
        SET_LED(0);
        spinDelayMs(50);

        SET_CS(0);
        transfer(0xbc);
        SET_CS(1);
        spinDelayMs(50);
    }
}
               
