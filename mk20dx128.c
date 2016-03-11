#include "kinetis.h"
#include "core_pins.h" 
#include "assert.h"

extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

extern void __libc_init_array(void);
extern int main (void);

void ResetHandler(void);

void fault_isr(void)
{
    PORTC_PCR5 = (1<<6) | (1<<8);
    GPIOC_PDDR = (1<<5);
    GPIOC_PDIR = 0;

    while (1) 
    {
        GPIOC_PDOR = (1<<5);
        spinDelayMs(50);
        GPIOC_PDOR = 0;
        spinDelayMs(50);
    }
}

extern volatile uint32_t systick_millis_count;
void systick_isr(void)
{
	systick_millis_count++;
}

#define INIT_STACK_SIZE 256
uint32_t _estack[INIT_STACK_SIZE];

__attribute__ ((section(".dmabuffers"), used, aligned(256)))
void (* _VectorsRam[NVIC_NUM_INTERRUPTS+16])(void);

__attribute__ ((section(".vectors"), used))
void (* const _VectorsFlash[NVIC_NUM_INTERRUPTS+16])(void) =
{
	(void (*)(void))((uint32_t)&_estack + sizeof(_estack)),	//  0 ARM: Initial Stack Pointer
	ResetHandler,				//  1 ARM: Initial Program Counter
	fault_isr,					//  2 ARM: Non-maskable Interrupt (NMI)
	fault_isr,					//  3 ARM: Hard Fault
	fault_isr,				    //  4 ARM: MemManage Fault
	fault_isr,					//  5 ARM: Bus Fault
	fault_isr,				    //  6 ARM: Usage Fault
	fault_isr,					//  7 --
	fault_isr,					//  8 --
	fault_isr,					//  9 --
	fault_isr,					// 10 --
	fault_isr,					// 11 ARM: Supervisor call (SVCall)
	fault_isr,				    // 12 ARM: Debug Monitor
	fault_isr,					// 13 --
	fault_isr,				    // 14 ARM: Pendable req serv(PendableSrvReq)
	systick_isr,				// 15 ARM: System tick timer (SysTick)
	fault_isr,					// 16 DMA channel 0 transfer complete
	fault_isr,					// 17 DMA channel 1 transfer complete
	fault_isr,					// 18 DMA channel 2 transfer complete
	fault_isr,					// 19 DMA channel 3 transfer complete
	fault_isr,					// 20 --
	fault_isr,					// 21 Flash Memory Command complete
	fault_isr,				    // 22 Low-voltage detect/warning
	fault_isr,					// 23 Low Leakage Wakeup
	fault_isr,					// 24 I2C0
	fault_isr,					// 25 I2C1
	fault_isr,					// 26 SPI0
	fault_isr,					// 27 SPI1
	fault_isr,				    // 28 UART0 status & error
	fault_isr,				    // 29 UART1 status & error
	fault_isr,				    // 30 UART2 status & error
	fault_isr,					// 31 ADC0
	fault_isr,					// 32 CMP0
	fault_isr,					// 33 FTM0
	fault_isr,					// 34 FTM1
	fault_isr,					// 35 FTM2
	fault_isr,					// 36 RTC Alarm interrupt
	fault_isr,				    // 37 RTC Seconds interrupt
	fault_isr,					// 38 PIT Both Channels
	fault_isr,					// 39 I2S0 Transmit & Receive
	fault_isr,					// 40 USB OTG
	fault_isr,					// 41 DAC0
	fault_isr,					// 42 TSI0
	fault_isr,					// 43 MCG
	fault_isr,					// 44 Low Power Timer
	fault_isr,					// 45 Software interrupt
	fault_isr,					// 46 Pin detect (Port A)
	fault_isr,					// 47 Pin detect (Port C and D)
};


#if 0
__attribute__ ((section(".flashconfig"), used))
const uint8_t flashconfigbytes[16] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF
};
#endif

#ifdef __clang__
// Clang seems to generate slightly larger code with Os than gcc
__attribute__ ((optimize("-Os")))
#else
__attribute__ ((section(".startup"),optimize("-Os")))
#endif

void ResetHandler(void)
{
	uint32_t *src = &_etext;
	uint32_t *dest = &_sdata;


#ifdef KINETISK
	WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
	WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
#endif

	// enable clocks to always-used peripherals
	SIM_SCGC4 = SIM_SCGC4_USBOTG | 0xF0000030;
	SIM_SCGC5 = 0x00003F82;		// clocks active to all GPIO
	SIM_SCGC6 = SIM_SCGC6_ADC0 | SIM_SCGC6_TPM0 | SIM_SCGC6_TPM1 | SIM_SCGC6_TPM2 | SIM_SCGC6_FTFL;

	// release I/O pins hold, if we woke up from VLLS mode
	if (PMC_REGSC & PMC_REGSC_ACKISO) 
        PMC_REGSC |= PMC_REGSC_ACKISO;

    // since this is a write once register, make it visible to all F_CPU's
    // so we can into other sleep modes in the future at any speed
	SMC_PMPROT = SMC_PMPROT_AVLP | SMC_PMPROT_ALLS | SMC_PMPROT_AVLLS;

	// hardware always starts in FEI mode
	//  C1[CLKS] bits are written to 00
	//  C1[IREFS] bit is written to 1
	//  C6[PLLS] bit is written to 0

// MCG_SC[FCDIV] defaults to divide by two for internal ref clock
// I tried changing MSG_SC to divide by 1, it didn't work for me

#if F_CPU <= 2000000
    #if defined(KINETISK)
    MCG_C1 = MCG_C1_CLKS(1) | MCG_C1_IREFS;
    #elif defined(KINETISL)
	// use the internal oscillator
	MCG_C1 = MCG_C1_CLKS(1) | MCG_C1_IREFS | MCG_C1_IRCLKEN;
    #endif
	// wait for MCGOUT to use oscillator
	while ((MCG_S & MCG_S_CLKST_MASK) != MCG_S_CLKST(1)) ;
	for (n=0; n<10; n++) ; // TODO: why do we get 2 mA extra without this delay?
	MCG_C2 = MCG_C2_IRCS;
	while (!(MCG_S & MCG_S_IRCST)) ;
	// now in FBI mode:
	//  C1[CLKS] bits are written to 01
	//  C1[IREFS] bit is written to 1
	//  C6[PLLS] is written to 0
	//  C2[LP] is written to 0
	MCG_C2 = MCG_C2_IRCS | MCG_C2_LP;
	// now in BLPI mode:
	//  C1[CLKS] bits are written to 01
	//  C1[IREFS] bit is written to 1
	//  C6[PLLS] bit is written to 0
	//  C2[LP] bit is written to 1
#else
    #if defined(KINETISK)
    // enable capacitors for crystal
    OSC0_CR = OSC_SC8P | OSC_SC2P;
    #elif defined(KINETISL)
    // enable capacitors for crystal
    OSC0_CR = OSC_SC8P | OSC_SC2P | OSC_ERCLKEN;
    #endif
	// enable osc, 8-32 MHz range, low power mode
	MCG_C2 = MCG_C2_RANGE0(2) | MCG_C2_EREFS;
	// switch to crystal as clock source, FLL input = 16 MHz / 512
	MCG_C1 =  MCG_C1_CLKS(2) | MCG_C1_FRDIV(4);
	// wait for crystal oscillator to begin
	while ((MCG_S & MCG_S_OSCINIT0) == 0) ;
	// wait for FLL to use oscillator
	while ((MCG_S & MCG_S_IREFST) != 0) ;
	// wait for MCGOUT to use oscillator
	while ((MCG_S & MCG_S_CLKST_MASK) != MCG_S_CLKST(2)) ;

	// now in FBE mode
	//  C1[CLKS] bits are written to 10
	//  C1[IREFS] bit is written to 0
	//  C1[FRDIV] must be written to divide xtal to 31.25-39 kHz
	//  C6[PLLS] bit is written to 0
	//  C2[LP] is written to 0
  #if F_CPU <= 16000000
	// if the crystal is fast enough, use it directly (no FLL or PLL)
	MCG_C2 = MCG_C2_RANGE0(2) | MCG_C2_EREFS | MCG_C2_LP;
	// BLPE mode:
	//   C1[CLKS] bits are written to 10
	//   C1[IREFS] bit is written to 0
	//   C2[LP] bit is written to 1
  #else
	// if we need faster than the crystal, turn on the PLL
   #if defined(__MK66FX1M0__)
    #if F_CPU == 96000000
	MCG_C5 = MCG_C5_PRDIV0(1);
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(8);
    #else
    #error "MK66FX1M0 only supports 96 MHz so far...."
    #endif
   #else
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(0); // config PLL for 96 MHz output
   #endif


	// wait for PLL to start using xtal as its input
	while (!(MCG_S & MCG_S_PLLST)) ;
	// wait for PLL to lock
	while (!(MCG_S & MCG_S_LOCK0)) ;
	// now we're in PBE mode
  #endif
#endif
	// now program the clock dividers
#if F_CPU == 48000000
	// config divisors: 48 MHz core, 48 MHz bus, 24 MHz flash, USB = 96 / 2
	#if defined(KINETISK)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV2(1) |	 SIM_CLKDIV1_OUTDIV4(3);
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(1);
	#elif defined(KINETISL)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV4(1);
	#endif
#elif F_CPU == 24000000
	// config divisors: 24 MHz core, 24 MHz bus, 24 MHz flash, USB = 96 / 2
	#if defined(KINETISK)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(3) | SIM_CLKDIV1_OUTDIV2(3) |	 SIM_CLKDIV1_OUTDIV4(3);
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(1);
	#elif defined(KINETISL)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(3) | SIM_CLKDIV1_OUTDIV4(0);
	#endif
#elif F_CPU == 16000000
	// config divisors: 16 MHz core, 16 MHz bus, 16 MHz flash
#if defined(KINETISK)
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(0) |	 SIM_CLKDIV1_OUTDIV4(0);
#elif defined(KINETISL)
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV4(0);
#endif
#elif F_CPU == 8000000
	// config divisors: 8 MHz core, 8 MHz bus, 8 MHz flash
#if defined(KINETISK)
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV2(1) |	 SIM_CLKDIV1_OUTDIV4(1);
#elif defined(KINETISL)
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV4(0);
#endif
#elif F_CPU == 4000000
    // config divisors: 4 MHz core, 4 MHz bus, 2 MHz flash
    // since we are running from external clock 16MHz
    // fix outdiv too -> cpu 16/4, bus 16/4, flash 16/4
    // here we can go into vlpr?
	// config divisors: 4 MHz core, 4 MHz bus, 4 MHz flash
#if defined(KINETISK)
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(3) | SIM_CLKDIV1_OUTDIV2(3) |	 SIM_CLKDIV1_OUTDIV4(3);
#elif defined(KINETISL)
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(3) | SIM_CLKDIV1_OUTDIV4(0);
#endif
#elif F_CPU == 2000000
    // since we are running from the fast internal reference clock 4MHz
    // but is divided down by 2 so we actually have a 2MHz, MCG_SC[FCDIV] default is 2
    // fix outdiv -> cpu 2/1, bus 2/1, flash 2/2
	// config divisors: 2 MHz core, 2 MHz bus, 1 MHz flash
#if defined(KINETISK)
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(0) |	 SIM_CLKDIV1_OUTDIV4(1);
#elif defined(KINETISL)
    // config divisors: 2 MHz core, 1 MHz bus, 1 MHz flash
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV4(1);
#endif
#else
#error "Error, F_CPU must be 168, 144, 120, 96, 72, 48, 24, 16, 8, 4, or 2 MHz"
#endif

#if F_CPU == 2000000
	SIM_SOPT2 = SIM_SOPT2_TRACECLKSEL | SIM_SOPT2_CLKOUTSEL(4) | SIM_SOPT2_UART0SRC(3);
#else
    SIM_SOPT2 = SIM_SOPT2_TRACECLKSEL | SIM_SOPT2_CLKOUTSEL(6) | SIM_SOPT2_UART0SRC(2);
#endif
    
#if F_CPU <= 2000000
    // since we are not going into "stop mode" i removed it
	SMC_PMCTRL = SMC_PMCTRL_RUNM(2); // VLPR mode :-)
#endif


 
	// TODO: do this while the PLL is waiting to lock....
	while (dest < &_edata) 
        *dest++ = *src++;

    // zero the bss
    for (dest = &_sbss; dest < &_ebss; dest++)
        *dest = 0;

    // move vector table to RAM so that we can alter it.
    //
    //	for (i=0; i < NVIC_NUM_INTERRUPTS + 16; i++) _VectorsRam[i] = _VectorsFlash[i];
	//for (i=0; i < NVIC_NUM_INTERRUPTS; i++) NVIC_SET_PRIORITY(i, 128);
	//SCB_VTOR = (uint32_t)_VectorsRam;	// use vector table in RAM

	// initialize the SysTick counter
	SYST_RVR = (F_CPU / 1000) - 1;
	SYST_CVR = 0;
	SYST_CSR = SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE;
	SCB_SHPR3 = 0x20200000;  // Systick = priority 32

	//init_pins();
	//__enable_irq();


    
    //__libc_init_array();

    //
    // Try to Drive PortC-5 high so that I can turn on the LED
    //

	//startup_late_hook();
	main();

    // should never return...
    assert(0);
    fault_isr();
}

char *__brkval = (char *)&_ebss;


__attribute__((weak)) 
void __cxa_pure_virtual()
{
	while (1);
}

__attribute__((weak)) 
int __cxa_guard_acquire (char *g) 
{
	return !(*g);
}

__attribute__((weak)) 
void __cxa_guard_release(char *g)
{
	*g = 1;
}

#if 0

int nvic_execution_priority(void)
{
	int priority=256;
	uint32_t primask, faultmask, basepri, ipsr;

	// full algorithm in ARM DDI0403D, page B1-639
	// this isn't quite complete, but hopefully good enough
	__asm__ volatile("mrs %0, faultmask\n" : "=r" (faultmask)::);
	if (faultmask) return -1;
	__asm__ volatile("mrs %0, primask\n" : "=r" (primask)::);
	if (primask) return 0;
	__asm__ volatile("mrs %0, ipsr\n" : "=r" (ipsr)::);
	if (ipsr) {
		if (ipsr < 16) priority = 0; // could be non-zero
		else priority = NVIC_GET_PRIORITY(ipsr - 16);
	}
	__asm__ volatile("mrs %0, basepri\n" : "=r" (basepri)::);
	if (basepri > 0 && basepri < priority) priority = basepri;
	return priority;
}

#endif

