/* 
 * File:   Blinky.c
 * Author: john
 *
 * Created on 29 March 2024, 21:11
 */

// PIC32MX250F256L Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config FUSBIDIO = OFF           // USB USID Selection (Controlled by Port Function)
#pragma config FVBUSONIO = OFF          // USB VBUS ON Selection (Controlled by Port Function)

// DEVCFG2
#pragma config FPLLIDIV = DIV_4         // PLL Input Divider (4x Divider)
#pragma config FPLLMUL = MUL_24         // PLL Multiplier (24x Multiplier)
#pragma config UPLLIDIV = DIV_4         // USB PLL Input Divider (4x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2)

// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = HS             // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (Communicate on PGEC2/PGED2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <sys/attribs.h>


#define FPBCLK  (48000000)      // PBCLK frequency is 48MHz


volatile uint32_t MilliSeconds = 0;


/* delayms --- busy-wait delay for given number of milliseconds */

static void delayms(const uint32_t interval)
{
    const uint32_t now = MilliSeconds;
    
    while ((MilliSeconds - now) < interval)
        ;
}


/* millis --- Arduino-like function to return milliseconds since start-up */

static uint32_t millis(void)
{
    return (MilliSeconds);
}


void __ISR(_TIMER_1_VECTOR, ipl7AUTO) Timer1Handler(void)
{
    MilliSeconds++;
    
    LATDINV = _LATD_LATD11_MASK;    // Toggle RD11, P3 pin 21 (500Hz)
    
    IFS0CLR = _IFS0_T1IF_MASK;      // Clear Timer 1 interrupt flag
}


/* initMCU --- set up the microcontroller in general */

void initMCU(void)
{
    /* Configure interrupts */
    INTCONSET = _INTCON_MVEC_MASK; // Multi-vector mode
}


/* initGPIOs --- set up the GPIO pins */

static void initGPIOs(void)
{
    TRISBbits.TRISB9 = 0;   // LED on RB9
    TRISDbits.TRISD11 = 0;  // SQWAVE on RD11
}


/* initMillisecondTimer --- set up a timer to interrupt every millisecond */

void initMillisecondTimer(void)
{
    /* Configure Timer 1 for 1kHz/1ms interrupts */
    T1CONbits.TCKPS = 0;        // Timer 1 prescale: 1
    
    TMR1 = 0x00;                // Clear Timer 1 counter
    PR1 = (FPBCLK / 1000) - 1;  // Interrupt every millisecond (1kHz)
    
    IPC1bits.T1IP = 7;          // Timer 1 interrupt priority 7 (highest)
    IPC1bits.T1IS = 1;          // Timer 1 interrupt sub-priority 1
    IFS0CLR = _IFS0_T1IF_MASK;  // Clear Timer 1 interrupt flag
    IEC0SET = _IEC0_T1IE_MASK;  // Enable Timer 1 interrupt
    
    T1CONSET = _T1CON_ON_MASK;  // Enable Timer 1
}


int main(void)
{
    initMCU();
    initGPIOs();
    initMillisecondTimer();
    
    __builtin_enable_interrupts();     // Global interrupt enable
    
    while (1)
    {
        LATBbits.LATB9 = 1;
        
        delayms(500);
         
        LATBbits.LATB9 = 0;
        
        delayms(500);
    }
    
    return (EXIT_SUCCESS);
}

