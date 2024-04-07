/* Blinky --- blink LED on RB9 of PIC32MX board             2024-03-29 */

// PIC32MX250F256L, PIC32MX570F512L, or PIC32MX795F512L Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#if __PIC32_FEATURE_SET__ == 795
#pragma config FSRSSEL = PRIORITY_7     // SRS Select (SRS Priority 7)
#pragma config FMIIEN = OFF             // Ethernet RMII/MII Enable (RMII Enabled)
#pragma config FETHIO = OFF             // Ethernet I/O Pin Select (Alternate Ethernet I/O)
#pragma config FCANIO = OFF             // CAN I/O Pin Select (Alternate CAN I/O)
#else
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#endif
#pragma config FUSBIDIO = OFF           // USB USID Selection (Controlled by Port Function)
#pragma config FVBUSONIO = OFF          // USB VBUS ON Selection (Controlled by Port Function)

// DEVCFG2
#pragma config FPLLIDIV = DIV_4         // PLL Input Divider (4x Divider)
#if __PIC32_FEATURE_SET__ == 795
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#else
#pragma config FPLLMUL = MUL_24         // PLL Multiplier (24x Multiplier)
#endif
#pragma config UPLLIDIV = DIV_4         // USB PLL Input Divider (4x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled)
#if __PIC32_FEATURE_SET__ == 795
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)
#else
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2)
#endif

// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = HS             // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#if __PIC32_FEATURE_SET__ == 795
#pragma config FPBDIV = DIV_2           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/2)
#else
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#endif
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS8192           // Watchdog Timer Postscaler (1:8192)
#if __PIC32_FEATURE_SET__ != 795
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#endif
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#if __PIC32_FEATURE_SET__ != 795
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)
#endif

// DEVCFG0
#if __PIC32_FEATURE_SET__ == 795
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#else
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#endif
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (Communicate on PGEC2/PGED2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdbool.h>


#if __PIC32_FEATURE_SET__ == 795
#define FPBCLK  (40000000)      // PBCLK frequency is 40MHz
#else
#define FPBCLK  (48000000)      // PBCLK frequency is 48MHz
#endif

#define UART_RX_BUFFER_SIZE  (128)
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
#if (UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK) != 0
#error UART_RX_BUFFER_SIZE must be a power of two and <= 256
#endif

#define UART_TX_BUFFER_SIZE  (128)
#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)
#if (UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK) != 0
#error UART_TX_BUFFER_SIZE must be a power of two and <= 256
#endif

struct UART_RX_BUFFER
{
    volatile uint8_t head;
    volatile uint8_t tail;
    uint8_t buf[UART_RX_BUFFER_SIZE];
};

struct UART_TX_BUFFER
{
    volatile uint8_t head;
    volatile uint8_t tail;
    uint8_t buf[UART_TX_BUFFER_SIZE];
};

struct UART_BUFFER
{
    struct UART_TX_BUFFER tx;
    struct UART_RX_BUFFER rx;
};

// UART buffer
static struct UART_BUFFER U1Buf;

uint32_t SavedRCON;
volatile uint32_t MilliSeconds = 0;
volatile bool Tick = false;


/* millis --- Arduino-like function to return milliseconds since start-up */

static uint32_t millis(void)
{
    return (MilliSeconds);
}


void __ISR(_TIMER_1_VECTOR, ipl7AUTO) Timer1Handler(void)
{
    MilliSeconds++;
    Tick = true;
    
    LATDINV = _LATD_LATD11_MASK;    // Toggle RD11, P3 pin 21 (500Hz)
    
    IFS0CLR = _IFS0_T1IF_MASK;      // Clear Timer 1 interrupt flag
}


void __ISR(_UART_1_VECTOR, ipl1AUTO) UART1Handler(void)
{
#if __PIC32_FEATURE_SET__ == 795
    if (IFS0bits.U1TXIF)
#else
    if (IFS1bits.U1TXIF)
#endif
    {
        if (U1Buf.tx.head != U1Buf.tx.tail) // Is there anything to send?
        {
            const uint8_t tmptail = (U1Buf.tx.tail + 1) & UART_TX_BUFFER_MASK;
            
            U1Buf.tx.tail = tmptail;

            U1TXREG = U1Buf.tx.buf[tmptail];     // Transmit one byte
        }
        else
        {
#if __PIC32_FEATURE_SET__ == 795
            IEC0CLR = _IEC0_U1TXIE_MASK;         // Nothing left to send; disable Tx interrupt
#else
            IEC1CLR = _IEC1_U1TXIE_MASK;         // Nothing left to send; disable Tx interrupt
#endif
        }
        
#if __PIC32_FEATURE_SET__ == 795
        IFS0CLR = _IFS0_U1TXIF_MASK;  // Clear UART1 Tx interrupt flag
#else
        IFS1CLR = _IFS1_U1TXIF_MASK;  // Clear UART1 Tx interrupt flag
#endif
    }
    
#if __PIC32_FEATURE_SET__ == 795
    if (IFS0bits.U1RXIF)
#else
    if (IFS1bits.U1RXIF)
#endif
    {
        while (U1STAbits.URXDA)     // Loop to empty the Rx FIFO
        {
            const uint8_t tmphead = (U1Buf.rx.head + 1) & UART_RX_BUFFER_MASK;
            const uint8_t ch = U1RXREG;   // Read received byte from UART

            if (tmphead == U1Buf.rx.tail)   // Is receive buffer full?
            {
                 // Buffer is full; discard new byte
            }
            else
            {
                U1Buf.rx.head = tmphead;
                U1Buf.rx.buf[tmphead] = ch;   // Store byte in buffer
            }

#if __PIC32_FEATURE_SET__ == 795
            IFS0CLR = _IFS0_U1RXIF_MASK;  // Clear UART1 Rx interrupt flag
#else
            IFS1CLR = _IFS1_U1RXIF_MASK;  // Clear UART1 Rx interrupt flag
#endif
        }
    }
    
#if __PIC32_FEATURE_SET__ == 795
    if (IFS0bits.U1EIF)
#else
    if (IFS1bits.U1EIF)
#endif
    {
        U1STACLR = _U1STA_FERR_MASK | _U1STA_OERR_MASK | _U1STA_PERR_MASK;
        
#if __PIC32_FEATURE_SET__ == 795
        IFS0CLR = _IFS0_U1EIF_MASK;   // Clear UART1 error interrupt flag
#else
        IFS1CLR = _IFS1_U1EIF_MASK;   // Clear UART1 error interrupt flag
#endif
    }
}


uint8_t UART1RxByte(void)
{
    const uint8_t tmptail = (U1Buf.rx.tail + 1) & UART_RX_BUFFER_MASK;
    
    while (U1Buf.rx.head == U1Buf.rx.tail)  // Wait, if buffer is empty
        ;
    
    U1Buf.rx.tail = tmptail;
    
    return (U1Buf.rx.buf[tmptail]);
}


void UART1TxByte(const uint8_t data)
{
    const uint8_t tmphead = (U1Buf.tx.head + 1) & UART_TX_BUFFER_MASK;
    
    while (tmphead == U1Buf.tx.tail)   // Wait, if buffer is full
        ;

    U1Buf.tx.buf[tmphead] = data;
    U1Buf.tx.head = tmphead;

#if __PIC32_FEATURE_SET__ == 795
    IEC0SET = _IEC0_U1TXIE_MASK;       // Enable UART1 Tx interrupt
#else
    IEC1SET = _IEC1_U1TXIE_MASK;       // Enable UART1 Tx interrupt
#endif
}


bool UART1RxAvailable(void)
{
    return (U1Buf.rx.head != U1Buf.rx.tail);
}


void _mon_putc(const char ch)
{
    // See: https://microchipdeveloper.com/faq:81
    if (ch == '\n')
    {
        UART1TxByte('\r');
    }
    
    UART1TxByte(ch); // Connect stdout to UART1
}


/* printDeviceID --- print the Device ID bytes as read from DEVID */

void printDeviceID(void)
{
    printf("Device ID = %08x\n", DEVIDbits.DEVID);
    printf("Version = %02x\n", DEVIDbits.VER);
   
    switch (DEVIDbits.DEVID)
    {
    case 0x06a13053:
        puts("PIC32MX250F256L");    // The usual
        break;
    case 0x06a15053:
        puts("PIC32MX550F256L");    // The other one
        break;
    case 0x06a35053:
        puts("PIC32MX570F512L");    // More RAM
        break;
    case 0x04307053:
        puts("PIC32MX795F512L");    // Faster
        break;
    }
}


/* printCompilerInfo --- print some useful information that xc32 gives us */

void printCompilerInfo(void)
{
    printf("__PIC32_FEATURE_SET__ = %d\n", __PIC32_FEATURE_SET__);
    printf("__XC32_PART_SUPPORT_VERSION = %d\n", __XC32_PART_SUPPORT_VERSION);
}


/* printResetReason --- print the cause of the chip's reset */

void printResetReason(void)
{
    printf("RCON = 0x%08x\n", SavedRCON);
    
    if (SavedRCON & _RCON_POR_MASK)
    {
       fputs("POR ", stdout);
    }
    
    if (SavedRCON & _RCON_BOR_MASK)
    {
        fputs("BOR ", stdout);
    }
    
    if (SavedRCON & _RCON_WDTO_MASK)
    {
        fputs("WDTO ", stdout);
    }
    
    if (SavedRCON & _RCON_SWR_MASK)
    {
        fputs("SWR ", stdout);
    }
    
    if (SavedRCON & _RCON_EXTR_MASK)
    {
        fputs("EXTR ", stdout);
    }
    
    if (SavedRCON & _RCON_CMR_MASK)
    {
        fputs("CMR ", stdout);
    }
    
#ifdef _RCON_HVDR_MASK
    if (SavedRCON & _RCON_HVDR_MASK)
    {
        fputs("HVDR ", stdout);
    }
#endif
    
    fputs("\n", stdout);
}


/* softwareReset --- reset the microcontroller */

void softwareReset(void)
{
    puts("Software RESET...");
    
    // Delay about 40ms so that the message can get out
    uint32_t end = millis() + 40u;
    
    while (millis() < end)
        ;
    
    __builtin_disable_interrupts();    // Global interrupt disable
    
    SYSKEY = 0x0;        // Ensure system is locked
    SYSKEY = 0xAA996655; // Write Key1 to SYSKEY
    SYSKEY = 0x556699AA; // Write Key2 to SYSKEY

    RSWRSTSET = _RSWRST_SWRST_MASK;
    volatile int __attribute__((unused)) junk = RSWRST;
    
    for (;;)
        ;
}


/* testWatchdog --- enter an infinite loop to test the watchdog */

void testWatchdog(void)
{
    int second;
    
    printf("Watchdog test...\n");
    
    for (second = 0; ; second++) {
        printf("%d ", second);
        fflush(stdout);
        
        uint32_t end = millis() + 1000u;
        
        while (millis() < end)
            ;
   }
}


/* nudgeWatchdog --- reset the watchdog counter */

void nudgeWatchdog(void)
{
    WDTCONSET = _WDTCON_WDTCLR_MASK;
}


/* initMCU --- set up the microcontroller in general */

void initMCU(void)
{
#if __PIC32_FEATURE_SET__ == 795
    // Data Memory SRAM wait states: Default Setting = 1; set it to 0
    BMXCONbits.BMXWSDRM = 0;
    
    // Flash PM Wait States: MX Flash runs at 2 wait states @ 80 MHz
    CHECONbits.PFMWS = 2;
    
    // Prefetch-cache: Enable prefetch for cacheable PFM instructions
    CHECONbits.PREFEN = 1;
#endif
    
    /* Configure interrupts */
    INTCONSET = _INTCON_MVEC_MASK; // Multi-vector mode
    
    /* Enable watchdog timer */
    WDTCONSET = _WDTCON_ON_MASK;
    
    /* Remember why this reset occurred */
    SavedRCON = RCON;
    RCONCLR = 0xffffffff;
}


/* initGPIOs --- set up the GPIO pins */

static void initGPIOs(void)
{
    /* No analog pins in use */
#ifdef ANSELA
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    ANSELD = 0;
    ANSELE = 0;
    ANSELF = 0;
    ANSELG = 0;
#else
    AD1PCFG = 0xFFFF;
#endif
    
    /* GPIO pins used as outputs */
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


/* initUARTs --- set up UART(s) and buffers, and connect to 'stdout' */

void initUARTs(void)
{
    const int baud = 9600;
    
    U1Buf.tx.head = 0;
    U1Buf.tx.tail = 0;
    U1Buf.rx.head = 0;
    U1Buf.rx.tail = 0;

    /* Configure PPS if we have it */
#ifdef RPG0R
    RPG0Rbits.RPG0R = 3;          // U1Tx on pin 90, RPG0, P3 pin 40
    U1RXRbits.U1RXR = 12;         // U1Rx on pin 89, RPG1, P3 pin 39 (5V tolerant)
    CNPUGbits.CNPUG1 = 1;         // Enable pull-up on U1Rx in case it's floating
#else
    // No PPS:
    // U1Tx on RF8, pin 53
    // U1Rx on RF2, pin 52
    // No Change Notify on RF2, so cannot be given a pull-up
#endif
    
    U1MODEbits.UEN = 3;           // Use just Rx/Tx; no handshaking
    
    U1BRG = (FPBCLK / (baud * 16)) - 1;
    
#if __PIC32_FEATURE_SET__ == 795
    IPC6bits.U1IP = 1;            // UART1 interrupt priority 1 (lowest)
    IPC6bits.U1IS = 0;            // UART1 interrupt sub-priority 0
    
    IFS0CLR = _IFS0_U1TXIF_MASK;  // Clear UART1 Tx interrupt flag
    IFS0CLR = _IFS0_U1RXIF_MASK;  // Clear UART1 Rx interrupt flag
    IFS0CLR = _IFS0_U1EIF_MASK;   // Clear UART1 error interrupt flag
    
    IEC0SET = _IEC0_U1RXIE_MASK;  // Enable UART1 Rx interrupt
    IEC0SET = _IEC0_U1EIE_MASK;   // Enable UART1 error interrupt
#else
    IPC7bits.U1IP = 1;            // UART1 interrupt priority 1 (lowest)
    IPC7bits.U1IS = 0;            // UART1 interrupt sub-priority 0
    
    IFS1CLR = _IFS1_U1TXIF_MASK;  // Clear UART1 Tx interrupt flag
    IFS1CLR = _IFS1_U1RXIF_MASK;  // Clear UART1 Rx interrupt flag
    IFS1CLR = _IFS1_U1EIF_MASK;   // Clear UART1 error interrupt flag
    
    IEC1SET = _IEC1_U1RXIE_MASK;  // Enable UART1 Rx interrupt
    IEC1SET = _IEC1_U1EIE_MASK;   // Enable UART1 error interrupt
#endif
    
    U1MODESET = _U1MODE_ON_MASK;  // Enable UART1
    U1STASET = _U1STA_UTXEN_MASK | _U1STA_URXEN_MASK; // Enable Rx and Tx
}


int main(void)
{
    uint32_t end;
    bool buttonState = true;
    
    initMCU();
    initGPIOs();
    initUARTs();
    initMillisecondTimer();
    
    __builtin_enable_interrupts();     // Global interrupt enable
    
    printf("\nHello from the PIC%dMX%dF%dL\n", 32, 250, 256);
    
    printResetReason();
    printDeviceID();
    
    end = millis() + 500UL;
    
    while (1)
    {
        if (Tick)
        {
            if (millis() >= end)
            {
                end = millis() + 500UL;
                LATBINV = _LATB_LATB9_MASK; // Toggle LED on RB9

                printf("millis() = %ld\n", millis());
            }
            
            if (PORTAbits.RA1 != buttonState)
            {
                if (PORTAbits.RA1)
                {
                    puts("RELEASE");
                }
                else
                {
                    puts("PRESS");
                }
                
                buttonState = PORTAbits.RA1;
            }
            
            nudgeWatchdog();
            
            Tick = false;
        }
        
        if (UART1RxAvailable())
        {
            const uint8_t ch = UART1RxByte();
         
            printf("UART1: %02x\n", ch);
            switch (ch) {
            case 'i':
            case 'I':
                printDeviceID();
                break;
            case 'r':
            case 'R':
                printResetReason();
                break;
            case '~':
                softwareReset();
                break;
            case 'w':
            case 'W':
                testWatchdog();
                break;
            case 'x':
            case 'X':
                printCompilerInfo();
                break;
            }
        }
    }
    
    return (EXIT_SUCCESS);
}

