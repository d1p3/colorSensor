// Timing C/ASM Mix Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "tm4c123gh6pm.h"




#define GREEN_LED_MASK 32
#define RED_LED_MASK 32
#define BLUE_LED_MASK 16
#define PUSH_BUTTON_MASK 16

//global variables
uint16_t pwmg,pwmb,pwmr;
#define MAX_CHARS 60
#define MAX_FIELDS 2
char str[MAX_CHARS];
uint8_t pos[MAX_FIELDS];
uint8_t count = 0;
uint16_t period, delta, avg_red, avg_blue, avg_green;
bool periodicMode = 0;
bool deltaMode = 0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A, B and E peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;
    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;             // turn-on PWM0 module
    SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status

    // Enable Timer
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;

    // Turn EEPROM
    SYSCTL_RCGCEEPROM_R |= SYSCTL_RCGCEEPROM_R0;

    // Configure Red LED in port B
    GPIO_PORTB_DIR_R = RED_LED_MASK;
    GPIO_PORTB_DR2R_R = RED_LED_MASK;
    GPIO_PORTB_DEN_R =  RED_LED_MASK;
    GPIO_PORTB_ODR_R = RED_LED_MASK;
    GPIO_PORTB_AFSEL_R |= RED_LED_MASK; // select auxilary function for bit 5
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB5_M0PWM3; // enable PWM on bit 5

    // Configure Green and Blue LED in port E
    GPIO_PORTE_DIR_R = GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTE_DR2R_R = GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTE_DEN_R = GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTE_ODR_R = GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTE_AFSEL_R |= GREEN_LED_MASK | BLUE_LED_MASK; // select auxilary function for bits 4 and 5
    GPIO_PORTE_PCTL_R = GPIO_PCTL_PE4_M0PWM4 | GPIO_PCTL_PE5_M0PWM5; // enable PWM on bits 4 and 5


    // Configure LED and pushbutton pins
    GPIO_PORTF_DEN_R = PUSH_BUTTON_MASK;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = PUSH_BUTTON_MASK; // enable internal pull-up for push button

    // Configure PWM module0 to drive RGB backlight
    // RED   on M0PWM3 (PB5), M0PWM1b
    // BLUE  on M0PWM4 (PE4), M0PWM2a
    // GREEN on M0PWM5 (PE5), M0PWM2b
    __asm(" NOP");                                   // wait 3 clocks
    __asm(" NOP");
    __asm(" NOP");
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM0_1_CTL_R = 0;                                // turn-off PWM0 generator 1
    PWM0_2_CTL_R = 0;                                // turn-off PWM0 generator 2
    PWM0_1_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;
                                                     // output 3 on PWM0, gen 1b, cmpb
    PWM0_2_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE;
                                                     // output 4 on PWM0, gen 2a, cmpa
    PWM0_2_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;
                                                     // output 5 on PWM0, gen 2b, cmpb
    PWM0_1_LOAD_R = 1024;                            // set period to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM0_2_LOAD_R = 1024;
    //PWM0_INVERT_R = PWM_INVERT_PWM3INV | PWM_INVERT_PWM4INV | PWM_INVERT_PWM5INV;
                                                     // invert outputs for duty cycle increases with increasing compare values
    PWM0_1_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
    PWM0_2_CMPB_R = 0;                               // green off
    PWM0_2_CMPA_R = 0;                               // blue off

    PWM0_1_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 1
    PWM0_2_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 2
    PWM0_ENABLE_R = PWM_ENABLE_PWM3EN | PWM_ENABLE_PWM4EN | PWM_ENABLE_PWM5EN;
                                                     // enable outputs

    // Configure AN0 as an analog input
    GPIO_PORTE_AFSEL_R |= 0x08;                      // select alternative functions for AN0 (PE3)
    GPIO_PORTE_DEN_R &= ~0x08;                       // turn off digital operation on pin PE3
    GPIO_PORTE_AMSEL_R |= 0x08;                      // turn on analog operation on pin PE3

    // Configure ADC
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 0;                               // set first sample to AN0
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

    // Configure UART0 pins
    GPIO_PORTA_DEN_R |= 3;                           // enable digital on UART0 pins: default, added for clarity
    GPIO_PORTA_DIR_R |= 2;                           // enable output on UART0 TX pin: default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // use peripheral to drive PA0, PA1: default, added for clarity
    GPIO_PORTA_PCTL_R &= 0xFFFFFF00;                 // set fields for PA0 and PA1 to zero
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                     // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
}

void Timer1Isr() //Interrupt function
{
    displayRgbTriplet();
    TIMER1_ICR_R = TIMER_ICR_TATOCINT; // clear interrupt flag
}
void displayRgbTriplet()
{
    uint16_t red, blue, green;
    uint16_t k = 1;
    char str[30];

    setRgbColor(pwmr, 0, 0);
    waitMicrosecond(10000);
    red = readAdc0Ss3();
    waitMicrosecond(10000);
    red = readAdc0Ss3();

    setRgbColor(0, pwmg, 0);
    waitMicrosecond(10000);
    green = readAdc0Ss3();
    waitMicrosecond(10000);
    green = readAdc0Ss3();

    setRgbColor(0, 0, pwmb);
    waitMicrosecond(10000);
    blue = readAdc0Ss3();
    waitMicrosecond(10000);
    blue = readAdc0Ss3();

    if (deltaMode){
        avg_red = 0.9 * avg_red + 0.1 * red;
        avg_green = 0.9 * avg_green + 0.1 * green;
        avg_blue = 0.9 * avg_blue + 0.1 * blue;

        uint16_t delta_red = abs(red - avg_red);
        uint16_t delta_green = abs(green - avg_green);
        uint16_t delta_blue = abs(blue - avg_blue);
        if (delta_red > delta || delta_green > delta || delta_blue > delta)
        {
            ltoa(red*k,str);
            putsUart0(str);
            putsUart0(", ");
            ltoa(green*k,str);
            putsUart0(str);
            putsUart0(", ");
            ltoa(blue*k,str);
            putsUart0(str);
            putsUart0("\r\n");
        }
    }
    else
    {
        ltoa(red*k,str);
        putsUart0(str);
        putsUart0(", ");
        ltoa(green*k,str);
        putsUart0(str);
        putsUart0(", ");
        ltoa(blue*k,str);
        putsUart0(str);
        putsUart0("\r\n");
    }
}


void waitPbPress()
{
    while(GPIO_PORTF_DATA_R & PUSH_BUTTON_MASK);
}

void setTimer()
{
    uint32_t freq = period * 40000 * 100; //(time/10)*40000*1000
    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;        // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;  // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
    TIMER1_TAILR_R = freq;                  // set load value to 40e6 for 1 Hz interrupt rate

    TIMER1_IMR_R = TIMER_IMR_TATOIM;        // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A - 16); // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;        // turn-on timer
}

void writeEEPROM(uint16_t data)
{
    EEPROM_EERDWRINC_R = data; //write data to EEPROM
    __asm("             NOP"); //wait for 6 cycles
    __asm("             NOP");
    __asm("             NOP");
    __asm("             NOP");
    __asm("             NOP");
    __asm("             NOP");
    while (EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING)
        ;                                                                                 //wait until EEPROM is not working
    if (EEPROM_EESUPP_R & EEPROM_EESUPP_PRETRY || EEPROM_EESUPP_R & EEPROM_EESUPP_ERETRY) //check the retry bits
    {
        putsUart0("Error Occured");
    }
}

uint16_t readEEPROM()
{
    return EEPROM_EERDWRINC_R; //get data from EEPROM
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}
int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3; // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY)
        ;                  // wait until SS3 is not busy
    return ADC0_SSFIFO3_R; // get single result from the FIFO
}
void setRgbColor(uint16_t red, uint16_t green, uint16_t blue)
{
    PWM0_1_CMPB_R = red;
    PWM0_2_CMPA_R = blue;
    PWM0_2_CMPB_R = green;
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

void getsUart0(char* str,uint8_t maxChars){
    uint8_t count;
    char c;
    count = 0;
    while (1){
        c = getcUart0();
        if (c == 8 && count != 0)
            count--;
        if (c == 13){
            str[count] = '\0';
            return;
        }
        if (c >= 32){
            if (c>='A' && c<='Z')
                c=c+('a'-'A');
            str[count] = c;
            count++;
        }
        if (count == maxChars){
            str[count] = '\0';
            return;
        }
    }
    return;
}
//Is Command function
// Tokenize Input string by replacing delimeter with null character and add position, type and field count
void tokenizeString()
{
    count = 0;
    int i;
    int N = strlen(str);//Length of the input
    char p ='d';//previous
    char c;//current
    for (i = 0; i < N; i++){
        char c = str[i];
        if (c>='a' && c<='z') //if c is alphabet (a-z)
            c='a';
        else if (c>='0' && c<='9') //if c is number (0-9)
            c='n';
        else{
            c='d';
            str[i]=0;
        }
        if (p != c){
            if (p=='d' && c=='a'){      //transition from delimeter to alpha
                pos[count]=i;
                count++;
            }
            else if (p=='d' && c=='n'){         //transition from delimeter to number
                pos[count]=i;
                count++;
            }
        }
        p = c;
    }
}

// Check valid command
bool isCommand(char *cmd, uint8_t min){
    if (count > min){
        int i;
        for(i = 0; i < strlen(cmd); i++){
            if (cmd[i] != str[pos[0]+i])
                return false;
        }
        return true;
    }
    else
        return false;
}
// Return the argument
char *getString(uint8_t argN){
    return &str[pos[argN+1]];
}

// Return the numerical value of argument
uint16_t getValue(uint8_t argN){
    return atoi(getString(argN));
}


bool ExecuteCommand(){
    bool ok=true;
        if (isCommand("rgb",3)){
            int16_t r, g, b;
            r = getValue(0);
            g = getValue(1);
            b = getValue(2);
            setRgbColor(r, g, b);
        }

        else if (isCommand("ramp",1)){
        int16_t i;
        //putsUart0(getString(0));
        if (strcmp(getString(0),"red") == 0){
            for (i = 0; i < 1024; i++){
                setRgbColor(i,0,0);
                waitMicrosecond(1000);
            }
        }
        else if (strcmp(getString(0), "green") == 0){
            for (i = 0; i < 1024; i++){
                setRgbColor(0, i, 0);
                waitMicrosecond(1000);
            }
        }
        else if (strcmp(getString(0), "blue") == 0){
            for (i = 0; i < 1024; i++){
                setRgbColor(0, 0, i);
                waitMicrosecond(1000);
            }
        }
        }

        else if (isCommand("light",0)){
            uint16_t raw=0;
            char str1[20];
            raw = readAdc0Ss3();
            //waitMicrosecond(10000);
            //raw = readAdc0Ss3();
            ltoa(raw,str1);
            putsUart0(str1);
            putsUart0("\r\n");
        }

        else if(isCommand("trigger",0)){
            if (periodicMode){
                TIMER1_IMR_R = 0; // turn-off interrupts
                periodicMode = 0;
                putsUart0("Periodic mode disabled.\r\n");
            }
            uint16_t raw;
            uint16_t i;
            char str1[20];

            setRgbColor(pwmr,0,0);
            waitMicrosecond(10000);
            ltoa(readAdc0Ss3(),str1);
            waitMicrosecond(10000);
            ltoa(readAdc0Ss3(),str1);
            putsUart0(str1);
            putsUart0(",");

            setRgbColor(0,pwmg,0);
            waitMicrosecond(10000);
            ltoa(readAdc0Ss3(),str1);
            waitMicrosecond(10000);
            ltoa(readAdc0Ss3(),str1);
            putsUart0(str1);
            putsUart0(",");

            setRgbColor(0,0,pwmb);
            waitMicrosecond(10000);
            ltoa(readAdc0Ss3(),str1);
            waitMicrosecond(10000);
            ltoa(readAdc0Ss3(),str1);
            putsUart0(str1);
            putsUart0("\r\n");

            setRgbColor(pwmr,pwmb,pwmg);
        }

        //Calibrating the sensor
        else if(isCommand("calibrate",0)){
            if (periodicMode){
                TIMER1_IMR_R = 0; // turn-off interrupts
                periodicMode = 0;
                putsUart0("Periodic mode disabled.\r\n");
            }
            uint16_t t=511;
            uint16_t i, raw;
            char str1[20];
            for (i = 0; i < 1024; i++){
                setRgbColor(i,0,0);
                waitMicrosecond(1000);
                raw = readAdc0Ss3();
                waitMicrosecond(1000);
                raw = readAdc0Ss3();
                if( i > 50 && raw > t){
                    pwmr = i;
                    break;
                }
            }

            setRgbColor(0,0,0);
            waitMicrosecond(10000);
            readAdc0Ss3();
            setRgbColor(0,0,0);
            waitMicrosecond(10000);
            readAdc0Ss3();

            for (i = 0; i < 1024; i++){
                setRgbColor(0,i,0);
                waitMicrosecond(1000);
                raw = readAdc0Ss3();
                waitMicrosecond(1000);
                raw = readAdc0Ss3();
                if( i > 50 && raw > t){
                    pwmg = i;
                    break;
                }
            }

            setRgbColor(0,0,0);
            waitMicrosecond(10000);
            readAdc0Ss3();
            setRgbColor(0,0,0);
            waitMicrosecond(10000);
            readAdc0Ss3();

            for (i = 0; i < 1024; i++){
                setRgbColor(0,0,i);
                waitMicrosecond(1000);
                raw = readAdc0Ss3();
                waitMicrosecond(1000);
                raw = readAdc0Ss3();
                if( i > 50 && raw > t){
                    pwmb = i;
                    break;
                }
            }

            ltoa(pwmr,str1);
            putsUart0(str1);
            putsUart0(", ");
            ltoa(pwmg,str1);
            putsUart0(str1);
            putsUart0(", ");
            ltoa(pwmb,str1);
            putsUart0(str1);
            }

        else if(isCommand("button",0)){
            if (periodicMode){
                TIMER1_IMR_R = 0; // turn-off interrupts
                periodicMode = 0;
                putsUart0("Periodic mode disabled.\r\n");
            }
            waitPbPress();
            uint16_t i;
            char str1[20];
            setRgbColor(pwmr,0,0);
            waitMicrosecond(10000);
            ltoa(readAdc0Ss3(),str);
            waitMicrosecond(10000);
            ltoa(readAdc0Ss3(),str1);

            putsUart0(str1);
            putsUart0(",");


            setRgbColor(0,pwmg,0);
            waitMicrosecond(10000);
            ltoa(readAdc0Ss3(),str1);
            waitMicrosecond(10000);
            ltoa(readAdc0Ss3(),str1);

            putsUart0(str1);
            putsUart0(",");


            setRgbColor(0,0,pwmb);
            waitMicrosecond(10000);
            ltoa(readAdc0Ss3(),str1);
            waitMicrosecond(10000);
            ltoa(readAdc0Ss3(),str1);

            putsUart0(str1);
            putsUart0("\r\n");
        }

        else if (isCommand("test",0)){
            if (periodicMode){
                TIMER1_IMR_R = 0; // turn-off interrupts
                periodicMode = 0;
                putsUart0("Periodic mode disabled.\r\n");
            }
            int16_t i;
            int16_t r,g,b;
            char str1[20];
            for (i = 0; i < 1024; i++){
                setRgbColor(i,0,0);
                ltoa(i,str1);
                putsUart0(str1);
                putsUart0(",0,0,");
                ltoa(readAdc0Ss3(),str1);
                putsUart0(str1);
                putsUart0("\r\n");
                waitMicrosecond(10000);
            }
            for (i = 0; i < 1024; i++){
                setRgbColor(0, i, 0);
                ltoa(i,str1);
                putsUart0("0,");
                putsUart0(str1);
                putsUart0(",0,");
                ltoa(readAdc0Ss3(),str1);
                putsUart0(str1);
                putsUart0("\r\n");
                waitMicrosecond(10000);
            }
            for (i = 0; i < 1024; i++){
                setRgbColor(0, 0, i);
                ltoa(i,str1);
                putsUart0("0,0,");
                putsUart0(str1);
                putsUart0(",");
                ltoa(readAdc0Ss3(),str1);
                putsUart0(str1);
                putsUart0("\r\n");
                waitMicrosecond(10000);
            }
        }

        else if (isCommand("periodic",0)){
            period = getString(0);
            if (period == 0){
                TIMER1_IMR_R = 0; // turn-off interrupts
                periodicMode = 0;
                putsUart0("Periodic mode disabled.\r\n");
            } else {
                periodicMode = 1;
                setTimer();
                putsUart0("Periodic mode enabled\r\n");// every %.1f second.\r\n", (float)period / 10);
            }
        }

        else if (isCommand("delta",0)){
            delta = getString(0);
            if (delta == 0){
                avg_red = 0;
                avg_green = 0;
                avg_blue = 0;
                deltaMode = 0;
                putsUart0("Delta mode disabled.\r\n");
            } else {
                deltaMode = 1;
                putsUart0("Delta Mode enabled. r, g, b value will be displayed if there is a big change.\r\n");
            }
        }

        else if (isCommand("try",0)){
            setTimer();
        }
        else if (isCommand("help", 0)){
            putsUart0("rgb x x x    -- set the value of LEDs(eg. rgb 1,2,3)\r\n");
            putsUart0("ramp x       -- ramp LEDs (eg. 'ramp red' ramps the red color)\r\n");
            putsUart0("light        -- display sensor reading\r\n");
            putsUart0("trigger      -- check color value immediately\r\n");
            putsUart0("calibrate    -- calibrate the sensor according to the threshold value\r\n");
            putsUart0("button       -- check color value on button press\r\n");
            putsUart0("color x      -- save the latest sampled rgb\r\n");
            putsUart0("memory       -- load currently stored colors\r\n");
            putsUart0("erase x      -- erase the stored color in x\r\n");
            putsUart0("periodic x   -- set the Periodic Mode\r\n");
            putsUart0("delta x      -- display rgb triplet only if the average changes by x\r\n");
            putsUart0("match x      -- display stored color if it is within Euclidean distance\r\n");
            putsUart0("save         -- save c configuration and color references in EEPROM\r\n");
            putsUart0("load         -- load configuration and color references from EEPROM\r\n");
            putsUart0("test         -- ramps LEDs individually and gets RGB triplet\r\n");
            putsUart0("threshold x  -- set the threshold value for calibration\r\n");
            putsUart0("help         -- display info for valid commands\r\n");
        }
        else ok=false;
    return ok;
}
int main(void)
{
    // Initialize hardware
    initHw();
    putsUart0("(Colorimeter Ready!");
    putsUart0("Enter Command:");
    putsUart0("\r\n");
    while(1){
        getsUart0(str,MAX_CHARS);
        tokenizeString();
        if(!ExecuteCommand()){
            putsUart0("\r\n");
            putsUart0("Invalid Command!");
        }
        putsUart0("\r\n");
    }
}
