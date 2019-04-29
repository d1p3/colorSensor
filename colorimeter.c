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
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"

#define GREEN_LED_MASK 32
#define RED_LED_MASK 32
#define BLUE_LED_MASK 16

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
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE;
    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;             // turn-on PWM0 module
    SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status

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
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY){           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;
    }// get single result from the FIFO
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

void getsUart0(char* str, uint8_t maxChars){
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

int main(void)
{
    // Initialize hardware
    initHw();

    int16_t r, g, b;
    char str[30];
    char delim[] = " ";
    char *cmd;
    while(true)
    {
        getsUart0(str, 28);
        putsUart0("\r\n");
        putsUart0(str);
        putsUart0("\r\n\r\n");
        cmd = strtok(str, delim);

        if (strcmp(cmd, "rgb") == 0){
            r = atoi(strtok(NULL, delim));
            g = atoi(strtok(NULL, delim));
            b = atoi(strtok(NULL, delim));
            setRgbColor(r, g, b);
            waitMicrosecond(1000);
        }
        if (strcmp(cmd, "ramp") == 0){
            cmd = strtok(NULL, delim);
            int16_t i;
            if (strcmp(cmd, "red") == 0){
                for (i = 0; i < 1024; i++){
                    setRgbColor(i,0,0);
                    waitMicrosecond(1000);
                }
            }
            else if (strcmp(cmd, "green") == 0){
                for (i = 0; i < 1024; i++){
                    setRgbColor(0, i, 0);
                    waitMicrosecond(1000);
                }
            }
            else if (strcmp(cmd, "blue") == 0){
                for (i = 0; i < 1024; i++){
                    setRgbColor(0, 0, i);
                    waitMicrosecond(1000);
                }
            }
        }
        if (strcmp(cmd, "light") == 0){
            uint16_t raw=0;
            char str[20];
            raw = readAdc0Ss3();
            waitMicrosecond(10000);
            raw = readAdc0Ss3();
            ltoa(raw,str,10);
            //sprintf(str, "Sensor Reading: %3.1f\r\n", raw);
            putsUart0(str);
            putsUart0("\r\n");
        }
        if(strcmp(cmd,"calibrate")==0){
            uint16_t t=255;
            uint16_t pwmg,pwmb,pwmr;
           for (i = 0; i < 1024; i++){
               if(setRgbColor(i,0,0)==readAdc0Ss3())
                   pwmr=readAdc0Ss3();
               waitMicrosecond(1000);
           }
           for (i = 0; i < 1024; i++){
              setRgbColor(i,0,0);
              if(setRgbColor(0,i,0)==readAdc0Ss3())
                  pwmb=readAdc0Ss3();
              waitMicrosecond(1000);
           }
           for (i = 0; i < 1024; i++){
              if(setRgbColor(0,0,i)==readAdc0Ss3())
                  pwmg=readAdc0Ss3();
              waitMicrosecond(1000);
           }
           setRgbColor(pwmr,pwmb,pwmg);
        }
        if(strcmp(cmd,"trigger")==0){
            for (i=0;i<pwmr;i++){
                setRgbColor(i,0,0);
            }
        }
        waitMicrosecond(1000);
    }
}
