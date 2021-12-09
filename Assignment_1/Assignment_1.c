#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/uart.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/debug.h"
#include "driverlib/adc.h"
#include "utils/uartstdio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include <string.h>

#define PWM_FREQUENCY 55

unsigned char inputchar;
uint32_t ui32Period;

// Globals
uint32_t ui32Period;
char     buffer[4];

uint32_t ui32ADC0Value[4];
volatile uint32_t ui32TempAvg;
volatile uint32_t ui32TempValueC;
volatile uint32_t ui32TempValueF;
volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint8_t ui8Adjust;


#ifdef DEBUG
void__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

// Timer 1 ISR show F
void Timer1IntHandlerF(void)
{
     // Clear the timer interrupt
     TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

     ADCIntClear(ADC0_BASE, 2);
     ADCProcessorTrigger(ADC0_BASE, 2);

     ADCSequenceDataGet(ADC0_BASE, 2, ui32ADC0Value);

     ui32TempAvg = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2)/4;
     ui32TempValueF = ((ui32TempValueC * 9) + 160) / 5;
     UARTprintf("F %3d\t",ui32TempValueF );
     UARTprintf("\n");
}

// Timer 1 ISR
void Timer1IntHandlerC(void)
{
     // Clear the timer interrupt
     TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

     ADCIntClear(ADC0_BASE, 2);
     ADCProcessorTrigger(ADC0_BASE, 2);

     ADCSequenceDataGet(ADC0_BASE, 2, ui32ADC0Value);

     ui32TempAvg = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2)/4;
     ui32TempValueC = (1475 - ((2475 * ui32TempAvg)) / 4096)/10;
     UARTprintf("C %3d\t",ui32TempValueC );
     UARTprintf("\n");
}



int main(){
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Configure peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // for UART Port A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // For LED Port F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    // Enabling Timer 1

    // PWM input for LED
    ui8Adjust = 83;

    MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
    MAP_GPIOPinConfigure(GPIO_PD0_M1PWM0); // PD0

    ui32PWMClock = SysCtlClockGet() / 64;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);

    MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
    MAP_PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    MAP_PWMGenEnable(PWM1_BASE, PWM_GEN_0);




    // Configure ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    ADCHardwareOversampleConfigure(ADC0_BASE, 32);

    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0); // Changed to sequencer #2

    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 2, ADC_CTL_TS);

    ADCSequenceStepConfigure(ADC0_BASE, 2, 3, ADC_CTL_TS|ADC_CTL_IE|ADC_CTL_END);

    ADCSequenceEnable(ADC0_BASE, 2);


    //Configure ADC port F for LED
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

  ////////////////////
      // Configure Timer 1 module
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    ui32Period = SysCtlClockGet()/2;   // Period of 0.5s 2Hz
    TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period -1);
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

      // Configure pins for UART
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);

// here is the loop that the program run though in order to understand the command
      while (1){
        UARTprintf("Type something : ");
        inputchar = UARTgetc();

      if (inputchar == 'R'){
        // Red LED turn on
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 2);
        UARTprintf("Red LED turns on");
      }
      else if (inputchar == 'r'){
        // Red LED turn off
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
        UARTprintf("Red LED turns off");
      }
      else if (inputchar == 'G'){
        //Green LED turn on
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 8);
        UARTprintf("Green LED turns on");
      }
      else if (inputchar == 'g'){
        // Green LED turn off
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
        UARTprintf("Green LED turns off");
      }
      else if (inputchar == 'B'){
        // Blue LED turn on
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 4);
        UARTprintf("Blue LED turns on");
      }
      else if (inputchar == 'b'){
        //Blue LED turn off
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
        UARTprintf("Blue LED turns off");
      }
      else if (inputchar == 'T'){
        //show temp in F
        Timer1IntHandlerF();
        UARTprintf("show temp in F");
      }
      else if (inputchar == 't'){
        // show temp in C
        Timer1IntHandlerC();
        UARTprintf("show temp in C");
      }
      else if (inputchar == 'p'){
        //Brighten the LED
        ui8Adjust++;
            if (ui8Adjust > 111)
            {
                ui8Adjust = 111;
            }
            MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
            UARTprintf("Brighten the LED");
      }
      else if (inputchar == 'p'){
        //fade the LED
        ui8Adjust--;
            if (ui8Adjust < 56)
            {
                ui8Adjust = 56;
            }
            MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
            UARTprintf("Fade the LED");
      }
      else{
        UARTprintf("Wrong input please enter an other input");
      }

      UARTprintf("\n");

  }


  }
