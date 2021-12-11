  /* TI-RTOS Header files */
  #include <xdc/std.h>
  #include <ti/sysbios/BIOS.h>
  #include <ti/sysbios/knl/Task.h>
  #include <ti/sysbios/knl/Clock.h>
  #include <ti/drivers/I2C.h>
  #include <ti/drivers/SPI.h>
  #include <ti/drivers/UART.h>
  #include <ti/drivers/Watchdog.h>
  #include <stdbool.h>
  #include <stdint.h>
  #include <stdlib.h>
  #include <stdio.h>
  #include <stdarg.h>
  #include <stdbool.h>
  #include "sensorlib/i2cm_drv.h"
  #include "sensorlib/hw_mpu6050.h"
  #include "sensorlib/mpu6050.h"
  #include "inc/hw_ints.h"
  #include "inc/hw_memmap.h"
  #include "inc/hw_sysctl.h"
  #include "inc/hw_types.h"
  #include "inc/hw_i2c.h"
  #include "inc/hw_types.h"
  #include "inc/hw_gpio.h"
  #include "driverlib/gpio.h"
  #include "driverlib/pin_map.h"
  #include "driverlib/rom.h"
  #include "driverlib/rom_map.h"
  #include "driverlib/debug.h"
  #include "driverlib/interrupt.h"
  #include "driverlib/i2c.h"
  #include "driverlib/sysctl.h"
  #include "driverlib/uart.h"
  #include "utils/uartstdio.h"
  #include "driverlib/uart.h"
  #include "math.h"
  #include <string.h>
  #include "icm20948_def.h"

  #include <ti/drivers/GPIO.h>

  /* Driver configuration */
  #include "ti_drivers_config.h"

  void myDelay(int count);

  /* Could be anything, like computing primes */
  #define FakeBlockingSlowWork()   myDelay(12000000)
  #define FakeBlockingFastWork()   myDelay(2000000)

  Task_Struct workTask;
  Task_Struct urgentWorkTask;
  /* Make sure we have nice 8-byte alignment on the stack to avoid wasting memory */
  #pragma DATA_ALIGN(workTaskStack, 8)
  #define STACKSIZE 1024
  static uint8_t workTaskStack[STACKSIZE];
  static uint8_t urgentWorkTaskStack[STACKSIZE];

  void doUrgentWork(void)
  {
      GPIO_write(CONFIG_GPIO_LED_1, CONFIG_LED_OFF);
      FakeBlockingFastWork(); /* Pretend to do something useful but time-consuming */
      GPIO_write(CONFIG_GPIO_LED_1, CONFIG_LED_ON);
  }

  void doWork(void)
  {
      GPIO_write(CONFIG_GPIO_LED_0, CONFIG_LED_OFF);
      FakeBlockingSlowWork(); /* Pretend to do something useful but time-consuming */
      GPIO_write(CONFIG_GPIO_LED_0, CONFIG_LED_ON);
  }

  void HeartBeat(UArg arg0, UArg arg1)
  {
      // Initialize the Sensor Controller
      scifOsalInit();
      scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
      scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
      scifInit(&scifDriverSetup);

      // Set the Sensor Controller task tick interval to 1 second
      uint32_t rtc_Hz = 1;  // 1Hz RTC
      scifStartRtcTicksNow(0x00010000 / rtc_Hz);

      // Configure Sensor Controller tasks
      scifTaskData.adcLevelTrigger.cfg.threshold = 600;

      // Start Sensor Controller task
      scifStartTasksNbl(BV(SCIF_ADC_LEVEL_TRIGGER_TASK_ID));

      while (1) {}
  }

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

  void UARTDisplay(void)
  {
      UARTprintf("Here is the UART fucntion");
  }

  void SwitchInput(void)
  {
      UARTprintf("Type something : ");
      inputchar = UARTgetc();
      UARTprintf( inputchar );
  }

  void initI2C0(void)
  {
      // Turn on I2C0
      SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
      SysCtlDelay(3);
      // Reset I2C0
      SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
      SysCtlDelay(3);
      // Enable GPIOB
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
      SysCtlDelay(3);

      // Configure GPIO SCL/SDA pins on PB2/PB3
      GPIOPinConfigure(GPIO_PB2_I2C0SCL);
      GPIOPinConfigure(GPIO_PB3_I2C0SDA);

      // Set pins to I2C function
      GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
      GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

      // Enable and master I2C
      I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

      // Clear I2C FIFOs
      HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
  }



  void I2C0Read(uint8_t slave_addr, uint8_t reg, uint8_t *data)
  {
      I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
      I2CMasterDataPut(I2C0_BASE, reg);
      I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
      while(I2CMasterBusy(I2C0_BASE));
      I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);
      I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
      while(I2CMasterBusy(I2C0_BASE));
      *data = I2CMasterDataGet(I2C0_BASE);
  }

  // This function has not been tested - for using 16bit read you can
  // also use the I2C0Read twice if this does not work
  void I2C0Read16(uint8_t slave_addr, uint8_t reg, uint16_t *data)
  {
      uint8_t HByte , LByte=0;
      I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
      I2CMasterDataPut(I2C0_BASE, reg);
      I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
      while(I2CMasterBusy(I2C0_BASE));
      I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);
      I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
      while(I2CMasterBusy(I2C0_BASE));
      HByte = I2CMasterDataGet(I2C0_BASE);
      I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
      while(I2CMasterBusy(I2C0_BASE));
      LByte = I2CMasterDataGet(I2C0_BASE);
      *data = (LByte <<8 | HByte);
      I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
      while(I2CMasterBusy(I2C0_BASE));
  }


  void I2C0Write(uint8_t slave_addr, uint8_t reg, uint8_t data)
  {
      I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
      I2CMasterDataPut(I2C0_BASE, reg);
      I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
      while(I2CMasterBusy(I2C0_BASE));
      I2CMasterDataPut(I2C0_BASE, data);
      I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
      while(I2CMasterBusy(I2C0_BASE));
  }

  /*reads the slave device*/
  void ICM_get_whom_am_I()
  {
      uint8_t WAI=0;
      uint8_t YES;
      I2C0Write(ICM20948_ADDRESS, ICM20948_REG_PWR_MGMT_1, ICM20948_REG_LP_CONFIG);
      SysCtlDelay(3);
      I2C0Write(ICM20948_ADDRESS, ICM20948_REG_BANK_SEL, ICM20948_BANK_0);
      SysCtlDelay(3);
      I2C0Read(ICM20948_ADDRESS, ICM20948_REG_WHO_AM_I, &WAI);
      I2C0Read(ICM20948_ADDRESS, ICM20948_DEVICE_ID, &YES);
      if (WAI != YES)
      UARTprintf("Device Not Found\n");
      else
      UARTprintf("Device Found\n");
  }

  /*Initializes the ICM20948 device*/
  void Init_ICM()
  {
      UARTprintf("I2C Initialized\n");
      SysCtlDelay(3);
      ICM_get_whom_am_I();
  }

  void I2Cread(void)
  {
      uint16_t* GYRO_X;
      uint16_t* GYRO_Y;
      uint16_t* GYRO_Z;
      uint16_t* ACCEL_X;
      uint16_t* ACCEL_Y;
      uint16_t* ACCEL_Z;



      //Init I2C
      initI2C0();
      Init_ICM();

      //Low power mode

      I2C0Write(ICM20948_ADDRESS, )

        // Read GYRO X value
        I2C0Read16(ICM20948_ADDRESS, ICM20948_REG_GYRO_XOUT_H_SH, GYRO_X);

        // Read GYRO Y value
        I2C0Read16(ICM20948_ADDRESS, ICM20948_REG_GYRO_YOUT_H_SH, GYRO_Y);

        // Read GYRO Z value
        I2C0Read16(ICM20948_ADDRESS, ICM20948_REG_GYRO_ZOUT_H_SH, GYRO_Z);

        // Read Accel X value
        I2C0Read16(ICM20948_ADDRESS, ICM20948_REG_ACCEL_XOUT_H_SH, ACCEL_X);

        // Read Accel Y value
        I2C0Read16(ICM20948_ADDRESS, ICM20948_REG_ACCEL_YOUT_H_SH, ACCEL_Y);

        // Read Accel Z value
        I2C0Read16(ICM20948_ADDRESS, ICM20948_REG_ACCEL_ZOUT_H_SH, ACCEL_Z);


        UARTprintf("RAW DATA DISPLAY HERE !!!!!-------------------------------------------------------------------");
        UARTprintf("GYRO_X =  %3d\n", GYRO_X );
        UARTprintf("GYRO_Y =  %3d\n", GYRO_Y );
        UARTprintf("GYRO_Z =  %3d\n", GYRO_Z );
        UARTprintf("ACCEL_X =  %3d\n", ACCEL_X );
        UARTprintf("ACCEL_Y =  %3d\n", ACCEL_X );
        UARTprintf("ACCEL_Z =  %3d\n", ACCEL_X );
        SysCtlDelay (5000);
  }


 void ADCFunc(void)
  {
      while (1) {

          /* Do work */
          Timer1IntHandlerC();

          /* Wait a while, because doWork should be a periodic thing, not continuous.*/
          myDelay(24000000);
          Task_sleep(500 * (1000 / Clock_tickPeriod));
      }
  }

 void UartFunc()
 {
     while (1) {

         /* Do work */
         UARTDisplay();

         /* Wait a while, because doWork should be a periodic thing, not continuous.*/
         myDelay(24000000);
         Task_sleep(500 * (1000 / Clock_tickPeriod));
     }

 }

 void SwitchFunc(UArg arg0, UArg arg1)
 {
     while (1) {

         /* Do work */
         SwitchInput();

         /* Wait a while, because doWork should be a periodic thing, not continuous.*/
         myDelay(24000000);
         Task_sleep(500 * (1000 / Clock_tickPeriod));
     }
 }

 void HeartBeatFunc(UArg arg0, UArg arg1)
 {
     while (1) {

         /* Do work */
         HeartBeat();

         /* Wait a while, because doWork should be a periodic thing, not continuous.*/
        myDelay(24000000);
         Task_sleep(500 * (1000 / Clock_tickPeriod));
     }
 }

  void workTaskFunc(UArg arg0, UArg arg1)
  {
      while (1) {

          /* Do work */
          doWork();

          /* Wait a while, because doWork should be a periodic thing, not continuous.*/
          //myDelay(24000000);
          Task_sleep(500 * (1000 / Clock_tickPeriod));
      }
  }

  Void urgentWorkTaskFunc(UArg arg0, UArg arg1)
  {
      while (1) {

          /* Do work */
          doUrgentWork();

          /* Wait a while, because doWork should be a periodic thing, not continuous.*/
          //myDelay(24000000);
          Task_sleep(50 * (1000 / Clock_tickPeriod));
      }
  }

  /*
   *  ======== main ========
   *
   */
  int main(void)
  {
      Board_init();
      GPIO_init();
      I2C_init();
      UART_init();

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

      /* Set up the led task */
      Task_Params workTaskParams;
      Task_Params_init(&workTaskParams);
      workTaskParams.stackSize = STACKSIZE;

      workTaskParams.priority = 5;
      workTaskParams.stack = &workTaskStack;

      Task_construct(&workTask, I2CFunc, &workTaskParams, NULL);

      workTaskParams.priority = 4;
      workTaskParams.stack = &workTaskStack;

      Task_construct(&workTask, ADCFunc, &workTaskParams, NULL);

      workTaskParams.priority = 3;
      workTaskParams.stack = &workTaskStack;

      Task_construct(&workTask, SwitchFunc, &workTaskParams, NULL);

      workTaskParams.priority = 2;
      workTaskParams.stack = &workTaskStack;

      Task_construct(&workTask, HeartBeatFunc, &workTaskParams, NULL);

      workTaskParams.priority = 1;
      workTaskParams.stack = &urgentWorkTaskStack;

      Task_construct(&urgentWorkTask, UartFunc, &workTaskParams, NULL);

      /* Start kernel. */
      BIOS_start();

      return (0);
  }

  /*
   *  ======== myDelay ========
   *  Assembly function to delay. Decrements the count until it is zero
   *  The exact duration depends on the processor speed.
   */
  __asm("    .sect \".text:myDelay\"\n"
        "    .clink\n"
        "    .thumbfunc myDelay\n"
        "    .thumb\n"
        "    .global myDelay\n"
        "myDelay:\n"
        "    subs r0, #1\n"
        "    bne.n myDelay\n"
        "    bx lr\n");
