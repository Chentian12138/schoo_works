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

#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

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

// Complementary filter
void ComplementaryFilter(_iq16 accData[3], _iq16 gyrData[3], float *pitch, float *roll)
{
    float pitchAcc, rollAcc;

    // Integrate the gyroscope data -> int(angularSpeed) = angle
    // Angle around the X-axis
    *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the Y-axis
    *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;

    // Compensate for drift with accelerometer data
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
        // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
        *pitch = *pitch * 0.98 + pitchAcc * 0.02;

        // Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
        *roll = *roll * 0.98 + rollAcc * 0.02;
    }
}


int main(void)
{
    // Set clock to 40 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    // Enable UART peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure UART GPIO
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);

    // set up all the variable for Gyroscope and accelerometer
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

    while(1)
    {
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


      float fAccel[3], fGyro[3];
      float pitch, roll;
      _iq16 QAccel[3];
      _iq16 QGyro[3];

      fGyro[0] = GYRO_X;
      fGyro[1] = GYRO_Y;
      fGyro[2] = GYRO_Z;
      fAccel[0] = ACCEL_X;
      fAccel[1] = ACCEL_Y;
      fAccel[2] = ACCEL_Z;

      //IQMath Conversion
      QAccel[0] = _IQ16(fAccel[0]);
      QAccel[1] = _IQ16(fAccel[1]);
      QAccel[2] = _IQ16(fAccel[2]);
      QGyro[0] = _IQ16(fGyro[0]);
      QGyro[1] = _IQ16(fGyro[1]);
      QGyro[2] = _IQ16(fGyro[2]);

      ComplementaryFilter(QAccel, QGyro, &pitch, &roll);
      UARTprintf("Pitch: %d\nRoll: %d\n\n", (int)(pitch*10), (int)(roll*10)); //Scaled it to make it easier to view on the terminal
    }

}
