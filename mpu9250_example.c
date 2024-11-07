/*
 * mpu9250_example.c
 *
 *  Created on: 29.10.2016
 *  Author: Teemu Leppanen / UBIComp / University of Oulu
 *
 */

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include "Board.h"
#include "sensors/mpu9250.h"
#include <stdio.h>
#include <stdbool.h>

#define STACKSIZE 2048
Char taskStack[STACKSIZE];

// MPU power pin global variables
static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;

// MPU power pin
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

// MPU uses its own I2C interface
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};

struct accelerometer_t {
    float ax, ay, az;
};

struct gyroscope_t {
    float gx, gy, gz;
};

struct mpu_status_t {
    int status_code;  // MPU:n tila
    int battery_level;  // Akun varausprosentti
};

struct mpu_sample_t {
    uint32_t timestamp;
    struct accelerometer_t accel;  // Kiihtyvyysanturin tiedot (ax, ay, az)
    struct gyroscope_t gyro;  // Gyroskoopin tiedot (gx, gy, gz)
};
struct mpu_sample_t samples[20] = {
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
{0, {0.00, 0.00, -1.00}, {0.00, 0.00, 0.00}},
};

Void sensorFxn(UArg arg0, UArg arg1) {

    float accx, accy, accz, gyx, gyy, gyz;

    I2C_Handle i2cMPU; // Own i2c-interface for MPU9250 sensor
    I2C_Params i2cMPUParams;

    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    // Note the different configuration below
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    // MPU power on
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

    // Wait 100ms for the MPU sensor to power up
    Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    // MPU open i2c
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2CMPU\n");
    }

    // MPU setup and calibration
    System_printf("MPU9250: Setup and calibration...\n");
    System_flush();

    mpu9250_setup(&i2cMPU);

    System_printf("MPU9250: Setup and calibration OK\n");
    System_flush();

    // Loop forever
    int j = 0;
    bool hasClockStarted = false;
    uint32_t tickStart = 0;
    bool takeSamples = true;
    while (takeSamples) {
        if(j >= 10) {
            takeSamples = false;
        }
        // MPU ask data
        mpu9250_get_data(&i2cMPU, &accx, &accy, &accz, &gyx, &gyy, &gyz);
        char debug_str[80];
        //uint32_t tickStart;
        if(hasClockStarted == false) {
            tickStart = Clock_getTicks();
            hasClockStarted = true;
        }
        uint32_t currentTick = Clock_getTicks() - tickStart;
        //Save data to the struct-array for each row j
        //Acceleration
        samples[j].accel.ax = accx;
        samples[j].accel.ay = accy;
        samples[j].accel.az = accz;
        //Gyro
        samples[j].gyro.gx = gyx;
        samples[j].gyro.gy = gyy;
        samples[j].gyro.gz = gyz;
        //Timestamp
        samples[j].timestamp = currentTick;
        //uint32_t currentTick = Clock_getTicks();
        //sprintf(debug_str,"tick:%u, x:%.2f, y:%.2f, z:%.2f, gx:%.2f, gy:%.2f, gz:%.2f\n",currentTick, ax,ay,az, gx, gy, gz);
        sprintf(debug_str,"%u, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",currentTick, accx,accy,accz, gyx, gyy, gyz);
        System_printf(debug_str);
        //printf("MPU data x-acc: %.2f\n", ax);
        //printf("MPU data y-acc: %.2f\n", ay);
        //printf("MPU data z-acc: %.2f\n", az);
        //printf("MPU data x-gyro: %.2f\n", gx);
        //printf("MPU data y-gyro: %.2f\n", gy);
        //printf("MPU data z-gyro: %.2f\n", gz);
        // Sleep 100ms
        Task_sleep(100000 / Clock_tickPeriod);
        j++;
    }
    //printtaus taulukosta
    printf("printtaus taulukosta tarkistuksena \n");
    int i = 0;
    for(i = 0; i < 10 ; i++) {
        //printf("accel.ax: %f\n", samples[i].accel.ax);
        printf("%u, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", samples[i].timestamp, samples[i].accel.ax, samples[i].accel.ay, samples[i].accel.az,
               samples[i].gyro.gx, samples[i].gyro.gy, samples[i].gyro.gz);

    }

    // Program never gets here..
    // MPU close i2c
    // I2C_close(i2cMPU);
    // MPU power off
    // PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_OFF);
}

int main(void) {

    Task_Handle task;
    Task_Params taskParams;

    Board_initGeneral();
    Board_initI2C();

    // Open MPU power pin
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
        System_abort("Pin open failed!");
    }

    Task_Params_init(&taskParams);
    taskParams.stackSize = STACKSIZE;
    taskParams.stack = &taskStack;
    task = Task_create((Task_FuncPtr)sensorFxn, &taskParams, NULL);
    if (task == NULL) {
        System_abort("Task create failed!");
    }

    System_printf("Hello World\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
