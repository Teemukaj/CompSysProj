/* C Standard library */
#include <stdio.h>

/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
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



/* Board Header files */
#include "Board.h"
#include "sensors/opt3001.h"
#include "sensors/mpu9250.h"

/* Task */
#define STACKSIZE 2048
Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];
Char mpuTaskStack[STACKSIZE];
Char moveTaskStack[STACKSIZE];

// JTKJ: Teht v  3. Tilakoneen esittely
// JTKJ: Exercise 3. Definition of the state machine
enum state { WAITING=1, DATA_READY };
enum state programState = WAITING;

// JTKJ: Teht v  3. Valoisuuden globaali muuttuja
// JTKJ: Exercise 3. Global variable for ambient light
double ambientLight = -1000.0;

//MPU globaalit muuttujat ja alustukset

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
//Struck johon tallennetaan havainnot
struct mpu_sample_t {
    uint32_t timestamp; //aikaleima
    struct accelerometer_t accel;  // Kiihtyvyysanturin tiedot (ax, ay, az)
    struct gyroscope_t gyro;  // Gyroskoopin tiedot (gx, gy, gz)
};

//Globaali muuttuja tallentamaan viimeisen havainnon indeksi
int lastSample = 0;

//Alustetaan taulukkoon arvot jotka vastaavat laitteen paikallaanoloa z-acc = -1.00 , muut arvot 0.00
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


// JTKJ: Teht v  1. Lis   painonappien RTOS-muuttujat ja alustus
// JTKJ: Exercise 1. Add pins RTOS-variables and configuration here

// RTOS:n globaalit muuttujat pinnien käyttöön
static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle ledHandle;
static PIN_State ledState;

// Pinnien alustukset, molemmille pinneille oma konfiguraatio
// Vakio BOARD_BUTTON_0 vastaa toista painonappia
PIN_Config buttonConfig[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tällä vakiolla
};

// Vakio Board_LED0 vastaa toista lediä

PIN_Config ledConfig[] = {
   Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tällä vakiolla
};

void buttonFxn(PIN_Handle handle, PIN_Id pinId) {

    // JTKJ: Teht v  1. Vilkuta jompaa kumpaa ledi
    // JTKJ: Exercise 1. Blink either led of the device


    // Vaihdetaan led-pinnin tilaa negaatiolla
    uint_t pinValue = PIN_getOutputValue( Board_LED0 );
    pinValue = !pinValue;
    PIN_setOutputValue( ledHandle, Board_LED0, pinValue );


}

/* Task Functions */
Void uartTaskFxn(UArg arg0, UArg arg1) {

    /*
    // JTKJ: Teht v  4. Lis   UARTin alustus: 9600,8n1
    // JTKJ: Exercise 4. Setup here UART connection as 9600,8n1

    while (1) {

        // JTKJ: Teht v  3. Kun tila on oikea, tulosta sensoridata merkkijonossa debug-ikkunaan
        //       Muista tilamuutos
        // JTKJ: Exercise 3. Print out sensor data as string to debug window if the state is correct
        //       Remember to modify state
        if (programState == DATA_READY)  {

            char merkkijono[20];
            sprintf(merkkijono,"%lf\n",ambientLight);
            //System_printf(merkkijono);
            System_printf("uart tulostus lux-arvo: %s\n",merkkijono);
            //System_flush();

            // Tilasiirtymä DATA_READY -> WAITING
            programState= WAITING;
        }

        // JTKJ: Teht v  4. L het  sama merkkijono UARTilla
        // JTKJ: Exercise 4. Send the same sensor data string with UART

        // Just for sanity check for exercise, you can comment this out
        //System_printf("uartTask\n");
        //System_flush();

        // Once per second, you can modify this (1000000 / Clock_tickPeriod) is equal to 1s)
        Task_sleep(1000000 / Clock_tickPeriod);
    }
    */
}

Void MPUsensorFxn(UArg arg0, UArg arg1) {

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
    bool hasClockStarted = false;
    uint32_t tickStart = 0;
    bool takeSamples = true;
    while (takeSamples) {
        //Taulukossa on vain 20 jäsentä, joten aloita tallennus alusta kun mennään yli 20 havainnon
        if(lastSample == 20) {
            //takeSamples = false;
            //System_printf("indeksi vaihdettu 20 -> 0 \n");
            lastSample = 0;
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
        samples[lastSample].accel.ax = accx;
        samples[lastSample].accel.ay = accy;
        samples[lastSample].accel.az = accz;
        //Gyro
        samples[lastSample].gyro.gx = gyx;
        samples[lastSample].gyro.gy = gyy;
        samples[lastSample].gyro.gz = gyz;
        //Timestamp
        samples[lastSample].timestamp = currentTick;
        //uint32_t currentTick = Clock_getTicks();
        //sprintf(debug_str,"tick:%u, x:%.2f, y:%.2f, z:%.2f, gx:%.2f, gy:%.2f, gz:%.2f\n",currentTick, accx,accy,accz, gyx, gyy, gyz);
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
        lastSample++;
    }
    /*
    //printtaus taulukosta
    printf("printtaus taulukosta tarkistuksena \n");
    int i = 0;
    for(i = 0; i < 10 ; i++) {
        //printf("accel.ax: %f\n", samples[i].accel.ax);
        printf("%u, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", samples[i].timestamp, samples[i].accel.ax, samples[i].accel.ay, samples[i].accel.az,
               samples[i].gyro.gx, samples[i].gyro.gy, samples[i].gyro.gz);

    }
    */

    // Program never gets here..
    // MPU close i2c
    // I2C_close(i2cMPU);
    // MPU power off
    // PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_OFF);
}

Void moveTaskFxn(UArg arg0, UArg arg1) {

    //TODO: Liikkentunnistus-taskin koodi
    //liikedata structista samples[20], 20 viimeistä havaintoa ~100ms välein
    //data: samples[i].timestamp, samples[i].accel.ax, samples[i].accel.ay, samples[i].accel.az, samples[i].gyro.gx, samples[i].gyro.gy, samples[i].gyro.gz)
    //Viimeisen havainnon indeksi lastSample. 20 viimeistä havaintoa, indeksi pyörähtää 20->0 aina kun 20 havaintoa tulee täyteen
    //Täytyy toteuttaa jonkinlainen tilakoneen tilaehto, esim tarkistetaan vain kun on uutta dataa tjmts, voi jättää tehtäväksi myöhemmäksi
    //3 liikettä: ylös-alas liike suuntiin x,y ja z-akseli. Varmaan olisi hyvä valita kynnysarvo ja tarkistetaan aika
    //kuin nopeasti liike on tehtävä. Esim x-akselin suuntaan kiihtyvyysarvot voisivat mennä 0, 0.25, -0.15, -0.01, 0.02
    //kääntöön liittyvistä asentomuutoksien kulmanopeuksista ei välitetä ainakaan tässä vaiheessa.

}

Int main(void) {

    // Task variables
    //Task_Handle sensorTaskHandle;
    //Task_Params sensorTaskParams;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;
    Task_Handle mpuTask;
    Task_Params mpuTaskParams;
    Task_Handle moveTaskHandle;
    Task_Params moveTaskParams;

    // Initialize board
    Board_initGeneral();

    // Otetaan pinnit käyttöön ohjelmassa
    buttonHandle = PIN_open(&buttonState, buttonConfig);
    if(!buttonHandle) {
       System_abort("Error initializing button pins\n");
    }
    ledHandle = PIN_open(&ledState, ledConfig);
    if(!ledHandle) {
       System_abort("Error initializing LED pins\n");
    }

    // Asetetaan painonappi-pinnille keskeytyksen käsittelijäksi
    // funktio buttonFxn
    if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
       System_abort("Error registering button callback function");
    }
    // JTKJ: Teht v  2. Ota i2c-v yl  k ytt  n ohjelmassa
    // JTKJ: Exercise 2. Initialize i2c bus
    Board_initI2C();
    // JTKJ: Teht v  4. Ota UART k ytt  n ohjelmassa
    // JTKJ: Exercise 4. Initialize UART

    // JTKJ: Teht v  1. Ota painonappi ja ledi ohjelman k ytt  n
    //       Muista rekister id  keskeytyksen k sittelij  painonapille
    // JTKJ: Exercise 1. Open the button and led pins
    //       Remember to register the above interrupt handler for button

    // Open MPU power pin
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
        System_abort("Pin open failed!");
    }

    Task_Params_init(&mpuTaskParams);
    mpuTaskParams.stackSize = STACKSIZE;
    mpuTaskParams.stack = &mpuTaskStack;
    mpuTask = Task_create((Task_FuncPtr)MPUsensorFxn, &mpuTaskParams, NULL);
    if (mpuTask == NULL) {
        System_abort("Task create failed!");
    }

    /* Task */

    Task_Params_init(&moveTaskParams);
    moveTaskParams.stackSize = STACKSIZE;
    moveTaskParams.stack = &moveTaskStack;
    moveTaskParams.priority=2;
    moveTaskHandle = Task_create(moveTaskFxn, &moveTaskParams, NULL);
    if (moveTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority=2;
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
