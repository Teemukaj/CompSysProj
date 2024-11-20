/* C Standard library */
#include <stdio.h>
#include <string.h>
#include <math.h>

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
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

/* Board Header files */
#include "Board.h"
#include "sensors/opt3001.h"
#include "sensors/mpu9250.h"
#include "buzzer.h"

/* Task */
#define STACKSIZE 2048
Char mainMenuTaskStack[STACKSIZE];
Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];
Char mpuTaskStack[STACKSIZE];
Char moveTaskStack[STACKSIZE];
Char communicationTaskStack[STACKSIZE];


/* Function prototypes */ //Onko tarpeellista ?
//void mainMenuTaskFxn(UArg arg0, UArg arg1);
//void uartTaskFxn(UArg arg0, UArg arg1);
//void moveTaskFxn(UArg arg0, UArg arg1);
//void communicationTaskFxn(UArg arg0, UArg arg1);
//void MPUsensorFxn(UArg arg0, UArg arg1);

/* Morse koodin pituudet (ms) */
#define DOT_DURATION     200000
#define LINE_DURATION    800000
#define SPACE_DURATION   1000000

/* Alustetaan buzzer */
//#define BUZZER_PIN  Board_BUZZER
//static PIN_Handle buzzerHandle;
//static PIN_State buzzerState;
static PIN_Handle hBuzzer;
static PIN_State sBuzzer;

/* Alustetaan Mailbox taskien väliseen viestittelyyn */
//typedef char msg_Buzzer;
//Mailbox_Handle mail_Buzzer;


/* Morse-koodin jaettu bufferi */ //Ei luultavasti tarvetta tälle
//#define MORSE_CODE_BUFFER_SIZE 256
//char morseCodeBuffer[MORSE_CODE_BUFFER_SIZE];
//volatile int morseCodeWriteIndex = 0;

//Tilakone
enum state { WAITING=1, DATA_READY, SEND_DATA, BUZZER, SEND_UART };
enum state programState = WAITING;

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

//Globaali muuttuja tallentamaan viimeisen havainnon indeksi taulukossa
volatile int lastSample = 0;

//globaali muuttuja havaitun liikkeen morse-arvolle (. tai - tai väli)
char message = 'a';

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

// Alustetaan buzzerin pin
//PIN_Config buzzerConfig[] = {
//    BUZZER_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//    PIN_TERMINATE
//};

PIN_Config cBuzzer[] = {
  Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};
/* Ei enää tarvetta, poista ennen palautusta
void buttonFxn(PIN_Handle handle, PIN_Id pinId) {

    // JTKJ: Teht v  1. Vilkuta jompaa kumpaa ledi
    // JTKJ: Exercise 1. Blink either led of the device


    // Vaihdetaan led-pinnin tilaa negaatiolla
    uint_t pinValue = PIN_getOutputValue( Board_LED0 );
    pinValue = !pinValue;
    PIN_setOutputValue( ledHandle, Board_LED0, pinValue );

}
*/

/* Task Functions */
Void uartTaskFxn(UArg arg0, UArg arg1) {

    UART_Handle uart;
    UART_Params uartParams;
    // JTKJ: Teht v  4. Lis   UARTin alustus: 9600,8n1
    // JTKJ: Exercise 4. Setup here UART connection as 9600,8n1
    UART_Params_init(&uartParams);
    uartParams.baudRate = 9600; // 9600 baud rate
    uartParams.dataLength = UART_LEN_8; // 8
    uartParams.parityType = UART_PAR_NONE; // n
    uartParams.stopBits = UART_STOP_ONE; // 1

    //Avataan UART
    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
        System_abort("Error opening the UART");
    }

    while (1) {
        if (programState == SEND_UART)  {
            char code[20];
            if (message == '.') {
                sprintf(code,".\r\n\0");
                UART_write(uart, code, strlen(code)+1);
                //Resetoidaan messagen arvo
                message = 'a';
                //Tilanvaihto
                programState = WAITING;
            } else if (message == '-') {
                sprintf(code,"-\r\n\0");
                UART_write(uart, code, strlen(code)+1);
                //Resetoidaan messagen arvo
                message = 'a';
                //Tilanvaihto
                programState = WAITING;
            } else if (message == ' ') {
                sprintf(code," \r\n\0");
                UART_write(uart, code, strlen(code)+1);
                //Resetoidaan messagen arvo
                message = 'a';
                //Tilanvaihto
                programState = WAITING;
            } else {
                System_printf("Ei oikeaa merkkiä\n");
                System_flush();
                //Resetoidaan char message
                message = 'a';
                //Tilanvaihto
                programState = WAITING;
            }
        }
        Task_sleep(100000 / Clock_tickPeriod);
    }

}

// Funktio summerin kontrollointiin
// TODO: Integrointi muuhun ohjelmaan
Void buzzerController(char signal) {
    if (signal == '.') {
        // Piste: Lyhyt piippaus
        PIN_setOutputValue(buzzerHandle, BUZZER_PIN, 1); // Summeri päälle
        Task_sleep(DOT_DURATION * (1000 / Clock_tickPeriod));
        PIN_setOutputValue(buzzerHandle, BUZZER_PIN, 0); // Summeri offille
    } else if (signal == '-') {
        // Viiva: Pitkä piippaus
        PIN_setOutputValue(buzzerHandle, BUZZER_PIN, 1); // Summeri päälle
        Task_sleep(LINE_DURATION * (1000 / Clock_tickPeriod));
        PIN_setOutputValue(buzzerHandle, BUZZER_PIN, 0); // Summeri offille
    } else if (signal == ' ') {
        // Väli: "Hiljainen hetki"
        PIN_setOutputValue(buzzerHandle, BUZZER_PIN, 0); // Varmistetaan, että summeri on offilla
        Task_sleep(SPACE_DURATION * (1000 / Clock_tickPeriod));
    }
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
    int curSample = 0;
    while (1) {
             if(takeSamples && programState == WAITING) {
            //Taulukossa on vain 20 jäsentä, joten aloita tallennus alusta kun mennään yli 20 havainnon
            curSample = (lastSample + 1) % 20;
            // MPU ask data
            mpu9250_get_data(&i2cMPU, &accx, &accy, &accz, &gyx, &gyy, &gyz);
            //char debug_str[80];
            //uint32_t tickStart;
            if(hasClockStarted == false) {
                tickStart = Clock_getTicks();
                hasClockStarted = true;
            }
            uint32_t currentTick = Clock_getTicks() - tickStart;
            //Save data to the struct-array for each row j
            //Acceleration
            samples[curSample].accel.ax = accx;
            samples[curSample].accel.ay = accy;
            samples[curSample].accel.az = accz;
            //Gyro
            samples[curSample].gyro.gx = gyx;
            samples[curSample].gyro.gy = gyy;
            samples[curSample].gyro.gz = gyz;
            //Timestamp
            samples[curSample].timestamp = currentTick;
            //uint32_t currentTick = Clock_getTicks();
            //sprintf(debug_str,"tick:%u, x:%.2f, y:%.2f, z:%.2f, gx:%.2f, gy:%.2f, gz:%.2f\n",currentTick, accx,accy,accz, gyx, gyy, gyz);
            //sprintf(debug_str,"%u, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",currentTick, accx,accy,accz, gyx, gyy, gyz);
            //System_printf(debug_str);
            //System_flush();
            //printf("MPU data x-acc: %.2f\n", ax);
            //printf("MPU data y-acc: %.2f\n", ay);
            //printf("MPU data z-acc: %.2f\n", az);
            //printf("MPU data x-gyro: %.2f\n", gx);
            //printf("MPU data y-gyro: %.2f\n", gy);
            //printf("MPU data z-gyro: %.2f\n", gz);
            lastSample = curSample;
            programState = DATA_READY;


        }
             // Sleep 100ms
             Task_sleep(100000 / Clock_tickPeriod);
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
    const float DEADZONE = 0.3; // kynnys (deadzone) erotukselle, jotta se voidaan tunnistaa liikkeeksi.
    const float ELIMIT = 0.1; // raja-arvo muiden akselien suhteen tapahtuvalle liikkeelle

    while (1) {
        if(programState == DATA_READY) {
            //Viimeisen ja sitä edellisen havainnon indeksi
            int currentSample = lastSample;
            int previousSample = (lastSample + 20 - 1) % 20;
            //bool liike_x = false, liike_y = false, liike_z = false;

            //System_printf("Data kerätty, yritetään havaita liikettä liikettä\n");
            //System_flush();
            //System_printf("nykyinen indeksi: %d\n, edellinen indeksi: %d\n,", currentSample, previousSample);
            // Liikkeen tunnistuksen logiikka. Lasketaan delta(x,y,z) eli peräkkäisten näytteiden kiihtyvyyden ero. Mikäli se ylittää kynnyksen
            // (deadzone) ja muiden akselien suhteen liike on tarpeeksi pientä niin tunnistetaan se liikkeeksi x,y,z.
            float delta_X = samples[currentSample].accel.ax - samples[previousSample].accel.ax;
            float delta_Y = samples[currentSample].accel.ay - samples[previousSample].accel.ay;
            float delta_Z = samples[currentSample].accel.az - samples[previousSample].accel.az;

            if (delta_X > DEADZONE && fabs(delta_Y) < ELIMIT && fabs(delta_Z) < ELIMIT) {
                //liike_x = true;
                System_printf("Liike positiiviseen suuntaan x-akselilla\n");
                System_flush();
                // Tallennetaan '-' globaalin muuttujaan ja tilamuutos
                message = '-';
                //Nollataan kiihtyvyysarvot currentSample-indeksissä alkuarvoon
                samples[currentSample].accel.ax = 0;
                samples[currentSample].accel.ay = 0;
                samples[currentSample].accel.az = -1.0;
                programState = BUZZER;
            } else if (delta_Y > DEADZONE && fabs(delta_X) < ELIMIT && fabs(delta_Z) < ELIMIT) {
                //liike_y = true;
                System_printf("Liike positiiviseen suuntaan y-akselilla\n");
                System_flush();
                // Tallennetaan '.' globaalin muuttujaan ja tilamuutos
                message = '.';
                //Nollataan kiihtyvyysarvot currentSample-indeksissä alkuarvoon
                samples[currentSample].accel.ax = 0;
                samples[currentSample].accel.ay = 0;
                samples[currentSample].accel.az = -1.0;
                programState = BUZZER;
            } else if (delta_Z > (DEADZONE + 0.1) && fabs(delta_X) < (ELIMIT + 0.1) && fabs(delta_Y) < (ELIMIT + 0.1)) {
                //liike_z = true;
                System_printf("Liike positiiviseen suuntaan z-akselilla\n");
                System_flush();
                // Tallennetaan ' ' globaalin muuttujaan ja tilamuutos
                message = ' ';
                //Nollataan kiihtyvyysarvot currentSample-indeksissä alkuarvoon
                samples[currentSample].accel.ax = 0;
                samples[currentSample].accel.ay = 0;
                samples[currentSample].accel.az = -1.0;
                programState = BUZZER;
            } else {
                programState = WAITING;
            }
        //programState = WAITING;
        }
        //Pienempi sleep niin tarkistetaan useammin kuin dataa kerätään
        Task_sleep(50000 / Clock_tickPeriod);
    }
}

Void mainMenuTaskFxn(UArg arg0, UArg arg1) {
    //char userInput;

    while (1) {
        //System_printf("=== Main Menu ===\n");
        Task_sleep(100000 / Clock_tickPeriod);

    }
}

void communicationTaskFxn(UArg arg0, UArg arg1) {
    //Toteutus
    while (1) {
        if(programState == BUZZER) {
            System_printf("CommunicationTask toteutus\n");
            System_flush();
            if (message == '.') {
                // Piste: Lyhyt piippaus ja LED välähdys
                System_printf("Piste\n");
                System_flush();
                // Summeri ja LED päälle
                buzzerOpen(hBuzzer);
                buzzerSetFrequency(2000);
                //PIN_setOutputValue(buzzerHandle, BUZZER_PIN, 1);
                PIN_setOutputValue(ledHandle, Board_LED0, 1);

                // Odotetaan hetki
                Task_sleep(DOT_DURATION / Clock_tickPeriod);

                // Sammutetaan summeri ja LED
                //PIN_setOutputValue(buzzerHandle, BUZZER_PIN, 0);
                buzzerClose();
                PIN_setOutputValue(ledHandle, Board_LED0, 0);
                //Tilanvaihto
                programState = SEND_UART;
            } else if (message == '-') {
                // Viiva: Pitkä piippaus ja LED välähdys
                System_printf("Viiva\n");
                System_flush();
                // Summeri ja LED päälle
                buzzerOpen(hBuzzer);
                buzzerSetFrequency(2000);
                //PIN_setOutputValue(buzzerHandle, BUZZER_PIN, 1);
                PIN_setOutputValue(ledHandle, Board_LED0, 1);

                // Odotetaan hetki
                Task_sleep(LINE_DURATION / Clock_tickPeriod);

                // Sammutetaan summeri ja LED
                buzzerClose();
                //PIN_setOutputValue(buzzerHandle, BUZZER_PIN, 0);
                PIN_setOutputValue(ledHandle, Board_LED0, 0);
                //Tilanvaihto
                programState = SEND_UART;
            } else if (message == ' ') {
                // Väli: Pitkä piippaus korkeammalla taajuudella
                System_printf("Välilyönti\n");
                System_flush();
                // Summeri ja LED päälle
                buzzerOpen(hBuzzer);
                buzzerSetFrequency(5000);
                //PIN_setOutputValue(buzzerHandle, BUZZER_PIN, 1);
                PIN_setOutputValue(ledHandle, Board_LED0, 1);

                // Odotetaan hetki
                Task_sleep(SPACE_DURATION / Clock_tickPeriod);

                // Sammutetaan summeri ja LED
                buzzerClose();
                //PIN_setOutputValue(buzzerHandle, BUZZER_PIN, 0);
                PIN_setOutputValue(ledHandle, Board_LED0, 0);
                //Tilanvaihto
                programState = SEND_UART;
            } else {
                System_printf("Ei oikeaa merkkiä\n");
                System_flush();
                //Resetoidaan char message
                message = 'a';
                //Tilanvaihto
                programState = WAITING;
            }
        }
        //Sleep 1000ms
        Task_sleep(100000 / Clock_tickPeriod);
    }
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
    Task_Handle communicationTaskHandle;
    Task_Params communicationTaskParams;
    Task_Handle mainMenuTaskHandle;
    Task_Params mainMenuTaskParams;

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

    // Asetetaan painonappi-pinnille keskeytyksen käsittelijäksi, EI käytössä
    // funktio buttonFxn
    //if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
    //   System_abort("Error registering button callback function");
    //}
    // JTKJ: Teht v  2. Ota i2c-v yl  k ytt  n ohjelmassa
    // JTKJ: Exercise 2. Initialize i2c bus
    Board_initI2C();
    // JTKJ: Teht v  4. Ota UART k ytt  n ohjelmassa
    // JTKJ: Exercise 4. Initialize UART
    Board_initUART();

    // JTKJ: Teht v  1. Ota painonappi ja ledi ohjelman k ytt  n
    //       Muista rekister id  keskeytyksen k sittelij  painonapille
    // JTKJ: Exercise 1. Open the button and led pins
    //       Remember to register the above interrupt handler for button

    // Open MPU power pin
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
        System_abort("Pin open failed!");
    }

    // Open buzzer pin
    //buzzerHandle = PIN_open(&buzzerState, buzzerConfig);
    //if (!buzzerHandle) {
    //    System_abort("Error initializing buzzer pin\n");
    //}

    // Buzzer
    hBuzzer = PIN_open(&sBuzzer, cBuzzer);
    if (hBuzzer == NULL) {
      System_abort("Pin open failed!");
    }

    // Mailbox buzzerTaskFxn
    /*
    Mailbox_Params mboxParams;
    Mailbox_Params_init(&mboxParams);
    mail_Buzzer = Mailbox_create(sizeof(msg_Buzzer), 10, &mboxParams, NULL);
    if (mail_Buzzer == NULL) {
        System_abort("Mailbox create failed!");
    }
    */

    /* Task Initializations */

    Task_Params_init(&mpuTaskParams);
    mpuTaskParams.stackSize = STACKSIZE;
    mpuTaskParams.stack = &mpuTaskStack;
    mpuTaskParams.priority=2;
    mpuTask = Task_create((Task_FuncPtr)MPUsensorFxn, &mpuTaskParams, NULL);
    if (mpuTask == NULL) {
        System_abort("Task create failed!");
    }

    Task_Params_init(&moveTaskParams);
    moveTaskParams.stackSize = STACKSIZE;
    moveTaskParams.stack = &moveTaskStack;
    moveTaskParams.priority=1;
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

    Task_Params_init(&communicationTaskParams);
    communicationTaskParams.stackSize = STACKSIZE;
    communicationTaskParams.stack = &communicationTaskStack;
    communicationTaskParams.priority=2;
    communicationTaskHandle = Task_create(communicationTaskFxn, &communicationTaskParams, NULL);
    if (communicationTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    Task_Params_init(&mainMenuTaskParams);
    mainMenuTaskParams.stackSize = STACKSIZE;
    mainMenuTaskParams.stack = &mainMenuTaskStack;
    mainMenuTaskParams.priority = 2;
    mainMenuTaskHandle = Task_create(mainMenuTaskFxn, &mainMenuTaskParams, NULL);
    if (mainMenuTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
