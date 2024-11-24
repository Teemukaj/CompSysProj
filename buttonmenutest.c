/* C Standard library */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

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
#include "Board.h"
#include "sensors/mpu9250.h"

/* Board Header files */
#include "Board.h"
#include "sensors/opt3001.h"
#include "sensors/mpu9250.h"

/* Task */
#define STACKSIZE 2048
Char mainMenuTaskStack[STACKSIZE];

// Tilakoneen esittely
enum state { WAITING=1, DATA_READY, MENU, BOTH_PRESSED, RECEIVE_DATA, SEND_DATA };
enum state programState = MENU;

// RTOS:n globaalit muuttujat pinnien käyttöön
static PIN_Handle button0Handle;
static PIN_State button0State;
static PIN_Handle button1Handle;
static PIN_State button1State;
static PIN_Handle led0Handle;
static PIN_State led0State;
static PIN_Handle led1Handle;
static PIN_State led1State;

// Pinnien alustukset, molemmille pinneille oma konfiguraatio
// Vakio BOARD_BUTTON_0 vastaa toista painonappia
PIN_Config button0Config[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tällä vakiolla
};

PIN_Config button1Config[] = {
   Board_BUTTON1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tällä vakiolla
};

// Vakio Board_LED0 vastaa toista lediä
PIN_Config led0Config[] = {
   Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tällä vakiolla
};

// Vakio Board_LED1 vastaa toista lediä
PIN_Config led1Config[] = {
   Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tällä vakiolla
};

// Button0 keskeytyksen käsittelijäfunktio
// Käytä globaaleja muuttujia, ei argumentteja!!!
void button0Fxn(PIN_Handle handle, PIN_Id pinId) {

    // Vaihdetaan led-pinnin tilaa negaatiolla
    //uint_t pinValue = PIN_getOutputValue( Board_LED0 );
    //pinValue = !pinValue;
    //PIN_setOutputValue( led0Handle, Board_LED0, pinValue );
    //programState vaihto MENU -> RECEIVE_DATA tai RECEIVE_DATA - > MENU
    if(programState == MENU) {
        System_printf("Siirrytään RECEIVE_DATA-tilaan.\n");
        System_flush();
        //LED0 päälle
        PIN_setOutputValue( led1Handle, Board_LED1, 1 );
        programState = RECEIVE_DATA;
    }else if(programState == RECEIVE_DATA) {
        System_printf("Siirrytään MENU-tilaan.\n");
        System_flush();
        //LED0 pois
        PIN_setOutputValue( led1Handle, Board_LED1, 0 );
        programState = MENU;
    }else {
        System_printf("Odota paluuta RECEIVE_DATA-tilaan ja paina nappia uudestaan\n");
        System_flush();
    }
    }

// Button1 keskeytyksen käsittelijäfunktio
// Käytä globaaleja muuttujia, ei argumentteja!!!
void button1Fxn(PIN_Handle handle, PIN_Id pinId) {

    // Vaihdetaan led-pinnin tilaa negaatiolla
    //uint_t pinValue = PIN_getOutputValue( Board_LED1 );
    //pinValue = !pinValue;
    //PIN_setOutputValue( led1Handle, Board_LED1, pinValue );
    //programState vaihto MENU -> SEND_DATA tai SEND_DATA - > MENU
    if(programState == MENU) {
        System_printf("Siirrytään SEND_DATA-tilaan.\n");
        System_flush();
        //LED1 päälle
        PIN_setOutputValue( led1Handle, Board_LED1, 1 );
        programState = SEND_DATA;
    }else if(programState == SEND_DATA) {
        System_printf("Siirrytään MENU-tilaan.\n");
        System_flush();
        //LED1 pois
        PIN_setOutputValue( led1Handle, Board_LED1, 0 );
        programState = MENU;
   }else {
        System_printf("Odota paluuta SEND_DATA-tilaan ja paina nappia uudestaan\n");
        System_flush();
   }
}

// Tästä voi muutella menutaskin "nukkumisaikaa"
void menuSleep() {
    Task_sleep(100 / Clock_tickPeriod);
}

void menuTaskFxn(UArg arg0, UArg arg1) {
    while (1) {
        if(programState == MENU) {
            //MENU-tilan ohjeiden tulostus konsoliin
            System_printf("Paina nappia 0 koodin vastaanottoon. Paina nappia 1 liikkeiden tunnistukseen laitteelta.\n");
            System_flush();
            System_printf("LED1(punainen) palaa jos suoritus on käynnissä.\n");
            System_flush();
            System_printf("Palaa valikkoon painamalla nappia uudestaan.\n");
            System_flush();
        }
        //Sleep 10s
        Task_sleep(10000000 / Clock_tickPeriod);
    }
}

int main(void) {

    Task_Handle menuTaskHandle;
    Task_Params menuTaskParams;

    Board_initGeneral();

    Board_initI2C();
    // Initialize UART
    Board_initUART();

    // Otetaan pinnit käyttöön ohjelmassa
    button0Handle = PIN_open(&button0State, button0Config);
    if(!button0Handle) {
    System_abort("Error initializing button0 pins\n");
    }
    // Otetaan pinnit käyttöön ohjelmassa
    button1Handle = PIN_open(&button1State, button1Config);
    if(!button1Handle) {
    System_abort("Error initializing button1 pins\n");
    }
    led0Handle = PIN_open(&led0State, led0Config);
    if(!led0Handle) {
    System_abort("Error initializing LED pins\n");
    }
    led1Handle = PIN_open(&led1State, led1Config);
    if(!led1Handle) {
    System_abort("Error initializing LED pins\n");
    }
    // Asetetaan painonappi-pinnille keskeytyksen käsittelijäksi
    // funktio buttonFxn
    if (PIN_registerIntCb(button0Handle, &button0Fxn) != 0) {
    System_abort("Error registering button0 callback function");
    }
    // Asetetaan painonappi-pinnille keskeytyksen käsittelijäksi
    // funktio buttonFxn
    if (PIN_registerIntCb(button1Handle, &button1Fxn) != 0) {
    System_abort("Error registering button1 callback function");
    }

    /* Task Initializations */

    Task_Params_init(&menuTaskParams);
    menuTaskParams.stackSize = STACKSIZE;
    menuTaskParams.stack = &mainMenuTaskStack;
    menuTaskParams.priority = 3;
    menuTaskHandle = Task_create(menuTaskFxn, &menuTaskParams, NULL);
    if (menuTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();
    BIOS_start();

    return (0);
}
