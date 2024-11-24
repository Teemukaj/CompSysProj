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
   uint_t pinValue = PIN_getOutputValue( Board_LED0 );
   pinValue = !pinValue;
   PIN_setOutputValue( led0Handle, Board_LED0, pinValue );
}

// Button1 keskeytyksen käsittelijäfunktio
// Käytä globaaleja muuttujia, ei argumentteja!!!
void button1Fxn(PIN_Handle handle, PIN_Id pinId) {

   // Vaihdetaan led-pinnin tilaa negaatiolla
   uint_t pinValue = PIN_getOutputValue( Board_LED1 );
   pinValue = !pinValue;
   PIN_setOutputValue( led1Handle, Board_LED1, pinValue );
}

int main(void) {

   Board_initGeneral();

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

   BIOS_start();

   return (0);
}
