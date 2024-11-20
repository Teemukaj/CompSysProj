/* C Standard library */
#include <stdio.h>
#include <string.h>

/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"

/* Task */
#define STACKSIZE 2048
Char uartTaskStack[STACKSIZE];

// UART-kahva
UART_Handle uart;

// Funktio, joka lähettää merkin UARTin kautta
void sendToUART(const char* data) {
    UART_write(uart, data, strlen(data));
    Task_sleep(100000 / Clock_tickPeriod);  // Pieni viive viestien välillä
}

/* Testikoodi Morse-viestin "aasi" lähettämiseksi */
Void uartTaskFxn(UArg arg0, UArg arg1) {
    UART_Params uartParams;

    // UART-parametrien alustaminen
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode = UART_MODE_BLOCKING;
    uartParams.baudRate = 9600;  // Nopeus 9600baud
    uartParams.dataLength = UART_LEN_8;
    uartParams.parityType = UART_PAR_NONE;
    uartParams.stopBits = UART_STOP_ONE;

    // Avataan UART
    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
        System_abort("Error opening the UART");
    }

    // Lähetetään sana "aasi" Morse-koodattuna
    char message[20];
    // a: .-
    sprintf(message,".\r\n\0");
    UART_write(uart, message, strlen(message)+1);
    sprintf(message,"-\r\n\0");
    UART_write(uart, message, strlen(message)+1);
    sprintf(message," \r\n\0");
    UART_write(uart, message, strlen(message)+1);
    // a: .-
    sprintf(message,".\r\n\0");
    UART_write(uart, message, strlen(message)+1);
    sprintf(message,"-\r\n\0");
    UART_write(uart, message, strlen(message)+1);
    sprintf(message," \r\n\0");
    UART_write(uart, message, strlen(message)+1);
    // s: ...
    sprintf(message,".\r\n\0");
    UART_write(uart, message, strlen(message)+1);
    sprintf(message,".\r\n\0");
    UART_write(uart, message, strlen(message)+1);
    sprintf(message,".\r\n\0");
    UART_write(uart, message, strlen(message)+1);
    sprintf(message," \r\n\0");
    UART_write(uart, message, strlen(message)+1);
    // i: ..
    sprintf(message,".\r\n\0");
    UART_write(uart, message, strlen(message)+1);
    sprintf(message,".\r\n\0");
    UART_write(uart, message, strlen(message)+1);
    sprintf(message," \r\n\0");
    UART_write(uart, message, strlen(message)+1);
    //Sana loppuu kahteen välilyöntiin
    sprintf(message," \r\n\0");
    UART_write(uart, message, strlen(message)+1);
    sprintf(message," \r\n\0");
    UART_write(uart, message, strlen(message)+1);
    /*
    sendToUART(".\r\n\0");  // a: .-
    sendToUART("-\r\n\0");
    sendToUART(" \r\n\0");  // Merkin väli

    sendToUART(".\r\n\0");  // a: .-
    sendToUART("-\r\n\0");
    sendToUART(" \r\n\0");  // Merkin väli

    sendToUART(".\r\n\0");  // s: ...
    sendToUART(".\r\n\0");
    sendToUART(".\r\n\0");
    sendToUART(" \r\n\0");  // Merkin väli

    sendToUART(".\r\n\0");  // i: ..
    sendToUART(".\r\n\0");

    // Viestin loppu (kolme välilyöntiä)
    sendToUART(" \r\n\0");
    sendToUART(" \r\n\0");
    sendToUART(" \r\n\0");
    */
    // Lopetetaan UART
    UART_close(uart);

    System_printf("Morse message sent.\n");
    System_flush();
}

/* Main-funktio */
Int main(void) {
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;

    // Alustetaan lauta
    Board_initGeneral();
    Board_initUART();

    // Luodaan UARTin tehtävä
    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority = 2;
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    // Käynnistetään BIOS
    BIOS_start();

    return (0);
}
