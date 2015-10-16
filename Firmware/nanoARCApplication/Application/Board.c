/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ====================== Board.c =============================================
 *  This file is responsible for setting up the board specific items for the
 *  nanoARC Mainboard. This board file is valid for PCB version 1
 */


/*
 *  ====================== Includes ============================================
 */
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <driverlib/ioc.h>
#include <driverlib/udma.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#include <ti/drivers/PIN.h>
#include "Board.h"

/*
 *  ========================= IO driver initialization =========================
 *  From main, PIN_init(BoardGpioInitTable) should be called to setup safe
 *  settings for this board.
 *  When a pin is allocated and then de-allocated, it will revert to the state
 *  configured in this table.
*/

/*
 *  ========================= IO driver initialization =========================
 *  From main, PIN_init(BoardGpioInitTable) should be called to setup safe
 *  settings for this board.
 *  When a pin is allocated and then de-allocated, it will revert to the state
 *  configured in this table
*/
PIN_Config BoardGpioInitTable[] = {
	Board_MOTOR1_PH		| PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* Motor controller initially off */
	Board_MOTOR1_EN		| PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* Motor controller initially off */
	Board_MOTOR2_PH		| PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* Motor controller initially off */
	Board_MOTOR2_EN		| PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* Motor controller initially off */

	Board_GPIO_0		| PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* GPIO use initially output */
	Board_GPIO_1		| PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* GPIO use initially output */
	Board_GPIO_2		| PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* GPIO use initially output */
	Board_GPIO_3		| PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* GPIO use initially output */
	Board_GPIO_4		| PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* GPIO use initially output */

	Board_BATTERY		| PIN_INPUT_EN | PIN_NOPULL,											 /* Battery voltage sense	   */

    PIN_TERMINATE
};
/*============================================================================*/

/*
 *  ============================= UART begin ===================================
*/
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(UART_config, ".const:UART_config")
#pragma DATA_SECTION(uartCC26XXHWAttrs, ".const:uartCC26XXHWAttrs")
#endif

/* Include drivers */
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

/* UART objects */
UARTCC26XX_Object uartCC26XXObjects[CC2650_UARTCOUNT];

/* UART hardware parameter structure, also used to assign UART pins */
const UARTCC26XX_HWAttrs uartCC26XXHWAttrs[CC2650_UARTCOUNT] = {
    {    /* CC2650_UART0 */
        .baseAddr = UART0_BASE,
        .intNum = INT_UART0,
        .powerMngrId = PERIPH_UART0,
        .txPin = Board_UART_TX,
        .rxPin = Board_UART_RX,
        .ctsPin = PIN_UNASSIGNED,
        .rtsPin = PIN_UNASSIGNED
    },
};

/* UART configuration structure */
const UART_Config UART_config[] = {
    { &UARTCC26XX_fxnTable, &uartCC26XXObjects[0], &uartCC26XXHWAttrs[0] },
    { NULL, NULL, NULL }
};
/*
 *  ============================= UART end =====================================
*/
