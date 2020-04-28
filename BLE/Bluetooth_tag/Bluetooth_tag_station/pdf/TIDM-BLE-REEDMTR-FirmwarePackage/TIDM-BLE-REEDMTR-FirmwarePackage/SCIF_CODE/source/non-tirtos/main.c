//*****************************************************************************
//  SENSOR CONTROLLER STUDIO EXAMPLE: LED BLINKER
//  Operating system: None
//
//  The Sensor Controller blinks two LEDs on the SmartRF06EB. For every N'th
//  LED toggling, a counter value (output.counter) is communicated to the
//  application, which uses this value to blink another two LEDs.
//
//  This examples also demonstrates starting and stopping the Sensor Controller
//  task by using the SmartRF06EB push buttons:
//  - LEFT: Starts the LED Blinker task
//  - RIGHT: Stops the LED Blinker task
//
//  This example application does not implement power management.
//
//
//  Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include "scif.h"
#include <driverlib/gpio.h>
#include <driverlib/cpu.h>
#include <driverlib/ioc.h>
#include <driverlib/prcm.h>
#include <driverlib/interrupt.h>
#include <driverlib/aon_rtc.h>
#include <inc/hw_nvic.h>
#include <inc/hw_ioc.h>
#include <inc/hw_gpio.h>


// Display error message if the SCIF driver has been generated with incorrect operating system setting
#ifndef SCIF_OSAL_NONE_H
    #error "Generated SCIF driver supports incorrect operating system. Please change to 'None' in the Sensor Controller Studio project panel and re-generate the driver."
#endif


#define BV(n)           (1 << (n))
#define IOC_O_IOCFG(n)  (IOC_O_IOCFG0 + (sizeof(uint32_t) * n))


// Pin mapping for SmartRF06EB
#define IOID_LED_1          IOID_25
#define IOID_LED_2          IOID_27
#define IOID_LED_3          IOID_7
#define IOID_LED_4          IOID_6
#define IOID_KEY_LEFT       IOID_15
#define IOID_KEY_RIGHT      IOID_18
#define IOID_KEY_UP         IOID_19
#define IOID_KEY_DOWN       IOID_12
#define IOID_KEY_SELECT     IOID_11




void ctrlReadyCallback(void) {

} // ctrlReadyCallback




void taskAlertCallback(void) {

    // Clear the ALERT interrupt source
    scifClearAlertIntSource();

    if (scifGetTaskIoStructAvailCount(SCIF_LED_BLINKER_TASK_ID, SCIF_STRUCT_OUTPUT) == 1) {
        SCIF_LED_BLINKER_OUTPUT_T* pOutput = scifGetTaskStruct(SCIF_LED_BLINKER_TASK_ID, SCIF_STRUCT_OUTPUT);
        if (pOutput->counter & 0x0001) {
            HWREG(GPIO_BASE + GPIO_O_DOUTSET31_0) = BV(IOID_LED_1);
        } else {
            HWREG(GPIO_BASE + GPIO_O_DOUTCLR31_0) = BV(IOID_LED_1);
        }
        if (pOutput->counter & 0x0002) {
            HWREG(GPIO_BASE + GPIO_O_DOUTSET31_0) = BV(IOID_LED_2);
        } else {
            HWREG(GPIO_BASE + GPIO_O_DOUTCLR31_0) = BV(IOID_LED_2);
        }
        scifHandoffTaskStruct(SCIF_LED_BLINKER_TASK_ID, SCIF_STRUCT_OUTPUT);
    }

    // Acknowledge the alert event
    scifAckAlertEvents();

} // taskAlertCallback




void main(void) {

    // Enable power domains
    PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
    PRCMLoadSet();
    while (PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON);

    // Enable the GPIO module
    PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
    PRCMPeripheralSleepEnable(PRCM_PERIPH_GPIO);
    PRCMPeripheralDeepSleepEnable(PRCM_PERIPH_GPIO);
    PRCMLoadSet();
    while (!PRCMLoadGet());

    // Setup GPIOs for LEDs and buttons
    HWREG(GPIO_BASE + GPIO_O_DOE31_0) &= ~(BV(IOID_KEY_LEFT) | BV(IOID_KEY_RIGHT) | BV(IOID_KEY_UP) | BV(IOID_KEY_DOWN));
    HWREG(GPIO_BASE + GPIO_O_DOE31_0) |= BV(IOID_LED_1) | BV(IOID_LED_2);
    HWREG(IOC_BASE + IOC_O_IOCFG(IOID_LED_1))             = IOC_STD_OUTPUT | IOC_PORT_GPIO;
    HWREG(IOC_BASE + IOC_O_IOCFG(IOID_LED_2))             = IOC_STD_OUTPUT | IOC_PORT_GPIO;
    HWREG(IOC_BASE + IOC_O_IOCFG(IOID_KEY_LEFT))          = ((IOC_STD_INPUT & ~IOC_NO_IOPULL) | IOC_IOPULL_UP) | IOC_PORT_GPIO;
    HWREG(IOC_BASE + IOC_O_IOCFG(IOID_KEY_RIGHT))         = ((IOC_STD_INPUT & ~IOC_NO_IOPULL) | IOC_IOPULL_UP) | IOC_PORT_GPIO;
    HWREG(IOC_BASE + IOC_O_IOCFG(IOID_KEY_UP))            = ((IOC_STD_INPUT & ~IOC_NO_IOPULL) | IOC_IOPULL_UP) | IOC_PORT_GPIO;
    HWREG(IOC_BASE + IOC_O_IOCFG(IOID_KEY_DOWN))          = ((IOC_STD_INPUT & ~IOC_NO_IOPULL) | IOC_IOPULL_UP) | IOC_PORT_GPIO;

    // In this example, we keep the AUX domain access permanently enabled
    scifOsalEnableAuxDomainAccess();

    // Initialize and start the Sensor Controller
    scifOsalRegisterCtrlReadyCallback(ctrlReadyCallback);
    scifOsalRegisterTaskAlertCallback(taskAlertCallback);
    scifInit(&scifDriverSetup);
    scifStartRtcTicksNow(0x00010000 / 128);
    IntMasterEnable();
    AONRTCEnable();

    // Main loop
    while (1) {

        // Start the LED blinker task?
        if (!(HWREG(GPIO_BASE + GPIO_O_DIN31_0) & BV(IOID_KEY_LEFT))) {
            if (scifStartTasksNbl(BV(SCIF_LED_BLINKER_TASK_ID)) == SCIF_SUCCESS) {
                scifWaitOnNbl(42);
            }

            // Debounce the button
            CPUdelay((100 * 1000 * 48) / 3);
            while (!(HWREG(GPIO_BASE + GPIO_O_DIN31_0) & BV(IOID_KEY_LEFT)));
            CPUdelay((100 * 1000 * 48) / 3);

        // Stop the LED blinker task?
        } else if (!(HWREG(GPIO_BASE + GPIO_O_DIN31_0) & BV(IOID_KEY_RIGHT))) {
            if (scifStopTasksNbl(BV(SCIF_LED_BLINKER_TASK_ID)) == SCIF_SUCCESS) {
                scifWaitOnNbl(42);
                scifResetTaskStructs(BV(SCIF_LED_BLINKER_TASK_ID), BV(SCIF_STRUCT_OUTPUT));
            }

            // Debounce the button
            CPUdelay((100 * 1000 * 48) / 3);
            while (!(HWREG(GPIO_BASE + GPIO_O_DIN31_0) & BV(IOID_KEY_RIGHT)));
            CPUdelay((100 * 1000 * 48) / 3);
        }
    }

} // main
