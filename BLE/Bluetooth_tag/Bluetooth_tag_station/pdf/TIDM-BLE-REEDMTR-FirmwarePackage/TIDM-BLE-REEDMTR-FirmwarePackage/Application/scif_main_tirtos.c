//*****************************************************************************
//  SENSOR CONTROLLER STUDIO EXAMPLE: Reed Switch Rotation Detection
//  Operating system: TI-RTOS
//
//  The Sensor Controller is used to sample the SmartRF06EB light sensor using
//  the ADC. The ADC value range is divided into 5 configurable bins. When the
//  ADC value changes to a new bin, the new bin value is reported to the
//  application through scTaskAlertCallback(). The application then enables 0
//  to 4 LEDs to indicate the bin value.
//
//  The DISABLE_LOW_POWER definition can be set to 1 to allow debugging.
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
#include "scif.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>
#include <Board.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#include <ti/drivers/pin/PINCC26XX.h>

#include <ti/drivers/lcd/LCDDogm1286.h>
#include "gatt.h"
#include "flowGATTprofile.h"
#include "board_lcd.h"
#include "scif_main_tirtos.h"



#define VALUELEN 2
//#define RS_FlowPROFILE_CHAR1                   0  // RW uint8 - Profile Characteristic 1 value

#define BV(n)               (1 << (n))


#define DISABLE_LOW_POWER   0


// Display error message if the SCIF driver has been generated with incorrect operating system setting
#ifndef SCIF_OSAL_TIRTOS_H
    #error "Generated SCIF driver supports incorrect operating system. Please change to 'TI-RTOS' in the Sensor Controller Studio project panel and re-generate the driver."
#endif

// Task data
Task_Struct myTask;
Char myTaskStack[512];


// Semaphore used to wait for Sensor Controller task ALERT event
static Semaphore_Struct semScTaskAlert;




void scCtrlReadyCallback(void) {

} // scCtrlReadyCallback




void scTaskAlertCallback(void) {

    // Wake up the OS task
    Semaphore_post(Semaphore_handle(&semScTaskAlert));

} // scTaskAlertCallback




PIN_Config pLedPinTable[] = {
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED3 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED4 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
PIN_State ledPinState;


void taskFxn(UArg a0, UArg a1) {


    // Initialize the Sensor Controller
    scifOsalInit();
    scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
    scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
    scifInit(&scifDriverSetup);
    scifStartRtcTicksNow(0x00010000 / 8);
    IntMasterEnable();

    if (scifStartTasksNbl(BV(SCIF_REED_SWITCH_TASK_ID)) == SCIF_SUCCESS) {
                   scifWaitOnNbl(42);
               }

// The following while loop is not used and never be called again
// Main loop
    while (1) {

        // Wait for an ALERT callback
        Semaphore_pend(Semaphore_handle(&semScTaskAlert), BIOS_WAIT_FOREVER);


        // Clear the ALERT interrupt source
        scifClearAlertIntSource();


        if (scifGetTaskIoStructAvailCount(SCIF_REED_SWITCH_TASK_ID, SCIF_STRUCT_OUTPUT) == 1) {
        	SCIF_REED_SWITCH_OUTPUT_T* pOutput = scifGetTaskStruct(SCIF_REED_SWITCH_TASK_ID, SCIF_STRUCT_OUTPUT);

        	//   add code here
        	//   to process the data

        	 scifHandoffTaskStruct(SCIF_REED_SWITCH_TASK_ID, SCIF_STRUCT_OUTPUT);

        }

        // Acknowledge the alert event
        scifAckAlertEvents();

    }

} // taskFxn



void RS_Flow_SCIF_performPeriodicTask(void)
{
    uint8_t rs_flow_count[2];
    uint16_t flow_counter[3];

	flow_counter[0] = scifTaskData.reedSwitch.output.effCounter ;
	flow_counter[1] = scifTaskData.reedSwitch.output.upCounter ;
	flow_counter[2] = scifTaskData.reedSwitch.output.downCounter ;

	rs_flow_count[0] =  (uint8_t)(flow_counter[0]&0xFF);
	rs_flow_count[1] =	(uint8_t)((flow_counter[0] >> 8)&0xFF);
	RS_FlowProfile_SetParameter(RS_FlowPROFILE_CHAR1, VALUELEN, rs_flow_count);

#ifdef LCD_DISPLAY_ON
	LCD_WRITE_STRING_VALUE("rs_flow_count:", flow_counter[0], 10, LCD_PAGE5);
	LCD_WRITE_STRING_VALUE("rs__up__count:", flow_counter[1], 10, LCD_PAGE6);
	LCD_WRITE_STRING_VALUE("rs_down_count:", flow_counter[2], 10, LCD_PAGE7);
#endif


}



int scif_main(void) {
    Task_Params taskParams;

    // Optional: Prevent the system from entering standby to allow debugging
#if DISABLE_LOW_POWER
    Power_setConstraint(Power_SB_DISALLOW);
    Power_setConstraint(Power_IDLE_PD_DISALLOW);
#endif

    // Initialize the PIN driver
    // PIN_init(BoardGpioInitTable);

    // Configure the OS task
    Task_Params_init(&taskParams);
    taskParams.stack = myTaskStack;
    taskParams.stackSize = sizeof(myTaskStack);
    taskParams.priority = 3;  //3
    Task_construct(&myTask, taskFxn, &taskParams, NULL);

    // Create the semaphore used to wait for Sensor Controller ALERT events
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&semScTaskAlert, 0, &semParams);

    // Start TI-RTOS
//    BIOS_start();
    return 0;

} // main
