/**************************************************************************************************
  Filename:       RS_Flowmeter.c
  Revised:        $Date: 2015-07-13 11:43:11 -0700 (Mon, 13 Jul 2015) $
  Revision:       $Revision: 44336 $

  Description:    This file contains the RS_Flow BLE Peripheral sample application
                  for use with the CC2650 Bluetooth Low Energy Protocol Stack.

  Copyright 2013 - 2015 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "hci_tl.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "flowGATTprofile.h"
#include "scif_main_tirtos.h"

#if defined(SENSORTAG_HW)
#include "bsp_spi.h"
#endif // SENSORTAG_HW

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "ICallBleAPIMSG.h"

#include "util.h"
#include "board_lcd.h"
#include "board_key.h"
#include "Board.h"

#include "RS_Flowmeasure.h"

#include <ti/drivers/lcd/LCDDogm1286.h>

/*********************************************************************
 * CONSTANTS
 */
// Push Button
#define RSF_KEY_CHANGE_EVT                    0x0010

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          5280  // 3.3 seconds

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

#ifndef FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800
#else
// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8
#endif // FEATURE_OAD

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE /// TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#ifdef LCD_DISPLAY_ON
#define RSF_PERIODIC_EVT_PERIOD               1000
#else
#define RSF_PERIODIC_EVT_PERIOD               5000
#endif

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define RSF_TASK_PRIORITY                     1

#ifndef RSF_TASK_STACK_SIZE
#define RSF_TASK_STACK_SIZE                   644
#endif

// Internal Events for RTOS application
#define RSF_STATE_CHANGE_EVT                  0x0001
#define RSF_CHAR_CHANGE_EVT                   0x0002
#define RSF_PERIODIC_EVT                      0x0004
#define RSF_CONN_EVT_END_EVT                  0x0008

/*********************************************************************
 * TYPEDEFS
 */
/*********************************************************************
 * Type of Characteristic value
 */
#define VALUELEN   2
#define ValueLen   16

#if ValueLen == 8
#define UPPERLIMIT   255
typedef uint8_t ValueType;
#elif ValueLen == 16
#define UPPERLIMIT 65535
typedef uint16_t ValueType;
#endif

uint8_t ValueBackup[VALUELEN]={0XFF,0XFF};

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
} sbpEvt_t;


/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

#if defined(FEATURE_OAD)
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[RSF_TASK_STACK_SIZE];

// Profile state and parameters
//static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x10,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x52,   // 'R'
  0x53,   // 'S'
  0x20,   // ' '
  0x46,   // 'F'
  0x6C,   // 'l'
  0x6F,   // 'o'
  0x77,   // 'w'
  0x20,   // ' '
  0x4D,   // 'M'
  0x65,   // 'e'
  0x61,   // 'a'
  0x73,   // 's'
  0x75,   // 'u'
  0x72,   // 'r'
  0x65,   // 'e'

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
#ifdef FEATURE_OAD
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID)
#else
  LO_UINT16(RS_FlowPROFILE_SERV_UUID),
  HI_UINT16(RS_FlowPROFILE_SERV_UUID)
#endif //!FEATURE_OAD
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "RS Flow measurement";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void RS_FlowBLEPeripheral_init( void );
static void RS_FlowBLEPeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t RS_FlowBLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t RS_FlowBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void RS_FlowBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void RS_FlowBLEPeripheral_processStateChangeEvt(gaprole_States_t newState);
static void RS_FlowBLEPeripheral_processCharValueChangeEvt(uint8_t paramID);
static void RS_FlowBLEPeripheral_performPeriodicTask(void);

static void RS_FlowBLEPeripheral_sendAttRsp(void);
static void RS_FlowBLEPeripheral_freeAttRsp(uint8_t status);

static void RS_FlowBLEPeripheral_stateChangeCB(gaprole_States_t newState);
#ifndef FEATURE_OAD
static void RS_FlowBLEPeripheral_charValueChangeCB(uint8_t paramID);
#endif //!FEATURE_OAD
static void RS_FlowBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state);

#ifdef FEATURE_OAD
void RS_FlowBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData);
#endif //FEATURE_OAD

static void RS_FlowBLEPeripheral_clockHandler(UArg arg);


void RS_Flow_keyChangeHandler(uint8 keysPressed);
static void RS_Flow_handleKeys(uint8_t shift, uint8_t keys);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t RS_FlowBLEPeripheral_gapRoleCBs =
{
  RS_FlowBLEPeripheral_stateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t rs_flowBLEPeripheral_BondMgrCBs =
{
  NULL, // Passcode callback (not used by application)
  NULL  // Pairing / Bonding state Callback (not used by application)
};

// RS_Flow GATT Profile Callbacks
#ifndef FEATURE_OAD
static rs_flowProfileCBs_t RS_FlowBLEPeripheral_rs_flowProfileCBs =
{
  RS_FlowBLEPeripheral_charValueChangeCB // Characteristic value change callback
};
#endif //!FEATURE_OAD

#ifdef FEATURE_OAD
static oadTargetCBs_t rs_flowBLEPeripheral_oadCBs =
{
  RS_FlowBLEPeripheral_processOadWriteCB // Write Callback.
};
#endif //FEATURE_OAD

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      RS_FlowBLEPeripheral_createTask
 *
 * @brief   Task creation function for the RS_Flow BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void RS_FlowBLEPeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = RSF_TASK_STACK_SIZE;
  taskParams.priority = RSF_TASK_PRIORITY;

  Task_construct(&sbpTask, RS_FlowBLEPeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      RS_FlowBLEPeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void RS_FlowBLEPeripheral_init(void)
{

#ifdef NON_SCIF_GPIO
	// Initialize keys on SmartRF06EB
	Board_initKeys(RS_Flow_keyChangeHandler);
#endif

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Hard code the BD Address till CC2650 board gets its own IEEE address
  //uint8 bdAddress[B_ADDR_LEN] = { 0xAD, 0xD0, 0x0A, 0xAD, 0xD0, 0x0A };
  //HCI_EXT_SetBDADDRCmd(bdAddress);

  // Set device's Sleep Clock Accuracy
  //HCI_EXT_SetSCACmd(40);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, RS_FlowBLEPeripheral_clockHandler,
                      RSF_PERIODIC_EVT_PERIOD, 0, false, RSF_PERIODIC_EVT);


#ifdef LCD_DISPLAY_ON
  RS_Flow_SCIF_performPeriodicTask() ;
  Util_startClock(&periodicClock);
#endif

#ifndef SENSORTAG_HW
  Board_openLCD();
#endif //SENSORTAG_HW
  
#if SENSORTAG_HW
  // Setup SPI bus for serial flash and Devpack interface
  bspSpiOpen();
#endif //SENSORTAG_HW
  
  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = 0; // passkey "000000"
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service

#ifndef FEATURE_OAD
  RS_FlowProfile_AddService(GATT_ALL_SERVICES); // RS_Flow GATT Profile
#endif //!FEATURE_OAD

#ifdef FEATURE_OAD
  VOID OAD_addService();                 // OAD Profile
  OAD_register((oadTargetCBs_t *)&rs_flowBLEPeripheral_oadCBs);
  hOadQ = Util_constructQueue(&oadQ);
#endif

#ifdef IMAGE_INVALIDATE
  Reset_addService();
#endif //IMAGE_INVALIDATE
  
  
#ifndef FEATURE_OAD
  // Setup the RS_FlowProfile Characteristic Values
  {
	  uint8_t charValue1[VALUELEN] = {0,0};
	  uint8_t charValue2[VALUELEN] = {0,0};

    RS_FlowProfile_SetParameter(RS_FlowPROFILE_CHAR1, VALUELEN,
                               charValue1);
    RS_FlowProfile_SetParameter(RS_FlowPROFILE_CHAR2, VALUELEN,
                               charValue2);

  }

  // Register callback with flowGATTprofile
  RS_FlowProfile_RegisterAppCBs(&RS_FlowBLEPeripheral_rs_flowProfileCBs);
#endif //!FEATURE_OAD

  // Start the Device
  VOID GAPRole_StartDevice(&RS_FlowBLEPeripheral_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&rs_flowBLEPeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);
  
  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);
  

#ifdef LCD_DISPLAY_ON
#if defined FEATURE_OAD
#if defined (HAL_IMAGE_A)
  LCD_WRITE_STRING("BLE RS_Flow A", LCD_PAGE0);
#else
  LCD_WRITE_STRING("BLE RS_Flow B", LCD_PAGE0);
#endif // HAL_IMAGE_A
#else
  LCD_WRITE_STRING("BLE RS_Flow", LCD_PAGE0);
#endif // FEATURE_OAD
#endif // LCD_Display
}



/*********************************************************************
 * @fn      RS_Flow_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void RS_Flow_handleKeys(uint8_t shift, uint8_t keys)
{
  (void)shift;  // Intentionally unreferenced parameter

  //uint8_t rs_flow_count;
  uint8_t rs_flow_count[2];
  uint16_t flow_counter;


  RS_FlowProfile_GetParameter(RS_FlowPROFILE_CHAR1, rs_flow_count);
  flow_counter  = (uint16_t)rs_flow_count[0];
  flow_counter |= (((uint16_t)rs_flow_count[1]) << 8) ;
  if (keys & KEY_UP)
  {
    if (flow_counter < UPPERLIMIT)
    {
    	flow_counter++;

    }
  }

  if (keys & KEY_DOWN)
  {
    if (flow_counter > 0)
    {
    	flow_counter--;
    //	RS_FlowProfile_SetParameter(RS_FlowPROFILE_CHAR1, sizeof(uint8_t), &rs_flow_count);
    }
  }

	rs_flow_count[0] =  (flow_counter&0xFF);
	rs_flow_count[1] =	((flow_counter >> 8)&0xFF);
	RS_FlowProfile_SetParameter(RS_FlowPROFILE_CHAR1, VALUELEN, rs_flow_count);


#ifdef LCD_DISPLAY_ON
  LCD_WRITE_STRING_VALUE("rs_flow_count:", (uint16_t)flow_counter, 10, LCD_PAGE5);
#endif

  return;
}





/*********************************************************************
 * @fn      RS_FlowBLEPeripheral_taskFxn
 *
 * @brief   Application task entry point for the RS_Flow BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void RS_FlowBLEPeripheral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  RS_FlowBLEPeripheral_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;
        
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Event *pEvt = (ICall_Event *)pMsg;
          
          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & RSF_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              RS_FlowBLEPeripheral_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = RS_FlowBLEPeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          RS_FlowBLEPeripheral_processAppMsg(pMsg);


          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    }

    if (events & RSF_PERIODIC_EVT)
    {
      events &= ~RSF_PERIODIC_EVT;

      Util_startClock(&periodicClock);

      // Perform periodic application task
      RS_Flow_SCIF_performPeriodicTask() ;
      RS_FlowBLEPeripheral_performPeriodicTask();
    }

#ifdef FEATURE_OAD
    while (!Queue_empty(hOadQ))
    {
      oadTargetWrite_t *oadWriteEvt = Queue_dequeue(hOadQ);

      // Identify new image.
      if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ)
      {
        OAD_imgIdentifyWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }
      // Write a next block request.
      else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ)
      {
        OAD_imgBlockWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }

      // Free buffer.
      ICall_free(oadWriteEvt);
    }
#endif //FEATURE_OAD
  }
}

/*********************************************************************
 * @fn      RS_FlowBLEPeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t RS_FlowBLEPeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;
    
  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = RS_FlowBLEPeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            break;
            
          default:
            break;
        }
      }
      break;
      
    default:
      // do nothing
      break;
  }
  
  return (safeToDealloc);
}

/*********************************************************************
 * @fn      RS_FlowBLEPeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t RS_FlowBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   RSF_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      RS_FlowBLEPeripheral_freeAttRsp(FAILURE);
      
      // Hold on to the response message for retransmission
      pAttRsp = pMsg;
      
      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.
    
    // Display the opcode of the message that caused the violation.
    LCD_WRITE_STRING_VALUE("FC Violated:", pMsg->msg.flowCtrlEvt.opcode,
                           10, LCD_PAGE5);
  }    
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    LCD_WRITE_STRING_VALUE("MTU Size:", pMsg->msg.mtuEvt.MTU, 10, LCD_PAGE5);
  }
  
  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
  
  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      RS_FlowBLEPeripheral_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void RS_FlowBLEPeripheral_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;
    
    // Increment retransmission count
    rspTxRetry++;
    
    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);
      
      // We're done with the response message
      RS_FlowBLEPeripheral_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      LCD_WRITE_STRING_VALUE("Rsp send retry:", rspTxRetry, 10, LCD_PAGE5);
    }
  }
}

/*********************************************************************
 * @fn      RS_FlowBLEPeripheral_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void RS_FlowBLEPeripheral_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      LCD_WRITE_STRING_VALUE("Rsp sent, retry:", rspTxRetry, 10, LCD_PAGE5);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
      
      LCD_WRITE_STRING_VALUE("Rsp retry failed:", rspTxRetry, 10, LCD_PAGE5);
    }
    
    // Free response message
    ICall_freeMsg(pAttRsp);
    
    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      RS_FlowBLEPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void RS_FlowBLEPeripheral_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case RSF_STATE_CHANGE_EVT:
      RS_FlowBLEPeripheral_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;

    case RSF_CHAR_CHANGE_EVT:
      RS_FlowBLEPeripheral_processCharValueChangeEvt(pMsg->hdr.state);
      break;

    case RSF_KEY_CHANGE_EVT:
      RS_Flow_handleKeys(0, pMsg->hdr.state);
      break;

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      RS_FlowBLEPeripheral_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void RS_FlowBLEPeripheral_stateChangeCB(gaprole_States_t newState)
{
  RS_FlowBLEPeripheral_enqueueMsg(RSF_STATE_CHANGE_EVT, newState);
}

/*********************************************************************
 * @fn      RS_FlowBLEPeripheral_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void RS_FlowBLEPeripheral_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

#ifdef LCD_DISPLAY_ON
        // Display device address
        LCD_WRITE_STRING(Util_convertBdAddr2Str(ownAddress), LCD_PAGE1);
        LCD_WRITE_STRING("Initialized", LCD_PAGE2);
#endif
      }
      break;

    case GAPROLE_ADVERTISING:
#ifdef LCD_DISPLAY_ON
      LCD_WRITE_STRING("Advertising", LCD_PAGE2);
#endif
      break;

#ifdef PLUS_BROADCASTER   
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of 
     * state to the application.  These are then disabled here so that sending 
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8_t advertEnabled = FALSE;
      
        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);
      
        advertEnabled = TRUE;
      
        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);
        
        // Reset flag for next connection.
        firstConnFlag = false;
        
        RS_FlowBLEPeripheral_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER   

    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

        Util_startClock(&periodicClock);

#ifdef LCD_DISPLAY_ON
        LCD_WRITE_STRING("Connected", LCD_PAGE2);
        LCD_WRITE_STRING(Util_convertBdAddr2Str(peerAddress), LCD_PAGE3);
#endif

        #ifdef PLUS_BROADCASTER
          // Only turn advertising on for this state when we first connect
          // otherwise, when we go from connected_advertising back to this state
          // we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            
            // Set to true for non-connectabel advertising.
            advertEnabled = TRUE;

            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      LCD_WRITE_STRING("Connected Advertising", LCD_PAGE2);
      break;

    case GAPROLE_WAITING:
     ///  Util_stopClock(&periodicClock);
      RS_FlowBLEPeripheral_freeAttRsp(bleNotConnected);

#ifdef LCD_DISPLAY_ON
      LCD_WRITE_STRING("Disconnected", LCD_PAGE2);

      // Clear remaining lines
      LCD_WRITE_STRING("", LCD_PAGE3);
      LCD_WRITE_STRING("", LCD_PAGE4);
 ///  LCD_WRITE_STRING("", LCD_PAGE5);
#endif
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      RS_FlowBLEPeripheral_freeAttRsp(bleNotConnected);
      
      LCD_WRITE_STRING("Timed Out", LCD_PAGE2);
      
      // Clear remaining lines
      LCD_WRITE_STRING("", LCD_PAGE3);
      LCD_WRITE_STRING("", LCD_PAGE4);
      LCD_WRITE_STRING("", LCD_PAGE5);

      #ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        firstConnFlag = false;
      #endif //#ifdef (PLUS_BROADCASTER)
      break;

    case GAPROLE_ERROR:
      LCD_WRITE_STRING("Error", LCD_PAGE2);
      break;

    default:
      LCD_WRITE_STRING("", LCD_PAGE2);
      break;
  }

  // Update the state
  //gapProfileState = newState;
}

#ifndef FEATURE_OAD
/*********************************************************************
 * @fn      RS_FlowBLEPeripheral_charValueChangeCB
 *
 * @brief   Callback from RS_Flow Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void RS_FlowBLEPeripheral_charValueChangeCB(uint8_t paramID)
{
  RS_FlowBLEPeripheral_enqueueMsg(RSF_CHAR_CHANGE_EVT, paramID);
}
#endif //!FEATURE_OAD

/*********************************************************************
 * @fn      RS_FlowBLEPeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending RS_Flow Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void RS_FlowBLEPeripheral_processCharValueChangeEvt(uint8_t paramID)
{
#ifndef FEATURE_OAD
/*
	uint8_t newValue;

  switch(paramID)
  {
    case SIMPLEPROFILE_CHAR1:
      RS_FlowProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);

      LCD_WRITE_STRING_VALUE("Char 1:", (uint16_t)newValue, 10, LCD_PAGE4);
      break;

    case SIMPLEPROFILE_CHAR3:
      RS_FlowProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue);

      LCD_WRITE_STRING_VALUE("Char 3:", (uint16_t)newValue, 10, LCD_PAGE4);
      break;

    default:
      // should not reach here!
      break;
  }
*/
#endif //!FEATURE_OAD
}

/*********************************************************************
 * @fn      RS_FlowBLEPeripheral_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (RSF_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the RS_FlowGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void RS_FlowBLEPeripheral_performPeriodicTask(void)
{
#ifndef FEATURE_OAD
  uint8_t valueToCopy[2];

  // Call to retrieve the value of the third characteristic in the profile
  if (RS_FlowProfile_GetParameter(RS_FlowPROFILE_CHAR1, valueToCopy) == SUCCESS)
  {
    // Call to set that value of the 2nd characteristic in the profile.
    // Note that if notifications of the 2nd characteristic have been
    // enabled by a GATT client device, then a notification will be sent
    // every time this function is called.

//	  if((valueToCopy[0] != ValueBackup[0])|| (valueToCopy[1] != ValueBackup[1]))
//	  {
		  RS_FlowProfile_SetParameter(RS_FlowPROFILE_CHAR2, VALUELEN, valueToCopy);
//	      memcpy( ValueBackup, valueToCopy, VALUELEN );
//	  }
  }
#endif //!FEATURE_OAD
}


#if defined(FEATURE_OAD)
/*********************************************************************
 * @fn      RS_FlowBLEPeripheral_processOadWriteCB
 *
 * @brief   Process a write request to the OAD profile.
 *
 * @param   event      - event type:
 *                       OAD_WRITE_IDENTIFY_REQ
 *                       OAD_WRITE_BLOCK_REQ
 * @param   connHandle - the connection Handle this request is from.
 * @param   pData      - pointer to data for processing and/or storing.
 *
 * @return  None.
 */
void RS_FlowBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
                                             sizeof(uint8_t) * OAD_PACKET_SIZE);
  
  if ( oadWriteEvt != NULL )
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;
    
    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);

    Queue_enqueue(hOadQ, (Queue_Elem *)oadWriteEvt);
    
    // Post the application's semaphore.
    Semaphore_post(sem);
  }
  else
  {
    // Fail silently.
  }
}
#endif //FEATURE_OAD

/*********************************************************************
 * @fn      RS_FlowBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void RS_FlowBLEPeripheral_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      RS_FlowBLEPeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static void RS_FlowBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state)
{
  sbpEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}



/*********************************************************************
 * @fn      RS_Flow_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void RS_Flow_keyChangeHandler(uint8 keysPressed)
{
	RS_FlowBLEPeripheral_enqueueMsg(RSF_KEY_CHANGE_EVT, keysPressed);
}


/*********************************************************************
*********************************************************************/
