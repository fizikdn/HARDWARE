/**************************************************************************************************
  Filename:       flowGATTprofile.c
  Revised:        $Date: 2015-07-20 11:31:07 -0700 (Mon, 20 Jul 2015) $
  Revision:       $Revision: 44370 $

  Description:    This file contains the RS_Flow GATT profile sample GATT service
                  profile for use with the BLE sample application.

  Copyright 2010 - 2015 Texas Instruments Incorporated. All rights reserved.

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

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "flowGATTprofile.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        8
#define VALUELEN						  2

/*********************************************************************
 * Type of Characteristic value
 */

typedef uint8 ValueType;
//typedef uint16 ValueType;

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// RS_Flow GATT Profile Service UUID: 0xEE00
CONST uint8 rs_flowProfileServUUID[ATT_UUID_SIZE] =          // oritinally it is ATT_UUID_SIZE
{ 
		TI_BASE_UUID_128(RS_FlowPROFILE_SERV_UUID),
	//	LO_UINT16(RS_FlowPROFILE_SERV_UUID), HI_UINT16(RS_FlowPROFILE_SERV_UUID),
};

// Characteristic 1 UUID: 0xEE01
CONST uint8 rs_flowProfilechar1UUID[ATT_UUID_SIZE] =         // oritinally it is ATT_UUID_SIZE
{ 
		TI_BASE_UUID_128(RS_FlowPROFILE_CHAR1_UUID),
	//	LO_UINT16(RS_FlowPROFILE_CHAR1_UUID), HI_UINT16(RS_FlowPROFILE_CHAR1_UUID),
};

// Characteristic 2 UUID: 0xEE02
CONST uint8 rs_flowProfilechar2UUID[ATT_UUID_SIZE] =         // oritinally it is ATT_UUID_SIZE
{ 
		TI_BASE_UUID_128(RS_FlowPROFILE_CHAR2_UUID),
	//	LO_UINT16(RS_FlowPROFILE_CHAR2_UUID), HI_UINT16(RS_FlowPROFILE_CHAR2_UUID),
};



/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static rs_flowProfileCBs_t *rs_flowProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// RS_Flow Profile Service attribute
static CONST gattAttrType_t rs_flowProfileService = { ATT_UUID_SIZE, rs_flowProfileServUUID };   // oritinally it is ATT_UUID_SIZE


// RS_Flow Profile Characteristic 1 Properties
static uint8 rs_flowProfileChar1Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 1 Value
//static uint8 rs_flowProfileChar1 = 0;
static uint8 rs_flowProfileChar1[VALUELEN] = {0,0};     ///

// RS_Flow Profile Characteristic 1 User Description
static uint8 rs_flowProfileChar1UserDesp[17] = "Characteristic 1";


// RS_Flow Profile Characteristic 2 Properties
static uint8 rs_flowProfileChar2Props = GATT_PROP_NOTIFY;

// Characteristic 2 Value
//static uint8 rs_flowProfileChar2 = 0;
static uint8 rs_flowProfileChar2[VALUELEN] = {0,0};		///

// RS_Flow Profile Characteristic 2 Configuration.
static gattCharCfg_t *rs_flowProfileChar2Config;

// RS_Flow Profile Characteristic 2 User Description
static uint8 rs_flowProfileChar2UserDesp[17] = "Characteristic 2";


/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t rs_flowProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] =
{
  // RS_Flow Profile Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&rs_flowProfileService            /* pValue */
  },

    // Characteristic 1 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &rs_flowProfileChar1Props
    },

      // Characteristic Value 1
      { 
        { ATT_UUID_SIZE, rs_flowProfilechar1UUID },              // oritinally it is ATT_UUID_SIZE
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        rs_flowProfileChar1
      },

      // Characteristic 1 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        rs_flowProfileChar1UserDesp
      },      

    // Characteristic 2 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &rs_flowProfileChar2Props
    },

      // Characteristic Value 2
      { 
        { ATT_UUID_SIZE, rs_flowProfilechar2UUID },                 // oritinally it is ATT_UUID_SIZE
        0,
        0, 
        rs_flowProfileChar2
      },

	  // Characteristic 2 configuration
	    {
	      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
	      GATT_PERMIT_READ | GATT_PERMIT_WRITE,
	      0,
	      (uint8 *)&rs_flowProfileChar2Config
	    },


      // Characteristic 2 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        rs_flowProfileChar2UserDesp
      },           
      

};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t rs_flowProfile_ReadAttrCB(uint16_t connHandle,
                                          gattAttribute_t *pAttr, 
                                          uint8_t *pValue, uint16_t *pLen,
                                          uint16_t offset, uint16_t maxLen,
                                          uint8_t method);
static bStatus_t rs_flowProfile_WriteAttrCB(uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len,
                                           uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// RS_Flow Profile Service Callbacks
CONST gattServiceCBs_t rs_flowProfileCBs =
{
  rs_flowProfile_ReadAttrCB,  // Read callback function pointer
  rs_flowProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

bStatus_t utilExtractUuid16(gattAttribute_t *pAttr, uint16_t *pUuid)
{
  bStatus_t status = SUCCESS;

  if (pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID direct
    *pUuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
#ifdef GATT_TI_UUID_128_BIT
  }
  else if (pAttr->type.len == ATT_UUID_SIZE)
  {
    // 16-bit UUID extracted bytes 12 and 13
    *pUuid = BUILD_UINT16( pAttr->type.uuid[12], pAttr->type.uuid[13]);
#endif
  } else {
    *pUuid = 0xFFFF;
    status = FAILURE;
  }

  return status;
}


/*********************************************************************
 * @fn      RS_FlowProfile_AddService
 *
 * @brief   Initializes the RS_Flow Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t RS_FlowProfile_AddService( uint32 services )
{
  uint8 status;

  // Allocate Client Characteristic Configuration table
  rs_flowProfileChar2Config = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                            linkDBNumConns );
  if ( rs_flowProfileChar2Config == NULL )
  {     
    return ( bleMemAllocError );
  }
  
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, rs_flowProfileChar2Config );
  

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService(   rs_flowProfileAttrTbl,
                                          GATT_NUM_ATTRS( rs_flowProfileAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &rs_flowProfileCBs );

  return ( status );
}

/*********************************************************************
 * @fn      RS_FlowProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t RS_FlowProfile_RegisterAppCBs( rs_flowProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    rs_flowProfile_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      RS_FlowProfile_SetParameter
 *
 * @brief   Set a RS_Flow Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t RS_FlowProfile_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case RS_FlowPROFILE_CHAR1:
      if ( len == VALUELEN )            /// uint8
      {
       // rs_flowProfileChar1 = *((uint8*)value);        /// uint8

    	  memcpy( rs_flowProfileChar1, value, VALUELEN );

      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case RS_FlowPROFILE_CHAR2:
      if ( len == VALUELEN )            /// uint8
      {
      //  rs_flowProfileChar2 = *((uint8*)value);            ///uint8
    	  memcpy( rs_flowProfileChar2, value, VALUELEN );
        
        // See if Notification has been enabled
        //      GATTServApp_ProcessCharCfg( rs_flowProfileChar2Config, &rs_flowProfileChar2, FALSE,
    	  	  	GATTServApp_ProcessCharCfg( rs_flowProfileChar2Config, rs_flowProfileChar2, FALSE,
                                          rs_flowProfileAttrTbl, GATT_NUM_ATTRS( rs_flowProfileAttrTbl ),
                                          INVALID_TASK_ID, rs_flowProfile_ReadAttrCB );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;


    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      RS_FlowProfile_GetParameter
 *
 * @brief   Get a RS_Flow Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t RS_FlowProfile_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case RS_FlowPROFILE_CHAR1:
     // *((uint8*)value) = rs_flowProfileChar1;           /// uint8
      memcpy( value, rs_flowProfileChar1, VALUELEN);
      break;

    case RS_FlowPROFILE_CHAR2:
     // *((uint8*)value) = rs_flowProfileChar2;           /// uint8
      memcpy( value, rs_flowProfileChar2, VALUELEN);
      break;      

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          rs_flowProfile_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t rs_flowProfile_ReadAttrCB(uint16_t connHandle,
                                          gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t *pLen,
                                          uint16_t offset, uint16_t maxLen,
                                          uint8_t method)
{
  uint16 uuid;
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 

  if (utilExtractUuid16(pAttr,&uuid) == FAILURE) {
    // Invalid handle
    *pLen = 0;
    return ATT_ERR_INVALID_HANDLE;
  }


     switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      // characteristics 1 has read permissions
      // characteritisc 2 does not have read permissions, but because it
      //   can be sent as a notification, it is included here
      case RS_FlowPROFILE_CHAR1_UUID:
      case RS_FlowPROFILE_CHAR2_UUID:
       // *pLen = 1;
       // pValue[0] = *pAttr->pValue;
        *pLen = VALUELEN;
        VOID memcpy( pValue, pAttr->pValue, VALUELEN );
        break;

      default:
        // Should never get here!
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }


  return ( status );
}

/*********************************************************************
 * @fn      rs_flowProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t rs_flowProfile_WriteAttrCB(uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len,
                                           uint16_t offset, uint8_t method)
{
  uint16 uuid;
  bStatus_t status = SUCCESS;
  

  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  

  if (utilExtractUuid16(pAttr,&uuid) == FAILURE) {
    // Invalid handle
    return ATT_ERR_INVALID_HANDLE;
  }



    // 16-bit UUID
    // uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {

      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;
        
      default:
        // Should never get here!
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  

  return ( status );
}

/*********************************************************************
*********************************************************************/
