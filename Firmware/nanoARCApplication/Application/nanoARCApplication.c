/**************************************************************************************************
  Filename:       nanoARCApplication.c
  Revised:        $Date: 2015-01-28 17:22:05 -0800 (Wed, 28 Jan 2015) $
  Revision:       $Revision: 42106 $

  Description:    This file contains the nanoARC application
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
#include <xdc/std.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include <ICall.h>

#include "gatt.h"
#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "controlservice.h"

#ifdef FEATURE_OAD
#include "oad_target.h"
#include "oad.h"
#endif

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "ICallBleAPIMSG.h"

#include "util.h"
#include "board_key.h"
#include "Board.h"

#include "nanoARCApplication.h"
/*********************************************************************
 * CONSTANTS
 */
// Advertising interval when device is discoverable (units of 625us, 800=500ms)
#define DEFAULT_ADVERTISING_INTERVAL          800

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 18=22.5ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     18

// Maximum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     80

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               5000

// Task configuration
#define SBP_TASK_PRIORITY                     1

//Location in flash memory containing device MAC address
#define MAC_ADDR_MEMORY_LOCATION 0x500012E8

//Device modes
#define DEVICE_MODE_TRANSMITTER 0x01
#define DEVICE_MODE_RECEIVER 0x02

//Hard coded device mode based on lowest MAC address byte
#define RECEIVER_ADDR 0x02 //First assembled Spintop 5
#define TRANSMITTER_ADDR 0x05 //First assembled nanoARCv1

#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   644
#endif

// Internal Events for RTOS application
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_PERIODIC_EVT                      0x0004
#ifdef FEATURE_OAD
#define SBP_OAD_WRITE_EVT                     0x0008
#endif //FEATURE_OAD

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  uint8_t event;  // Which profile's event
  uint8_t status; // New status
} sbpEvt_t;

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


// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

// Profile state and parameters
//static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x08,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x6e,   // 'n'
  0x61,   // 'a'
  0x6e,   // 'n'
  0x6f,   // 'o'
  0x41,   // 'A'
  0x52,   // 'R'
  0x43,   // 'C'

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 22.5ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 100ms
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
  LO_UINT16(CONTROLSERVICE_SERV_UUID),
  HI_UINT16(CONTROLSERVICE_SERV_UUID)
#endif //!FEATURE_OAD
};

// GAP GATT Attributes
uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "nanoARC"; //Used in init

#if FEATURE_OAD
// Event data from OAD profile.
static oadTargetWrite_t oadWriteEventData;
#endif //FEATURE_OAD

//Device mode, whether device is a transmitter or receiver
static uint8_t deviceMode = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void nanoARCApplication_init( void );
static void nanoARCApplication_taskFxn(UArg a0, UArg a1);

static void nanoARCApplication_processStackMsg(ICall_Hdr *pMsg);
static void nanoARCApplication_processGATTMsg(gattMsgEvent_t *pMsg);
static void nanoARCApplication_processAppMsg(sbpEvt_t *pMsg);
static void nanoARCApplication_processStateChangeEvt(gaprole_States_t newState);
static void nanoARCApplication_processCharValueChangeEvt(uint8_t paramID);
static void nanoARCApplication_performPeriodicTask(void);

static void nanoARCApplication_stateChangeCB(gaprole_States_t newState);
#ifndef FEATURE_OAD
static void nanoARCApplication_charValueChangeCB(uint8_t paramID);
#endif //!FEATURE_OAD
static void nanoARCApplication_enqueueMsg(uint8_t event, uint8_t status);

#ifdef FEATURE_OAD
void nanoARCApplication_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData);
#endif //FEATURE_OAD

static void nanoARCApplication_clockHandler(UArg arg);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t nanoARCApplication_gapRoleCBs =
{
  nanoARCApplication_stateChangeCB,    // Profile State Change Callbacks
  NULL                                  // When a valid RSSI is read from
                                        // controller (not used by app)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t nanoARCApplication_BondMgrCBs =
{
  NULL, // Passcode callback (not used by application)
  NULL  // Pairing / Bonding state Callback (not used by application)
};

// Control Service Callbacks
#ifndef FEATURE_OAD
static controlServiceCBs_t nanoARCApplication_controlServiceCBs =
{
  nanoARCApplication_charValueChangeCB // Characteristic value change callback
};
#endif //!FEATURE_OAD

#ifdef FEATURE_OAD
static oadTargetCBs_t nanoARCApplication_oadCBs =
{
  NULL,                                 // Read Callback.  Optional.
  nanoARCApplication_processOadWriteCB // Write Callback.  Mandatory.
};
#endif //FEATURE_OAD

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      nanoARCApplication_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void nanoARCApplication_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;

  Task_construct(&sbpTask, nanoARCApplication_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      nanoARCApplication_init
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
static void nanoARCApplication_init(void)
{
	// ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Hard code the BD Address till CC2650 board gets its own IEEE address
 // uint8 bdAddress[B_ADDR_LEN] = { 0xF0, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA };
 // HCI_EXT_SetBDADDRCmd(bdAddress);

  // Set device's Sleep Clock Accuracy
  //HCI_EXT_SetSCACmd(40);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, nanoARCApplication_clockHandler,
                      SBP_PERIODIC_EVT_PERIOD, 0, false, SBP_PERIODIC_EVT);

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
    uint8_t pairMode = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
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
  controlService_AddService(GATT_ALL_SERVICES); // Control Service
#endif //!FEATURE_OAD

#ifdef FEATURE_OAD
  VOID OAD_addService();                 // OAD Profile
  OAD_register((oadTargetCBs_t *)&nanoARCApplication_oadCBs);
#endif

#ifndef FEATURE_OAD
  // Setup the Control Service Characteristic Values
  {
    uint8_t charValue1[CONTROLSERVICE_CHAR1_LEN] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };

    controlService_SetParameter(CONTROLSERVICE_CHAR1, CONTROLSERVICE_CHAR1_LEN,
                               charValue1);
  }

  // Register callback with Control Service
  controlService_RegisterAppCBs(&nanoARCApplication_controlServiceCBs);
#endif //!FEATURE_OAD

  // Start the Device
  VOID GAPRole_StartDevice(&nanoARCApplication_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&nanoARCApplication_BondMgrCBs);

#if defined FEATURE_OAD
#if defined (HAL_IMAGE_A)

#else

#endif // HAL_IMAGE_A
#else

#endif // FEATURE_OAD
}

/*********************************************************************
 * @fn      nanoARCApplication_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void nanoARCApplication_taskFxn(UArg a0, UArg a1)
{
	//Check whether peripheral or central device
    //Read MAC address direct from memory
    uint8_t ownAddress[B_ADDR_LEN];
    VOID memcpy(ownAddress, (void*)MAC_ADDR_MEMORY_LOCATION, B_ADDR_LEN);

    switch (ownAddress[B_ADDR_LEN - 1]){
    	case RECEIVER_ADDR:
    		deviceMode = DEVICE_MODE_RECEIVER;
    		strcpy((char*)attDeviceName, "nanoARC RX-02"); //21 chars max
    		break;
    	case TRANSMITTER_ADDR:
    		deviceMode = DEVICE_MODE_TRANSMITTER;
    		strcpy((char*)attDeviceName, "nanoARC TX-05"); //21 chars max
    		break;
    	default:
    		deviceMode = DEVICE_MODE_RECEIVER;
    		strcpy((char*)attDeviceName, "nanoARC RX-default"); //21 chars max
    		break;
    }

    // Initialize application
    if (deviceMode == DEVICE_MODE_TRANSMITTER){
    	  //nanoARCApplication_initCentral();
    }
    else{
    	  //nanoARCApplication_initPeripheral();
    }

  // Initialize application
  nanoARCApplication_init();

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
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          nanoARCApplication_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (!Queue_empty(appMsgQueue))
      {
        sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          nanoARCApplication_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    }

    if (events & SBP_PERIODIC_EVT)
    {
      events &= ~SBP_PERIODIC_EVT;

      Util_startClock(&periodicClock);

      // Perform periodic application task
      nanoARCApplication_performPeriodicTask();
    }

#ifdef FEATURE_OAD
    if (events & SBP_OAD_WRITE_EVT)
    {
      events &= ~SBP_OAD_WRITE_EVT;

      // Identify new image.
      if (oadWriteEventData.event == OAD_WRITE_IDENTIFY_REQ)
      {
        OAD_imgIdentifyWrite(oadWriteEventData.connHandle,
                                   oadWriteEventData.pData);
      }
      // Write a next block request.
      else if (oadWriteEventData.event == OAD_WRITE_BLOCK_REQ)
      {
        OAD_imgBlockWrite(oadWriteEventData.connHandle, oadWriteEventData.pData);
      }

      // Free buffer.
      ICall_free(oadWriteEventData.pData);
    }
#endif //FEATURE_OAD
  }
}

/*********************************************************************
 * @fn      nanoARCApplication_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void nanoARCApplication_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      nanoARCApplication_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    default:
      // do nothing
      break;
  }
}

/*********************************************************************
 * @fn      nanoARCApplication_processGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  None.
 */
static void nanoARCApplication_processGATTMsg(gattMsgEvent_t *pMsg)
{
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      nanoARCApplication_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void nanoARCApplication_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->event)
  {
    case SBP_STATE_CHANGE_EVT:
      nanoARCApplication_processStateChangeEvt((gaprole_States_t)pMsg->status);
      break;

    case SBP_CHAR_CHANGE_EVT:
      nanoARCApplication_processCharValueChangeEvt(pMsg->status);
      break;

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      nanoARCApplication_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void nanoARCApplication_stateChangeCB(gaprole_States_t newState)
{
  nanoARCApplication_enqueueMsg(SBP_STATE_CHANGE_EVT, newState);
}

/*********************************************************************
 * @fn      nanoARCApplication_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void nanoARCApplication_processStateChangeEvt(gaprole_States_t newState)
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

        // Display device address
        char* address = Util_convertBdAddr2Str(ownAddress);
        System_printf("Initialized: %s\n", address);
        System_flush();
      }
      break;

    case GAPROLE_ADVERTISING:
      System_printf("Advertising\n");
      System_flush();
      break;

    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

        Util_startClock(&periodicClock);

        char* address = Util_convertBdAddr2Str(peerAddress);
        System_printf("Connected: %s\n", address);
        System_flush();

        #ifdef PLUS_BROADCASTER
          // Only turn advertising on for this state when we first connect
          // otherwise, when we go from connected_advertising back to this state
          // we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = TRUE; // Turn on Advertising

            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      System_printf("Connected Advertising\n");
      System_flush();
      break;

    case GAPROLE_WAITING:
      Util_stopClock(&periodicClock);

      System_printf("Disconnected\n");
      System_flush();
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
        System_printf("Timed Out\n");
        System_flush();

      #ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        firstConnFlag = false;
      #endif //#ifdef (PLUS_BROADCASTER)
      break;

    case GAPROLE_ERROR:
      System_printf("Error\n");
      System_flush();
      break;

    default:
      System_printf("Default\n");
      System_flush();
      break;
  }

  // Update the state
  //gapProfileState = newState;
}

#ifndef FEATURE_OAD
/*********************************************************************
 * @fn      nanoARCApplication_charValueChangeCB
 *
 * @brief   Callback from Control Service indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void nanoARCApplication_charValueChangeCB(uint8_t paramID)
{
  nanoARCApplication_enqueueMsg(SBP_CHAR_CHANGE_EVT, paramID);
}
#endif //!FEATURE_OAD

/*********************************************************************
 * @fn      nanoARCApplication_processCharValueChangeEvt
 *
 * @brief   Process a pending Control Service characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void nanoARCApplication_processCharValueChangeEvt(uint8_t paramID)
{
#ifndef FEATURE_OAD
  uint8_t newValue[CONTROLSERVICE_CHAR1_LEN];

  switch(paramID)
  {
    case CONTROLSERVICE_CHAR1:
      controlService_GetParameter(CONTROLSERVICE_CHAR1, &newValue);

      int16_t channel1 = newValue[0] << 8;
      channel1 |= newValue[1];

      int16_t channel2 = newValue[2] << 8;
      channel2 |= newValue[3];

      System_printf("Char 1 - X: %d, Y: %d\n", (int16_t)channel1, (int16_t)channel2);
      System_flush();
      break;

    default:
      // should not reach here!
      break;
  }
#endif //!FEATURE_OAD
}

/*********************************************************************
 * @fn      nanoARCApplication_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SBP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void nanoARCApplication_performPeriodicTask(void)
{
#ifndef FEATURE_OAD
	/*
  uint8_t valueToCopy;

  // Call to retrieve the value of the third characteristic in the profile
  if (SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &valueToCopy) == SUCCESS)
  {
    // Call to set that value of the fourth characteristic in the profile.
    // Note that if notifications of the fourth characteristic have been
    // enabled by a GATT client device, then a notification will be sent
    // every time this function is called.
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                               &valueToCopy);
  }
*/
#endif //!FEATURE_OAD
}


#if FEATURE_OAD
/*********************************************************************
 * @fn      nanoARCApplication_processOadWriteCB
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
void nanoARCApplication_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  uint8_t numOfBytes = (event == OAD_WRITE_IDENTIFY_REQ) ? 8 : 18;

  // Store OAD Identify Request dynamically.
  if (oadWriteEventData.pData = ICall_malloc(sizeof(uint8_t) * numOfBytes))
  {
    // Set the event.
    events |= SBP_OAD_WRITE_EVT;

    oadWriteEventData.event = event;
    oadWriteEventData.connHandle = connHandle;

    memcpy(oadWriteEventData.pData, pData, numOfBytes);

    // Post the application's semaphore.
    Semaphore_post(sem);
  }
}
#endif //FEATURE_OAD

/*********************************************************************
 * @fn      nanoARCApplication_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void nanoARCApplication_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      nanoARCApplication_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event  - message event.
 * @param   status - message status.
 *
 * @return  None.
 */
static void nanoARCApplication_enqueueMsg(uint8_t event, uint8_t status)
{
  sbpEvt_t *pMsg;

  // Create dynamic pointer to message.
  if (pMsg = ICall_malloc(sizeof(sbpEvt_t)))
  {
    pMsg->event = event;
    pMsg->status = status;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}

/*********************************************************************
*********************************************************************/
