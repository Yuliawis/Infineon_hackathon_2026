/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include "ble_nus_subsys.h"
#include "cm0_ipc.h"
#include "cy_ble.h"
#include "cycfg_ble.h"
#include "cy_sysint.h"
#include "cm0_cm4_shared_cmd.h"
#include "cy_ipc_pipe.h"
#include "debug_uart.h"

// ----------------------- Variables -----------------------
// Connection handle. We will use only one connection for simplicity.
static cy_stc_ble_conn_handle_t appConnHandle;
// Track Notification subscription status. BLE stack can do it for you, yet access is not trivial.
static bool isSubscribedToNtf = false;
// Notification structure which will be sent to subscribed Central.
// Example
/*
notifyValPair.attrHandle = CY_BLE_NUS_UART_NUS_TX_CHAR_HANDLE;  // Be sure to select correct handle! Find those in BLE_config.h
notifyValPair.value.len = writeReqParam->handleValPair.value.len;
notifyValPair.value.val = writeReqParam->handleValPair.value.val;

Cy_BLE_GATTS_SendNotification(&appConnHandle, &notifyValPair);  // Need to check status from return! Dropped it for simplicity.
*/
static cy_stc_ble_gatt_handle_value_pair_t notifyValPair;

ipc_msg_t ipcMsgForCM4 = {/* IPC structure to be sent to CM4 */
                          .clientId = IPC_CM0_TO_CM4_CLIENT_ID,
                          .userCode = 0,
                          .intrMask = CY_SYS_CYPIPE_INTR_MASK,
                          .buffer = {},
                          .len = 0};

uint16_t mtu_val = 0;
const char* carsay = "Wroom!";
// ----------------------- Variables END -----------------------

static void processIncomingIPCMessage(ipc_msg_t* msg);
static void processCM0Command(ipc_msg_t* msg);

// NvicMux0/1 are reserved (ROM calls, IPC pipe); mux must be < 8 to wake from Deep Sleep.
static const cy_stc_sysint_t blessIsrCfg = {
        .intrSrc = NvicMux3_IRQn, .cm0pSrc = bless_interrupt_IRQn, .intrPriority = 1u};

static void BlessInterrupt(void) {
    Cy_BLE_BlessIsrHandler();
}

// TODO comment
cy_en_ble_api_result_t sendDataNotificationNUS(uint8_t* data, uint16_t len) {
    cy_en_ble_api_result_t err;

    // Notification payload is MTU minus 3 bytes of ATT header (opcode + handle)
    if ((len + 3u) > mtu_val) {
        // Length to send is bigger than possible MTU
        err = CY_BLE_ERROR_MEMORY_ALLOCATION_FAILED;
    } else {
        notifyValPair.attrHandle = CY_BLE_NUS_UART_NUS_TX_CHAR_HANDLE;
        notifyValPair.value.actualLen = len;
        notifyValPair.value.len = len;
        notifyValPair.value.val = data;  // Please be aware, it is only a pointer! Data is not allocated for you

        err = Cy_BLE_GATTS_SendNotification(&appConnHandle, &notifyValPair);
    }

    DBG_PRINTF("sendDataNotificationNUS %X\r\n", err);

    return err;
}

static void parseIncomingBLECommand(uint8_t* val, uint16 len) {
    enum blePayloadTypes payloadType = val[0];

    switch (payloadType) {
        case BLE_NUS_PAYLOAD_CM0_CMD: {
            // Not implemented
            // For example, here you can configure CM0P core without routing messages to CM4
            DBG_PRINTF("CM0 CMD\r\n");
            break;
        }
        case BLE_NUS_PAYLOAD_CM4_CMD: {
            // Command will be routed to CM4
            DBG_PRINTF("CM4 CMD\r\n");
            // Only touch the IPC buffer when the previous message has been released by CM4
            if (CM0_IsCM4Ready()) {
                ipcMsgForCM4.userCode = IPC_USR_CODE_CMD;
                // We don't send the first byte because it is intended only for CM0P core!
                ipcMsgForCM4.len = len - 1;
                memcpy(&ipcMsgForCM4.buffer[0], &val[1], ipcMsgForCM4.len);
                if (!CM0_SendCM4Message(&ipcMsgForCM4)) {
                    DBG_PRINTF("IPC send to CM4 failed, command dropped\r\n");
                }
            } else {
                DBG_PRINTF("CM4 busy, command dropped\r\n");
            }
            break;
        }
        default: break;
    }
}

// Generic callback for all possible BLE events.
// Please note, for standardized services, you could go with specialized functions.
// For custom services, such approach is used.
static void AppCallBack(uint32_t event, void* eventParam) {
    cy_en_ble_api_result_t apiResult;
    // Local variable to store Write Request data from central.
    cy_stc_ble_gatts_write_cmd_req_param_t* writeReqParam;
    uint32_t i;

    // Unused unless you want privacy.
    static cy_stc_ble_gap_sec_key_info_t keyInfo = {
            .localKeysFlag = CY_BLE_GAP_SMP_INIT_ENC_KEY_DIST | CY_BLE_GAP_SMP_INIT_IRK_KEY_DIST |
                             CY_BLE_GAP_SMP_INIT_CSRK_KEY_DIST,
            .exchangeKeysFlag = CY_BLE_GAP_SMP_INIT_ENC_KEY_DIST | CY_BLE_GAP_SMP_INIT_IRK_KEY_DIST |
                                CY_BLE_GAP_SMP_INIT_CSRK_KEY_DIST | CY_BLE_GAP_SMP_RESP_ENC_KEY_DIST |
                                CY_BLE_GAP_SMP_RESP_IRK_KEY_DIST | CY_BLE_GAP_SMP_RESP_CSRK_KEY_DIST,
    };

    switch (event) {
        /**********************************************************
        *                       General Events
        ***********************************************************/
        case CY_BLE_EVT_STACK_ON: /* This event is received when the component is Started */
            DBG_PRINTF("CY_BLE_EVT_STACK_ON, StartAdvertisement \r\n");

            /* Enter into discoverable mode so that remote can find it. */
            apiResult =
                    Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);

            if (apiResult != CY_BLE_SUCCESS) {
                DBG_PRINTF("Cy_BLE_GAPP_StartAdvertisement API Error: 0x%x \r\n", apiResult);
            }
            DBG_PRINTF("Cy_BLE_GAPP_StartAdvertisement OK!\n");

            /* Generates the security keys */
            // Unused for simplicity.
            apiResult = Cy_BLE_GAP_GenerateKeys(&keyInfo);
            if (apiResult != CY_BLE_SUCCESS) {
                DBG_PRINTF("Cy_BLE_GAP_GenerateKeys API Error: 0x%x \r\n", apiResult);
            }
            DBG_PRINTF("Cy_BLE_GAP_GenerateKeys OK!\n");

            break;

        case CY_BLE_EVT_GET_DEVICE_ADDR_COMPLETE:
            DBG_PRINTF("CY_BLE_EVT_GET_DEVICE_ADDR_COMPLETE: ");
            for (i = CY_BLE_GAP_BD_ADDR_SIZE; i > 0u; i--) {
                DBG_PRINTF("%2.2x",
                           ((cy_stc_ble_bd_addrs_t*) ((cy_stc_ble_events_param_generic_t*) eventParam)->eventParams)
                                   ->publicBdAddr[i - 1]);
            }
            DBG_PRINTF("\r\n");

            break;

        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            DBG_PRINTF("CY_BLE_EVT_GAP_DEVICE_DISCONNECTED: bdHandle=%x, reason=%x, status=%x\r\n",
                       (*(cy_stc_ble_gap_disconnect_param_t*) eventParam).bdHandle,
                       (*(cy_stc_ble_gap_disconnect_param_t*) eventParam).reason,
                       (*(cy_stc_ble_gap_disconnect_param_t*) eventParam).status);

            // If Central unsubscribed from Peripheral (us), let's update subscription variable.
            isSubscribedToNtf = false;

            /* Put the device into discoverable mode so that a remote can search it. */
            apiResult =
                    Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            if (apiResult != CY_BLE_SUCCESS) {
                DBG_PRINTF("StartAdvertisement API Error: 0x%x \r\n", apiResult);
            }
            break;

        /**********************************************************
        *                       GATT Events
        ***********************************************************/
        case CY_BLE_EVT_GATT_CONNECT_IND:
            appConnHandle = *(cy_stc_ble_conn_handle_t*) eventParam;
            // Centrals may never request an MTU exchange (typical on Android), so start from the spec default
            mtu_val = CY_BLE_GATT_DEFAULT_MTU;
            DBG_PRINTF("CY_BLE_EVT_GATT_CONNECT_IND: %x, %x \r\n", (*(cy_stc_ble_conn_handle_t*) eventParam).attId,
                       (*(cy_stc_ble_conn_handle_t*) eventParam).bdHandle);
            break;

        case CY_BLE_EVT_GATT_DISCONNECT_IND:
            DBG_PRINTF("CY_BLE_EVT_GATT_DISCONNECT_IND: %x, %x \r\n", (*(cy_stc_ble_conn_handle_t*) eventParam).attId,
                       (*(cy_stc_ble_conn_handle_t*) eventParam).bdHandle);
            break;

        // Exchange MTU (Maximal Transmission Unit). To put it simply, those are max bytes in one packet.
        // Determined BOTH by Client and Peripheral. For peripheral, check BLE 5.0 Component!
        case CY_BLE_EVT_GATTS_XCNHG_MTU_REQ: {
            cy_stc_ble_gatt_xchg_mtu_param_t mtu = {
                    .connHandle = ((cy_stc_ble_gatt_xchg_mtu_param_t*) eventParam)->connHandle};
            Cy_BLE_GATT_GetMtuSize(&mtu);

            // Store MTU for further usage

            mtu_val = mtu.mtu;

            DBG_PRINTF("CY_BLE_EVT_GATTS_XCNHG_MTU_REQ %x, %x, final mtu= %d \r\n", mtu.connHandle.attId,
                       mtu.connHandle.bdHandle, mtu.mtu);
        } break;

        case CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ:
            /* Triggered on server side when client sends read request and when
            * characteristic has CY_BLE_GATT_DB_ATTR_CHAR_VAL_RD_EVENT property set.
            * This event could be ignored by application unless it need to response
            * by error response which needs to be set in gattErrorCode field of
            * event parameter. */
            DBG_PRINTF("CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ: handle: %x \r\n",
                       ((cy_stc_ble_gatts_char_val_read_req_t*) eventParam)->attrHandle);
            break;

        // Handle write request from Central. This is very important function.
        case CY_BLE_EVT_GATTS_WRITE_CMD_REQ:  // We won't distinguish commands and requests via BLE
        case CY_BLE_EVT_GATTS_WRITE_REQ:
            // Store and cast it just for the ease of use
            writeReqParam = (cy_stc_ble_gatts_write_cmd_req_param_t*) eventParam;

            DBG_PRINTF("CY_BLE_EVT_GATTS_WRITE_REQ: handle: %d \r\n", writeReqParam->handleValPair.attrHandle);
            DBG_PRINTF("Req len: %d\r\n", writeReqParam->handleValPair.value.len);

            // Separately check whether someone is trying to modify CCCD. This is Notification-related descriptor.
            // See BLE_config.h to find the definitions.
            if (writeReqParam->handleValPair.attrHandle == CY_BLE_NUS_UART_NUS_TX_TX_CCCD_DESC_HANDLE) {
                // This is of utmost importance! You need to update your CCCD Database to know whether someone has really subscribed or not.
                Cy_BLE_GATTS_WriteAttributeValuePeer(&appConnHandle, &writeReqParam->handleValPair);

                // In that case, Central will always sent 2 bytes. First would be either 0u or 1u. 1u => Subscribed
                isSubscribedToNtf = !!(writeReqParam->handleValPair.value.val[0]);

                DBG_PRINTF("NEW NTF STATUS: 0x%X\r\n", isSubscribedToNtf);
            } else if (writeReqParam->handleValPair.attrHandle == CY_BLE_NUS_UART_NUS_RX_CHAR_HANDLE) {
// Do not waste cycles if not in debug mode
#if DEBUG_UART_ENABLED == 1
                // Debug-display message/data
                for (uint16_t i = 0; i < writeReqParam->handleValPair.value.len; i++) {
                    DBG_PRINTF("%X/%c\r\n", writeReqParam->handleValPair.value.val[i],
                               writeReqParam->handleValPair.value.val[i]);
                }
#endif

                if (writeReqParam->handleValPair.value.len != 0) {
                    parseIncomingBLECommand(writeReqParam->handleValPair.value.val,
                                            writeReqParam->handleValPair.value.len);
                }
            }
            // Write Request needs a response; Write Command (without response) must not get one.
            if (event == CY_BLE_EVT_GATTS_WRITE_REQ) {
                Cy_BLE_GATTS_WriteRsp(writeReqParam->connHandle);
            }
            break;

        default: DBG_PRINTF("Other event: 0x%X \r\n", event); break;
    }
}

void BLE_NUS_Init(void) {
    DBG_PRINTF("BleLogService Init\n");

    volatile cy_en_ble_api_result_t apiResult;

    Cy_SysInt_Init(&blessIsrCfg, &BlessInterrupt);
    cy_ble_config.hw->blessIsrConfig = &blessIsrCfg;

    Cy_BLE_RegisterEventCallback(AppCallBack);
    apiResult = Cy_BLE_Init(&cy_ble_config);
    if (apiResult == CY_BLE_SUCCESS) {
        apiResult = Cy_BLE_Enable();
    }

    if (apiResult != CY_BLE_SUCCESS) {
        DBG_PRINTF("Failed to start BLE... err %d\n", apiResult);
    }

    DBG_PRINTF("BleLogService Init Success!\n");

    /* Register callback to handle response from CM4 */
    Cy_IPC_Pipe_RegisterCallback(CY_IPC_EP_CYPIPE_ADDR, CM0_MessageCallback, CY_IPC_EP_CYPIPE_CM4_ADDR);


    /***************************************************************************
    * Main polling loop
    ***************************************************************************/
    while (1) {
        /* Cy_BLE_ProcessEvents() allows BLE stack to process pending events */
        // This must be executed in order for BLE to function properly
        Cy_BLE_ProcessEvents();

        // Check for new messages from CM4 core and process them.
        if (CM0_isDataAvailableFromCM4()) {
            processIncomingIPCMessage(CM0_GetCM4Message());
        }

        // Mostly you don't want more code here!
    }
}

static void processIncomingIPCMessage(ipc_msg_t* msg) {
    // In general, impossible situation, but never trust anyone.
    if (msg != NULL) {
        // Process messages with at least 1 byte of payload
        if (msg->len > 0) {
            // Decode message code. You would mostly like to use Commands
            switch (msg->userCode) {
                case IPC_USR_CODE_CMD: {
                    // Only [0] element was used for simplicity.
                    // IDEA: You could expand it to accept strings or more complex data packets!
                    // You can define array of strings and perform memcmp. This operation is a bit more costful.
                    processCM0Command(msg);
                    break;
                }
                case IPC_USR_CODE_REQ: {
                    // Not implemented. You can be very creative here!
                    break;
                }
                default: break;
            }
        }
    }
}

// msg->buffer[0] is the command, msg->buffer[1..len-1] are its parameters.
static void processCM0Command(ipc_msg_t* msg) {
    switch (msg->buffer[0]) {
        case CM0_SHARED_BLE_NTF_RELAY: {
            DBG_PRINTF("Relay\r\n");
            sendDataNotificationNUS(&msg->buffer[1], msg->len - 1);
            break;
        }
        case CM0_SHARED_CAR_SAY: {
            DBG_PRINTF("Car Say\r\n");
            sendDataNotificationNUS((uint8_t*) carsay, strlen(carsay));
            break;
        }
        default: break;
    }
}


/* [] END OF FILE */
