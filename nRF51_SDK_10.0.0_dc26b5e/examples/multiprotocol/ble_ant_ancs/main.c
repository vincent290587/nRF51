/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 */

/** @file
 *
 * @defgroup ble_sdk_apple_notification_main main.c
 * @{
 * @ingroup ble_sdk_app_apple_notification
 * @brief Apple Notification Client Sample Application main file. Disclaimer: 
 * This client implementation of the Apple Notification Center Service can and 
 * will be changed at any time by Nordic Semiconductor ASA.
 *
 * Server implementations such as the ones found in iOS can be changed at any 
 * time by Apple and may cause this client implementation to stop working.
 *
 * This file contains the source code for a sample application using the Apple 
 * Notification Center Service Client.
 */

#include "myIncludes.h"
#include "antplus_mgr.h"


static const char * lit_catid[BLE_ANCS_NB_OF_CATEGORY_ID] =
{
    "Other",
    "Incoming Call",
    "Missed Call",
    "Voice Mail",
    "Social",
    "Schedule",
    "Email",
    "News",
    "Health And Fitness",
    "Business And Finance",
    "Location",
    "Entertainment"
};

static const char * lit_eventid[BLE_ANCS_NB_OF_EVT_ID] =
{
    "Added",
    "Modified",
    "Removed"
};

static const char * lit_attrid[BLE_ANCS_NB_OF_ATTRS] =
{
    "App Identifier",
    "Title",
    "Subtitle",
    "Message",
    "Message Size",
    "Date",
    "Positive Action Label",
    "Negative Action Label"
};

static ble_nus_t                 m_nus;                                      /**< Structure to identify the Nordic UART Service. */

static ble_ancs_c_t              m_ancs_c;                                 /**< Structure used to identify the Apple Notification Service Client. */

static uint16_t                  m_conn_handle = BLE_CONN_HANDLE_INVALID;  /**< Handle of the current connection. */

static ble_db_discovery_t        m_ble_db_discovery;                       /**< Structure used to identify the DB Discovery module. */

static uint8_t                   m_ancs_uuid_type;                         /**< Store ANCS UUID. */
static dm_application_instance_t m_app_handle;                             /**< Application identifier allocated by the Device Manager. */
static dm_handle_t               m_peer_handle;                            /**< Identifies the peer that is currently connected. */
static ble_gap_sec_params_t      m_sec_param;                              /**< Security parameter for use in security requests. */

APP_TIMER_DEF(m_sec_req_timer_id);                                         /**< Security request timer. The timer lets us start pairing request if one does not arrive from the Central. */
APP_TIMER_DEF(m_sec_ping); 

static ble_ancs_c_evt_notif_t m_notification_latest;                       /**< Local copy to keep track of the newest arriving notifications. */

static uint8_t m_attr_title[ATTR_DATA_SIZE];                               /**< Buffer to store attribute data. */
static uint8_t m_attr_subtitle[ATTR_DATA_SIZE];                            /**< Buffer to store attribute data. */
static uint8_t m_attr_message[ATTR_DATA_SIZE];                             /**< Buffer to store attribute data. */
static uint8_t m_attr_message_size[ATTR_DATA_SIZE];                        /**< Buffer to store attribute data. */
static uint8_t m_attr_date[ATTR_DATA_SIZE];                                /**< Buffer to store attribute data. */
static uint8_t m_attr_posaction[ATTR_DATA_SIZE];                           /**< Buffer to store attribute data. */
static uint8_t m_attr_negaction[ATTR_DATA_SIZE];                           /**< Buffer to store attribute data. */

static ble_uuid_t m_adv_uuids[] = {{ANCS_UUID_SERVICE,BLE_UUID_TYPE_VENDOR_BEGIN}};  /**< Universally unique service identifiers. */

static ble_uuid_t m_adv_uuids2[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifiers. */


/**@brief Callback function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. 
 *          You must analyze how your product should react to asserts.
 * @warning On assert from the SoftDevice, the system can recover only on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


void uart_error_handle(app_uart_evt_t * p_event)
{
    uint8_t read_byte = 0;
  
    if (p_event->evt_type == APP_UART_DATA_READY)
    {
      // get data
      while(app_uart_get(&read_byte) != NRF_SUCCESS) {;}
      if (encode(read_byte)) {
        transmit_order ();
      }
      
    }
  
    if (0) {
      if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
      {
        APP_ERROR_HANDLER(p_event->data.error_communication);
      }
      else if (p_event->evt_type == APP_UART_FIFO_ERROR)
      {
        APP_ERROR_HANDLER(p_event->data.error_code);
      }
    }

}


/**@brief Function for handling the security request timer time-out.
 *
 * @details This function is called each time the security request timer expires.
 *
 * @param[in] p_context  Pointer used for passing context information from the
 *                       app_start_timer() call to the time-out handler.
 */
static void sec_req_timeout_handler(void * p_context)
{
    uint32_t             err_code;
    dm_security_status_t status;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        err_code = dm_security_status_req(&m_peer_handle, &status);
        APP_ERROR_CHECK(err_code);

        // If the link is still not secured by the peer, initiate security procedure.
        if (status == NOT_ENCRYPTED)
        {
            err_code = dm_security_setup_req(&m_peer_handle);
            APP_ERROR_CHECK(err_code);
        }
    }
}


/**@brief Function for setting up GATTC notifications from the Notification Provider.
 *
 * @details This function is called when a successful connection has been established.
 */
static void apple_notification_setup(void)
{
    uint32_t err_code;

    nrf_delay_ms(100); // Delay because we cannot add a CCCD to close to starting encryption. iOS specific.

    err_code = ble_ancs_c_notif_source_notif_enable(&m_ancs_c);
    APP_ERROR_CHECK(err_code);

    err_code = ble_ancs_c_data_source_notif_enable(&m_ancs_c);
    APP_ERROR_CHECK(err_code);

    //SEGGER_RTT_WriteString(0, "Notifications Enabled.\n");
}


/**@brief Function for printing an iOS notification.
 *
 * @param[in] p_notif  Pointer to the iOS notification.
 */
static void notif_print(ble_ancs_c_evt_notif_t * p_notif)
{
    //SEGGER_RTT_WriteString(0, "Notification");
    printf("\nNotification\n");
    printf("Event:       %s\n", lit_eventid[p_notif->evt_id]);
    printf("Category ID: %s\n", lit_catid[p_notif->category_id]);
    printf("Category Cnt:%u\n", (unsigned int) p_notif->category_count);
    printf("UID:         %u\n", (unsigned int) p_notif->notif_uid);
  
    if (p_notif->evt_id==0) {
      //SEGGER_RTT_WriteString(0, ": ");
      //SEGGER_RTT_WriteString(0, lit_eventid[p_notif->evt_id]);
      //SEGGER_RTT_WriteString(0, " / ");
      //SEGGER_RTT_WriteString(0, lit_catid[p_notif->category_id]);
    }
    
    //SEGGER_RTT_WriteString(0, "   \n");
    

    printf("Flags:\n");
    if(p_notif->evt_flags.silent == 1)
    {
        printf(" Silent\n");
    }
    if(p_notif->evt_flags.important == 1)
    {
        printf(" Important\n");
    }
    if(p_notif->evt_flags.pre_existing == 1)
    {
        printf(" Pre-existing\n");
    }
    if(p_notif->evt_flags.positive_action == 1)
    {
        printf(" Positive Action\n");
    }
    if(p_notif->evt_flags.negative_action == 1)
    {
        printf(" Positive Action\n");
    }
}


/**@brief Function for printing iOS notification attribute data.
 * 
 * @param[in] p_attr         Pointer to an iOS notification attribute.
 * @param[in] ancs_attr_list Pointer to a list of attributes. Each entry in the list stores 
                             a pointer to its attribute data, which is to be printed.
 */
static void notif_attr_print(ble_ancs_c_evt_notif_attr_t * p_attr,
                             ble_ancs_c_attr_list_t      * ancs_attr_list)
{
    //SEGGER_RTT_WriteString(0, "Attributs: ");
    if (p_attr->attr_len != 0)
    {
        printf("%s: %s\n",
               lit_attrid[p_attr->attr_id],
               ancs_attr_list[p_attr->attr_id].p_attr_data);
        //SEGGER_RTT_WriteString(0, lit_attrid[p_attr->attr_id]);
        //SEGGER_RTT_WriteString(0, " / ");
        //SEGGER_RTT_WriteString(0, ancs_attr_list[p_attr->attr_id].p_attr_data);
    }
    else if (p_attr->attr_len == 0)
    {
        printf("%s: (N/A)\n", lit_attrid[p_attr->attr_id]);
        //SEGGER_RTT_WriteString(0, lit_attrid[p_attr->attr_id]);
    }
    //SEGGER_RTT_WriteString(0, "   \n");
}



// on fait un ping sur iOS pour eviter la deconnexion
static void sec_ping(void * p_context) {
     ble_nus_string_send(&m_nus, "PING", 4);
}

// on fait un ping sur iOS pour eviter la deconnexion
void nus_send(char * text) {
     ble_nus_string_send(&m_nus, text, strlen(text));
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module, making it use the scheduler.
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);

    // Create security request timer.
    err_code = app_timer_create(&m_sec_req_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                sec_req_timeout_handler);
    APP_ERROR_CHECK(err_code);
  
    err_code = app_timer_create(&m_sec_ping, APP_TIMER_MODE_REPEATED, sec_ping);
    APP_ERROR_CHECK(err_code);
  
    
}


/**@brief Function for handling the Apple Notification Service client.
 *
 * @details This function is called for all events in the Apple Notification client that
 *          are passed to the application.
 *
 * @param[in] p_evt  Event received from the Apple Notification Service client.
 */
static void on_ancs_c_evt(ble_ancs_c_evt_t * p_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_evt->evt_type)
    {
        case BLE_ANCS_C_EVT_DISCOVER_COMPLETE:
            //SEGGER_RTT_WriteString(0, "Apple Notification Service discovered on the server.\n");
            apple_notification_setup();
            break;

        case BLE_ANCS_C_EVT_NOTIF:
            m_notification_latest = p_evt->notif;
            notif_print(&m_notification_latest);
            break;

        case BLE_ANCS_C_EVT_NOTIF_ATTRIBUTE:
            notif_attr_print(&p_evt->attr, p_evt->ancs_attr_list);
            break;

        case BLE_ANCS_C_EVT_DISCOVER_FAILED:
            // ANCS not found.
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
            //SEGGER_RTT_WriteString(0, "Apple Notification Service not discovered on the server.\n");
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the GAP.
 *
 * @details Use this function to set up all necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Apple Notification Service client errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void apple_notification_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the Device Manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_evt,
                                           ret_code_t          event_result)
{
    uint32_t err_code;
    APP_ERROR_CHECK(event_result);
    ble_ancs_c_on_device_manager_evt(&m_ancs_c, p_handle, p_evt);

    switch (p_evt->event_id)
    {
        case DM_EVT_CONNECTION:
            m_peer_handle = (*p_handle);
            err_code      = app_timer_start(m_sec_req_timer_id, SECURITY_REQUEST_DELAY, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case DM_EVT_LINK_SECURED:
            err_code = ble_db_discovery_start(&m_ble_db_discovery,
                                              p_evt->event_param.p_gap_param->conn_handle);
            APP_ERROR_CHECK(err_code);
            break; 

        default:
            break;

    }
    return NRF_SUCCESS;
}


/**@brief Function for initializing the Device Manager.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    memcpy(&m_sec_param, &register_param.sec_param, sizeof(ble_gap_sec_params_t));

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    //uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    //APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    //err_code = bsp_btn_ble_sleep_mode_prepare();
    //APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    //err_code = sd_power_system_off();
    //APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function is called for advertising events that are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_DIRECTED:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_whitelist_t whitelist;
            ble_gap_addr_t    * p_whitelist_addr[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t     * p_whitelist_irk[BLE_GAP_WHITELIST_IRK_MAX_COUNT];

            whitelist.addr_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            whitelist.irk_count  = BLE_GAP_WHITELIST_IRK_MAX_COUNT;
            whitelist.pp_addrs   = p_whitelist_addr;
            whitelist.pp_irks    = p_whitelist_irk;

            err_code = dm_whitelist_create(&m_app_handle, &whitelist);
            APP_ERROR_CHECK(err_code);

            err_code = ble_advertising_whitelist_reply(&whitelist);
            APP_ERROR_CHECK(err_code);
            break;
        }

        default:
            break;
    }
}


/**@brief Function for handling the application's BLE stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            //SEGGER_RTT_WriteString(0, "Connected.\n");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
        
            err_code = app_timer_start(m_sec_ping, PING_DELAY, NULL);
            APP_ERROR_CHECK(err_code);

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            //SEGGER_RTT_WriteString(0, "Disconnected.\n");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
        
            err_code = app_timer_stop(m_sec_ping);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
        case BLE_GATTS_EVT_TIMEOUT:
            //SEGGER_RTT_WriteString(0, "Timeout.\n");
            // Disconnect on GATT Server and Client time-out events.
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_KEY_1:
            err_code = ble_ancs_c_request_attrs(&m_notification_latest);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_ancs_c_on_ble_evt(&m_ancs_c, p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    //bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the system event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}



/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#if (defined(S130) || defined(S132) || defined(S310))
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for System events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}



#define TAILLE_BUFFER 30

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
  uint32_t i;
  char buffer[TAILLE_BUFFER];
  
    memset(buffer, 0, sizeof(buffer));
  
    if (length < 2) return;

    ////SEGGER_RTT_WriteString(0, p_data);
    for (i = 0; i < length && i < TAILLE_BUFFER - 5; i++) {
        buffer[i] = p_data[i];
        buffer[i+1] = '\n';
        buffer[i+2] = '\0';
    }
    
    //SEGGER_RTT_WriteString(0, buffer);
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing the Apple Notification Center Service.
*/
static void services_init(void)
{
    ble_ancs_c_init_t ancs_init_obj;
    ble_uuid_t        service_uuid;
    uint32_t          err_code;

    err_code = sd_ble_uuid_vs_add(&ble_ancs_base_uuid128, &m_ancs_uuid_type);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_uuid_vs_add(&ble_ancs_cp_base_uuid128, &service_uuid.type);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_uuid_vs_add(&ble_ancs_ns_base_uuid128, &service_uuid.type);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_uuid_vs_add(&ble_ancs_ds_base_uuid128, &service_uuid.type);
    APP_ERROR_CHECK(err_code);

    memset(&ancs_init_obj, 0, sizeof(ancs_init_obj));

    err_code = ble_ancs_c_attr_add(BLE_ANCS_NOTIF_ATTR_ID_TITLE, m_attr_title, ATTR_DATA_SIZE);
    APP_ERROR_CHECK(err_code);
    
    err_code = ble_ancs_c_attr_add(BLE_ANCS_NOTIF_ATTR_ID_SUBTITLE,
                                   m_attr_subtitle,
                                   ATTR_DATA_SIZE);
    APP_ERROR_CHECK(err_code);

    err_code = ble_ancs_c_attr_add(BLE_ANCS_NOTIF_ATTR_ID_MESSAGE, m_attr_message, ATTR_DATA_SIZE);
    APP_ERROR_CHECK(err_code);

    err_code = ble_ancs_c_attr_add(BLE_ANCS_NOTIF_ATTR_ID_MESSAGE_SIZE,
                                   m_attr_message_size,
                                   ATTR_DATA_SIZE);
    APP_ERROR_CHECK(err_code);

    err_code = ble_ancs_c_attr_add(BLE_ANCS_NOTIF_ATTR_ID_DATE, m_attr_date, ATTR_DATA_SIZE);
    APP_ERROR_CHECK(err_code);

    err_code = ble_ancs_c_attr_add(BLE_ANCS_NOTIF_ATTR_ID_POSITIVE_ACTION_LABEL,
                                   m_attr_posaction,
                                   ATTR_DATA_SIZE);
    APP_ERROR_CHECK(err_code);

    err_code = ble_ancs_c_attr_add(BLE_ANCS_NOTIF_ATTR_ID_NEGATIVE_ACTION_LABEL,
                                   m_attr_negaction,
                                   ATTR_DATA_SIZE);
    APP_ERROR_CHECK(err_code);

    ancs_init_obj.evt_handler   = on_ancs_c_evt;
    ancs_init_obj.error_handler = apple_notification_error_handler;

    err_code = ble_ancs_c_init(&m_ancs_c, &ancs_init_obj);
    APP_ERROR_CHECK(err_code);
    

    ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

}



/**@brief Function for initializing the advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

     // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type                = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance       = true;
    advdata.flags                    = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
  
    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids2) / sizeof(m_adv_uuids2[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids2;
  
    advdata.uuids_complete.uuid_cnt  = 0;
    advdata.uuids_complete.p_uuids   = NULL;
    advdata.uuids_solicited.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_solicited.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_whitelist_enabled = BLE_ADV_WHITELIST_ENABLED;
    options.ble_adv_fast_enabled      = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval     = APP_ADV_FAST_INTERVAL;
    options.ble_adv_fast_timeout      = APP_ADV_FAST_TIMEOUT;
    options.ble_adv_slow_enabled      = BLE_ADV_SLOW_ENABLED;
    options.ble_adv_slow_interval     = APP_ADV_SLOW_INTERVAL;
    options.ble_adv_slow_timeout      = APP_ADV_SLOW_TIMEOUT;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the UART.
 */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and LEDs.
 *
 * @param[out] p_erase_bonds  True if the clear bonds button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    uint32_t err_code = bsp_init(BSP_INIT_LED,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Event Scheduler.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for initializing the database discovery module.
 */
static void db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void)
{
    bool     erase_bonds;
    uint32_t err_code;

    // Initialize.
    timers_init();
    uart_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    device_manager_init(erase_bonds);
    db_discovery_init();
    scheduler_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
  
    ant_init();

    // Start execution.
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    //SEGGER_RTT_WriteString(0, "BLE ANCS\r\n");
    
    // Enter main loop.
    for (;;)
    {
        app_sched_execute();
        power_manage();
    }

}


/**
 * @}
 */

