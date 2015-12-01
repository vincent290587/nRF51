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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
//#include <SEGGER_RTT.h>

#include "ble_ancs_c.h"
#include "ble_db_discovery.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble_hci.h"

#include "ble_nus.h"

#include "ble_gap.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "nrf_gpio.h"
#include "ble_srv_common.h"
#include "ble_conn_params.h"
#include "nrf51_bitfields.h"
#include "device_manager.h"
#include "app_button.h"
#include "app_timer.h"
#include "pstorage.h"
#include "nrf_soc.h"
#include "bsp.h"
#include "softdevice_handler.h"
#include "app_uart.h"
#include "app_trace.h"
#include "nrf_delay.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "app_pwm.h"

#include "ant_stack_config.h"
#include "ant_hrm.h"
#include "ant_cad.h"
#include "ant_glasses.h"
#include "ant_interface.h"
#include "ant_state_indicator.h"

#include "tunes.h"

#define TAILLE_BUFFER 30

#define IS_SRVC_CHANGED_CHARACT_PRESENT 1

// Multiple ANT channel settings
#define ANT_STACK_TOTAL_CHANNELS_ALLOCATED  4               // (SIZE_OF_NONENCRYPTED_ANT_CHANNEL bytes)
#define ANT_STACK_ENCRYPTED_CHANNELS        0               // (SIZE_OF_ENCRYPTED_ANT_CHANNEL bytes)
#define ANT_STACK_TX_BURST_QUEUE_SIZE       128             // (128 bytes)


#define HRM_RX_CHANNEL_NUMBER       0x00                        /**< Channel number assigned to HRM profile. */
#define CAD_RX_CHANNEL_NUMBER       0x01
#define GLASSES_TX_CHANNEL_NUMBER   0x02

#define WILDCARD_TRANSMISSION_TYPE  0x00                        /**< Wildcard transmission type. */
#define WILDCARD_DEVICE_NUMBER      0x0000                        /**< Wildcard device number. */
#define HRM_DEVICE_NUMBER           0x0D22                        
#define CAD_DEVICE_NUMBER           0xB02B                      

#define ANTPLUS_NETWORK_NUMBER      0x00                        /**< Network number. */
#define HRMRX_NETWORK_KEY           {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45}    /**< The default network key used. */

#define UART_TX_BUF_SIZE                1024                                        /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                1                                          /**< UART RX buffer size. */

#define ATTR_DATA_SIZE                  BLE_ANCS_ATTR_DATA_MAX                      /**< Allocated size for attribute data. */

#define DISPLAY_MESSAGE_BUTTON_ID       1                                           /**< Button used to request notification attributes. */

#define DEVICE_NAME                     "ANCS"                                      /**< Name of the device. Will be included in the advertising data. */
#define APP_ADV_FAST_INTERVAL           40                                          /**< The advertising interval (in units of 0.625 ms). The default value corresponds to 25 ms. */
#define APP_ADV_SLOW_INTERVAL           3200                                        /**< Slow advertising interval (in units of 0.625 ms). The default value corresponds to 2 seconds. */
#define APP_ADV_FAST_TIMEOUT            180                                         /**< The advertising time-out in units of seconds. */
#define APP_ADV_SLOW_TIMEOUT            180                                         /**< The advertising time-out in units of seconds. */
#define ADV_INTERVAL_FAST_PERIOD        30                                          /**< The duration of the fast advertising period (in seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            10                  /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         5                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating an event (connect or start of notification) to the first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define MESSAGE_BUFFER_SIZE             18                                          /**< Size of buffer holding optional messages in notifications. */

#define SECURITY_REQUEST_DELAY          APP_TIMER_TICKS(1500, APP_TIMER_PRESCALER)  /**< Delay after connection until security request is sent, if necessary (ticks). */
#define PING_DELAY                      APP_TIMER_TICKS(10000, APP_TIMER_PRESCALER)
#define ANT_DELAY                       APP_TIMER_TICKS(10000, APP_TIMER_PRESCALER)

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump. Can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE, \
                                            BLE_STACK_HANDLER_SCHED_EVT_SIZE) /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                10                                    /**< Maximum number of events in the scheduler queue. */

APP_PWM_INSTANCE(PWM1, 1);                   // Create the instance "PWM1" using TIMER1.


#ifdef TRACE_DEBUG
#define LOG printf
#else
#define LOG(...)
#endif // TRACE_HRM_GENERAL_ENABLE 


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

static volatile bool ready_flag;            // A flag indicating PWM status.

static ble_ancs_c_t              m_ancs_c;                                 /**< Structure used to identify the Apple Notification Service Client. */
static ble_nus_t                 m_nus;                                      /**< Structure to identify the Nordic UART Service. */

static uint16_t                  m_conn_handle = BLE_CONN_HANDLE_INVALID;  /**< Handle of the current connection. */

static ble_db_discovery_t        m_ble_db_discovery;                       /**< Structure used to identify the DB Discovery module. */

static uint8_t                   m_ancs_uuid_type;                         /**< Store ANCS UUID. */
static dm_application_instance_t m_app_handle;                             /**< Application identifier allocated by the Device Manager. */
static dm_handle_t               m_peer_handle;                            /**< Identifies the peer that is currently connected. */
static ble_gap_sec_params_t      m_sec_param;                              /**< Security parameter for use in security requests. */

static app_timer_id_t            m_sec_req_timer_id;                       /**< Security request timer. The timer lets us start pairing request if one does not arrive from the Central. */
static app_timer_id_t            m_sec_ping;
static app_timer_id_t            m_sec_hrm;
static app_timer_id_t            m_sec_cad;
static app_timer_id_t            m_sec_tune;

static ble_ancs_c_evt_notif_t m_notification_latest;                       /**< Local copy to keep track of the newest arriving notifications. */

static uint8_t m_attr_title[ATTR_DATA_SIZE];                               /**< Buffer to store attribute data. */
static uint8_t m_attr_subtitle[ATTR_DATA_SIZE];                            /**< Buffer to store attribute data. */
static uint8_t m_attr_message[ATTR_DATA_SIZE];                             /**< Buffer to store attribute data. */
static uint8_t m_attr_message_size[ATTR_DATA_SIZE];                        /**< Buffer to store attribute data. */
static uint8_t m_attr_date[ATTR_DATA_SIZE];                                /**< Buffer to store attribute data. */
static uint8_t m_attr_posaction[ATTR_DATA_SIZE];                           /**< Buffer to store attribute data. */
static uint8_t m_attr_negaction[ATTR_DATA_SIZE];                           /**< Buffer to store attribute data. */

static ble_uuid_t m_adv_uuids[] = {{ANCS_UUID_SERVICE,BLE_UUID_TYPE_VENDOR_BEGIN}};  /**< Universally unique service identifiers. */

static ble_uuid_t m_adv_uuids2[] = {{BLE_UUID_NUS_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}};  /**< Universally unique service identifiers. */

static uint8_t m_network_key[] = HRMRX_NETWORK_KEY;             /**< ANT PLUS network key. */

/** @snippet [ANT HRM RX Instance] */
ant_hrm_profile_t           m_ant_hrm;
ant_cad_profile_t           m_ant_cad;
ant_glasses_profile_t       m_ant_glasses;
const ant_channel_config_t  ant_rx_channel_config = HRM_RX_CHANNEL_CONFIG(HRM_RX_CHANNEL_NUMBER, WILDCARD_TRANSMISSION_TYPE,
                                                    HRM_DEVICE_NUMBER, ANTPLUS_NETWORK_NUMBER, HRM_MSG_PERIOD_4Hz);

const ant_channel_config_t  ant_rx_channel_config2 = CAD_RX_CHANNEL_CONFIG(CAD_RX_CHANNEL_NUMBER, WILDCARD_TRANSMISSION_TYPE,
                                                    CAD_DEVICE_NUMBER, ANTPLUS_NETWORK_NUMBER, CAD_MSG_PERIOD);

const ant_channel_config_t  ant_tx_channel_config  = GLASSES_TX_CHANNEL_CONFIG(GLASSES_TX_CHANNEL_NUMBER, 5,
                                                    GLASSES_DEVICE_NUMBER, ANTPLUS_NETWORK_NUMBER);

static uint8_t is_hrm_init = 0;
static uint8_t is_cad_init = 0;

static uint8_t isTunePlaying = 0;
static uint32_t iTune = 0;

#if (LEDS_NUMBER > 0)
const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;
#else
const uint8_t leds_list;
#endif

static char notif_message[BLE_ANCS_ATTR_DATA_MAX];
static char notif_title  [BLE_ANCS_ATTR_DATA_MAX];

static uint8_t read_byte[100];
static uint8_t glasses_payload[8];
static uint16_t status_byte;

static uint8_t pwm_ready = 0;

static void play_tune ();

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


static void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    pwm_ready = 1;
}


// on fait un ping sur iOS pour eviter la deconnexion
void nus_send_const(const char * text) {
     uint8_t txt[25] = {0};
     if (strlen(text) < 23) {
       memcpy(txt, text, strlen(text));
       ble_nus_string_send(&m_nus, txt, strlen(text));
     } else {
       memcpy(txt, text, 23);
       ble_nus_string_send(&m_nus, txt, 23);
     }
}


// on fait un ping sur iOS pour eviter la deconnexion
void nus_send(char * text) {
     nus_send_const(text);
}


// on fait un ping sur iOS pour eviter la deconnexion
static void sec_ping(void * p_context) {
     uint8_t ping[7] = {0};
     memcpy(ping, "PING", 4);
     ble_nus_string_send(&m_nus, ping, 4);
}

// on fait un ping sur iOS pour eviter la deconnexion
static void sec_tune(void * p_context) {
   play_tune();
}


uint8_t encode (uint8_t byte) {
  
   switch (byte) {
     case '$':
       status_byte = 0;
       //memset(read_byte, 0, sizeof(read_byte));
     
       break;
     case '\n':
       LOG("%s\n\r", read_byte);
       status_byte = 0;
       //memset(read_byte, 0, sizeof(read_byte));
       return 1;
     
     default:
       if (status_byte < sizeof(read_byte) - 10) {
         read_byte[status_byte] = byte;
         status_byte++;
       } else {
         status_byte = 0;
       }
       break;
   }
  
   return 0;
}


void transmit_order () {
   // pass-through
   uint8_t i;
  
   LOG("Envoi aux lunettes: \"%c%c%c%c%c%c%c%c\"\n\r", 
                   read_byte[0], read_byte[1],
                   read_byte[2], read_byte[3],
                   read_byte[4], read_byte[5],
                   read_byte[6], read_byte[7]);
  

   memset(glasses_payload, 0, 8);
  
   for (i=0; i<4; i++) {
     if (read_byte[2*i] <= '9') {
       glasses_payload[i] = read_byte[2*i] - '0';
     } else if (read_byte[2*i] <= 'F') {
       // lettres majuscules
       glasses_payload[i] = 10 + read_byte[2*i] - 'A';
     } else {
       // lettres minuscules
       glasses_payload[i] = 10 + read_byte[2*i] - 'a';
     }
     
     glasses_payload[i] *= 16;
     
     if (read_byte[2*i+1] <= '9') {
       glasses_payload[i] += read_byte[2*i+1] - '0';
     } else if (read_byte[2*i+1] <= 'F') {
       // lettres majuscules
       glasses_payload[i] += 10 + read_byte[2*i+1] - 'A';
     } else {
       // lettres minuscules
       glasses_payload[i] += 10 + read_byte[2*i+1] - 'a';
     }
     
   }
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

static void hrm_connect(void * p_context)
{
  uint32_t err_code = NRF_SUCCESS;
  
  err_code = ant_hrm_open(&m_ant_hrm);
  APP_ERROR_CHECK(err_code);
}

static void cad_connect(void * p_context)
{
  uint32_t err_code = NRF_SUCCESS;
  
  err_code = ant_cad_open(&m_ant_cad);
  APP_ERROR_CHECK(err_code);
}


void ant_evt_glasses (ant_evt_t * p_ant_evt)
{
    uint32_t err_code = NRF_SUCCESS;
    
    switch (p_ant_evt->event)
					{
							case EVENT_TX:
                  ant_glasses_tx_evt_handle(&m_ant_glasses, p_ant_evt, glasses_payload);
                  LOG("Payload: \"%x %x %x %x %x %x %x %x\"\n\r", 
                   glasses_payload[0], glasses_payload[1],
                   glasses_payload[2], glasses_payload[3],
                   glasses_payload[4], glasses_payload[5],
                   glasses_payload[6], glasses_payload[7]);
									break;
              case EVENT_RX:
									break;
							case EVENT_RX_FAIL:
									break;
							case EVENT_RX_FAIL_GO_TO_SEARCH:
									break;
							case EVENT_CHANNEL_CLOSED:
							case EVENT_RX_SEARCH_TIMEOUT:
									break;
					}
          
    APP_ERROR_CHECK(err_code);
}

void ant_evt_cad (ant_evt_t * p_ant_evt)
{
    uint32_t err_code = NRF_SUCCESS;
    
    uint16_t pusDeviceNumber = 0;
    uint8_t pucDeviceType    = 0;
    uint8_t pucTransmitType  = 0;
	

    switch (p_ant_evt->event)
					{
							case EVENT_RX:
                  if (!is_cad_init) {
                    sd_ant_channel_id_get (CAD_RX_CHANNEL_NUMBER, 
                          &pusDeviceNumber, &pucDeviceType, &pucTransmitType);
                    printf("$ANCS,0,CAD 0x%x connected\n\r", pusDeviceNumber);
                    if (pusDeviceNumber) is_cad_init = 1;
                  }
              
									break;
							case EVENT_RX_FAIL:
									break;
							case EVENT_RX_FAIL_GO_TO_SEARCH:
									break;
							case EVENT_CHANNEL_CLOSED:
							case EVENT_RX_SEARCH_TIMEOUT:
                  LOG("Reconnexion CAD...\n\r");
                  is_cad_init = 0;
                  err_code = app_timer_stop(m_sec_cad);
									err_code = app_timer_start(m_sec_cad, ANT_DELAY, NULL);
									break;
					}
          
    APP_ERROR_CHECK(err_code);
}

void ant_evt_hrm (ant_evt_t * p_ant_evt)
{
    uint32_t err_code = NRF_SUCCESS;
    
    uint16_t pusDeviceNumber = 0;
    uint8_t pucDeviceType    = 0;
    uint8_t pucTransmitType  = 0;

    switch (p_ant_evt->event)
					{
							case EVENT_RX:
                  if (!is_hrm_init) {
                    sd_ant_channel_id_get (HRM_RX_CHANNEL_NUMBER, 
                          &pusDeviceNumber, &pucDeviceType, &pucTransmitType);
                    printf("$ANCS,0,HRM 0x%x connected\n\r", pusDeviceNumber);
                    if (pusDeviceNumber) is_hrm_init = 1;
                  }
      
									break;
							case EVENT_RX_FAIL:
									break;
							case EVENT_RX_FAIL_GO_TO_SEARCH:
									break;
							case EVENT_CHANNEL_CLOSED:
							case EVENT_RX_SEARCH_TIMEOUT:
                  LOG("Reconnexion HRM...\n\r");
                  is_hrm_init = 0;
                  err_code = app_timer_stop(m_sec_hrm);
									err_code = app_timer_start(m_sec_hrm, ANT_DELAY, NULL);
                  break;
					}
          
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for dispatching a ANT stack event to all modules with a ANT stack event handler.
 *
 * @details This function is called from the ANT Stack event interrupt handler after a ANT stack
 *          event has been received.
 *
 * @param[in] p_ant_evt  ANT stack event.
 */
void ant_evt_dispatch(ant_evt_t * p_ant_evt)
{

	  switch(p_ant_evt->channel) {
			case HRM_RX_CHANNEL_NUMBER:
				ant_evt_hrm (p_ant_evt);
        ant_hrm_rx_evt_handle(&m_ant_hrm, p_ant_evt);
				break;
      
			case CAD_RX_CHANNEL_NUMBER:
				ant_evt_cad (p_ant_evt);
        ant_cad_rx_evt_handle(&m_ant_cad, p_ant_evt);
				break;
      
      case GLASSES_TX_CHANNEL_NUMBER:
        ant_evt_glasses (p_ant_evt);
        break;
      
			default:
				break;
		}

    // BSP_LED_0_MASK  BSP_LED_1_MASK BSP_LED_2_MASK
    LEDS_INVERT(BSP_LED_2_MASK);
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

    LOG("Notifications Enabled.\n\r");
}


/**@brief Function for printing an iOS notification.
 *
 * @param[in] p_notif  Pointer to the iOS notification.
 */
static void notif_print(ble_ancs_c_evt_notif_t * p_notif)
{
    //LOG("\n\rNotification\n\r");
    //LOG("Event %d :       %s\n", p_notif->evt_id, lit_eventid[p_notif->evt_id]);
    //LOG("Category ID %d: %s\n\r\n\r", p_notif->category_id, lit_catid[p_notif->category_id]);
    //LOG("Category Cnt:%u\n", (unsigned int) p_notif->category_count);
    //LOG("UID:         %u\n\r", (unsigned int) p_notif->notif_uid);
	
	  //LOG("Title     : %s\n", m_attr_title);
	  //LOG("Subtitle  : %s\n", m_attr_subtitle);
	  //LOG("Message   : %s\n\r", m_attr_message);
    //nus_send_const(lit_eventid[p_notif->evt_id]);
    //nus_send(m_attr_subtitle);
    //nus_send(m_attr_message);

    if (p_notif->evt_id == 0 && 
         (p_notif->category_id == 1 ||
          p_notif->category_id == 2 ||
          p_notif->category_id == 3)) {
	    printf("$ANCS,1, ,%s\n\r", lit_catid[p_notif->category_id]);
    }
	
	  //memset(m_attr_title  , 0, ATTR_DATA_SIZE*sizeof(char));
	  //memset(m_attr_message, 0, ATTR_DATA_SIZE*sizeof(char));

	  // BSP_LED_0_MASK  BSP_LED_1_MASK BSP_LED_2_MASK
    LEDS_ON(BSP_LED_0_MASK);
#ifndef BSP_SIMPLE
    nrf_delay_ms(200);
#endif
    LEDS_OFF(BSP_LED_0_MASK);

	  return;
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

	  LOG("\n\rNotif_attr_print\n\r");
  
    if (p_attr->attr_len != 0)
    {
        LOG("%s: %s\n\r",
               lit_attrid[p_attr->attr_id],
               ancs_attr_list[p_attr->attr_id].p_attr_data);
    }
    else if (p_attr->attr_len == 0)
    {
        LOG("%s: (N/A)\n\r", lit_attrid[p_attr->attr_id]);
    }
    
	
    if (p_attr->attr_id==1) {
      memset(notif_title, 0, ATTR_DATA_SIZE);
	    memcpy(notif_title, ancs_attr_list[p_attr->attr_id].p_attr_data, 
        p_attr->attr_len >= ATTR_DATA_SIZE-3 ? ATTR_DATA_SIZE-3 : p_attr->attr_len);
    } else if (p_attr->attr_id==3) {
      memset(notif_message, 0, ATTR_DATA_SIZE);
      memcpy(notif_message, ancs_attr_list[p_attr->attr_id].p_attr_data,
        p_attr->attr_len >= ATTR_DATA_SIZE-3 ? ATTR_DATA_SIZE-3 : p_attr->attr_len);
    }
    
    char *pos;
    pos = strstr(notif_message, "$");
    if (pos != 0) {
      *pos = 0;
    }
    pos = strstr(notif_message, "@");
    if (pos != 0) {
      *pos = 0;
    }
    
    if (p_attr->attr_id==3) {

        // on sort la notif
        printf("$ANCS,1,%s,%s\n\r", notif_title, notif_message);
        //SEGGER_RTT_WriteString(0, "Notif_attr_print: ");
        //SEGGER_RTT_WriteString(0, notif_message);
        //SEGGER_RTT_WriteString(0, "  \n");
        
    }
    
  
}


/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module, making it use the scheduler.
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);

    // Create security request timer.
    err_code = app_timer_create(&m_sec_req_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                sec_req_timeout_handler);
    APP_ERROR_CHECK(err_code);
  
    err_code = app_timer_create(&m_sec_ping, APP_TIMER_MODE_REPEATED, sec_ping);
    APP_ERROR_CHECK(err_code);
  
    err_code = app_timer_create(&m_sec_hrm, APP_TIMER_MODE_SINGLE_SHOT, hrm_connect);
    APP_ERROR_CHECK(err_code);
  
    err_code = app_timer_create(&m_sec_cad, APP_TIMER_MODE_SINGLE_SHOT, cad_connect);
    APP_ERROR_CHECK(err_code);
  
    err_code = app_timer_create(&m_sec_tune, APP_TIMER_MODE_SINGLE_SHOT, sec_tune);
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
            LOG("Apple Notification Service discovered on the server.\n");
            apple_notification_setup();
            break;

        case BLE_ANCS_C_EVT_NOTIF:
            m_notification_latest = p_evt->notif;
            //notif_print(&m_notification_latest);
            ble_ancs_c_request_attrs(&m_notification_latest);
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
            LOG("Apple Notification Service not discovered on the server.\n");
            break;

        default:
            // No implementation needed.
            break;
    }
}


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

    for (i = 0; i < length && i < TAILLE_BUFFER - 5; i++) {
        buffer[i] = p_data[i];
        buffer[i+1] = '\n';
        buffer[i+2] = '\0';
    }
    nus_send(buffer);
    
}
/**@snippet [Handling the data received over BLE] */


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
            //sleep_mode
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
            printf("$ANCS,0,ANCS connected\n\r");
            //SEGGER_RTT_WriteString(0, "Connected\n");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            err_code = app_timer_start(m_sec_ping, PING_DELAY, NULL);
            APP_ERROR_CHECK(err_code);
				
				    // BSP_LED_0_MASK  BSP_LED_1_MASK BSP_LED_2_MASK
            LEDS_OFF(BSP_LED_1_MASK);

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            printf("$ANCS,0,ANCS disconnected\n\r");
            //SEGGER_RTT_WriteString(0, "Disconnected\n");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
				
				    // BSP_LED_0_MASK  BSP_LED_1_MASK BSP_LED_2_MASK
            LEDS_OFF(BSP_LED_1_MASK);
        
            err_code = app_timer_stop(m_sec_ping);
            APP_ERROR_CHECK(err_code);
            
#ifndef BSP_SIMPLE
            uint8_t var = 20;
            while (var--) {
              nrf_delay_ms(200);
              LEDS_INVERT(BSP_LED_1_MASK);
            }
#else
            nrf_delay_ms(200);
#endif
            break;

        case BLE_GATTC_EVT_TIMEOUT:
        case BLE_GATTS_EVT_TIMEOUT:
            LOG("Timeout.\n\r");
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
    
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            // sleep_mode
            break;

        case BSP_EVENT_DISCONNECT:
            break;

        case BSP_EVENT_WHITELIST_OFF:
            break;

        case BSP_EVENT_KEY_1:
            ble_ancs_c_request_attrs(&m_notification_latest);
            break;

        default:
            break;
    }
}


static void pwm_start(uint32_t period_us) {
  
    /* 2-channel PWM, 200Hz, output on DK LED pins. */
    //app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(period_us, 22, 24);
    
    /* Switch the polarity of the second channel. */
    //pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
  
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(period_us, 22);
    nrf_gpio_cfg_output(24);
    nrf_gpio_pin_clear(24);
    
    /* Initialize and enable PWM. */
    uint32_t err_code = app_pwm_init(&PWM1,&pwm1_cfg, pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);
  
    pwm_ready = 0;
    while (app_pwm_channel_duty_set(&PWM1, 0, 50) == NRF_ERROR_BUSY) {
      nrf_delay_ms(1);
    }
    while (!pwm_ready) {
      nrf_delay_ms(1);
    }
    APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM1, 1, 50));
    
}


static void pwm_stop() {
    app_pwm_disable(&PWM1);
    uint32_t err_code = app_pwm_uninit(&PWM1);
    APP_ERROR_CHECK(err_code);
}

static void play_mario() {
   isTunePlaying = 0;
   iTune = NOTES_NB;
   play_tune ();
}

static void play_tune () {
  
  uint32_t err_code, delay, period_us;
  
  if (iTune > 1) {
    // indice non nul
    if (isTunePlaying) {
      // on vient de jouer une note
      // on joue un blanc
      isTunePlaying = 0;
      pwm_stop();
      delay = APP_TIMER_TICKS(1000 / tempo[NOTES_NB - iTune], APP_TIMER_PRESCALER);
      err_code = app_timer_start(m_sec_tune, delay, NULL);
      APP_ERROR_CHECK(err_code);
      
      iTune--;
    } else {
      // on vient de jouer un blanc ou on commence
      if (iTune - 1 > 1) {
        // on relance
        if (melody[NOTES_NB - iTune] > 0) {
          period_us = 1000000 / melody[NOTES_NB - iTune];
          pwm_start(period_us);
        }
        delay = APP_TIMER_TICKS(1300 / tempo[NOTES_NB - iTune], APP_TIMER_PRESCALER);
        err_code = app_timer_start(m_sec_tune, delay, NULL);
        APP_ERROR_CHECK(err_code);
      }
      
      isTunePlaying = 1;
    }
    
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

#if defined(S110) || defined(S130) || defined(S310)
    // Enable BLE stack.
#if defined(S130) || defined(S310)
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
#endif

    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for System events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    err_code = softdevice_ant_evt_handler_set(ant_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // TODO check this
    err_code = ant_stack_static_config();
    APP_ERROR_CHECK(err_code);

    err_code = sd_ant_network_address_set(ANTPLUS_NETWORK_NUMBER, m_network_key);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for HRM profile initialization.
 *
 * @details Initializes the HRM profile and open ANT channel.
 */
static void ant_profile_setup(void)
{
/** @snippet [ANT HRM RX Profile Setup] */
    uint32_t err_code;
        
    // HRM
    err_code = ant_hrm_init(&m_ant_hrm, &ant_rx_channel_config, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = ant_hrm_open(&m_ant_hrm);
    APP_ERROR_CHECK(err_code);

    
    // CAD
    err_code = ant_cad_init(&m_ant_cad, &ant_rx_channel_config2);
    APP_ERROR_CHECK(err_code);

    err_code = ant_cad_open(&m_ant_cad);
    APP_ERROR_CHECK(err_code);
  
  
    // GLASSES
    err_code = ant_glasses_init(&m_ant_glasses, &ant_tx_channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = ant_glasses_open(&m_ant_glasses);
    APP_ERROR_CHECK(err_code);

/** @snippet [ANT HRM RX Profile Setup] */
}

/**@brief Function for initializing the Apple Notification Center Service.
*/
static void service_init(void)
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
    
    // NUS
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
        12,/*RX_PIN_NUMBER,*/
        15,/*TX_PIN_NUMBER,*/
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
    //bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);

    APP_ERROR_CHECK(err_code);

    //err_code = bsp_btn_ble_init(NULL, &startup_event);
    //APP_ERROR_CHECK(err_code);

    //*p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
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
    app_trace_init();
    
    timers_init();
    uart_init();
  
    buttons_leds_init(&erase_bonds);
    LEDS_OFF(LEDS_MASK);
    
    ble_stack_init();
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    
    device_manager_init(erase_bonds);
    db_discovery_init();
    scheduler_init();
    gap_params_init();
    service_init();
    advertising_init();
    conn_params_init();
	
    ant_profile_setup();
    
    // Start execution.
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    
    LEDS_OFF(LEDS_MASK);
    
    play_mario();
    
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

