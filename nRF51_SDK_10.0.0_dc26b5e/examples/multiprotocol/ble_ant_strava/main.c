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

#if (NRF_LOG_USES_RTT==1)
#include "SEGGER_RTT.h"
#endif

#include "ble_db_discovery.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble_hci.h"
#include "ble_hrs.h"

#include "ble_gap.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "nrf_gpio.h"
#include "ble_srv_common.h"
#include "ble_conn_params.h"
#include "ble_nus.h"
//#include "nrf51_bitfields.h"
#include "device_manager.h"
#include "app_button.h"
#include "app_timer.h"
#include "bsp.h"
#include "pstorage.h"
#include "softdevice_handler.h"
#include "app_uart.h"
#include "app_trace.h"
#include "nrf_delay.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "app_pwm.h"


#ifdef BLE_DFU_APP_SUPPORT
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#endif

#include "nrf_drv_wdt.h"

#include "ant_stack_config.h"
#include "ant_hrm.h"
#include "ant_bsc.h"
#include "ant_glasses.h"
#include "ant_interface.h"



#define TAILLE_BUFFER 30


// 30
#define GPIO_BUTTON                    30

#define IS_SRVC_CHANGED_CHARACT_PRESENT 1

#define HRM_RX_CHANNEL_NUMBER       0x00                        /**< Channel number assigned to HRM profile. */
#define BSC_RX_CHANNEL_NUMBER       0x01
#define GLASSES_TX_CHANNEL_NUMBER   0x02

#define BSC_DEVICE_TYPE             0x79

#define WILDCARD_TRANSMISSION_TYPE  0x00                        /**< Wildcard transmission type. */
#define WILDCARD_DEVICE_NUMBER      0x0000                        /**< Wildcard device number. */
#define HRM_DEVICE_NUMBER           0x0D22                        
#define BSC_DEVICE_NUMBER           0xB02B                      

#define ANTPLUS_NETWORK_NUMBER      0x00                        /**< Network number. */
#define HRMRX_NETWORK_KEY           {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45}    /**< The default network key used. */

#define UART_TX_BUF_SIZE                512                                        /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                32                                          /**< UART RX buffer size. */

#define ATTR_DATA_SIZE                  BLE_ANCS_ATTR_DATA_MAX                      /**< Allocated size for attribute data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define DEVICE_NAME                     "ANCS"                                      /**< Name of the device. Will be included in the advertising data. */
#define APP_ADV_FAST_INTERVAL           40                                          /**< The advertising interval (in units of 0.625 ms). The default value corresponds to 25 ms. */
#define APP_ADV_SLOW_INTERVAL           3200                                        /**< Slow advertising interval (in units of 0.625 ms). The default value corresponds to 2 seconds. */
#define APP_ADV_FAST_TIMEOUT            180                                         /**< The advertising time-out in units of seconds. */
#define APP_ADV_SLOW_TIMEOUT            180                                         /**< The advertising time-out in units of seconds. */
#define ADV_INTERVAL_FAST_PERIOD        30                                          /**< The duration of the fast advertising period (in seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
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
#define PLAY_DELAY                      APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)

#define SEC_PARAM_TIMEOUT               30                                           /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump. Can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE, \
                                            BLE_STACK_HANDLER_SCHED_EVT_SIZE) /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                25                                    /**< Maximum number of events in the scheduler queue. */

#ifdef BLE_DFU_APP_SUPPORT
#define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                    0x01                                       /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
#define APP_SERVICE_HANDLE_START         0x000C                                     /**< Handle of first application specific service when when service changed characteristic is present. */
#define BLE_HANDLE_MAX                   0xFFFF                                     /**< Max handle value in BLE. */

STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                                     /** When having DFU Service support in application the Service Changed Characteristic should always be present. */
#endif // BLE_DFU_APP_SUPPORT


#ifdef USE_TUNES
#include "tunes.h"

APP_PWM_INSTANCE(PWM1, 1);                   // Create the instance "PWM1" using TIMER1.
#endif

#ifdef TRACE_DEBUG
#define LOG printf
#else
#define LOG(...)
#endif 


static volatile bool ready_flag;            // A flag indicating PWM status.


static uint16_t                  m_conn_handle = BLE_CONN_HANDLE_INVALID;  /**< Handle of the current connection. */

//static ble_db_discovery_t        m_ble_db_discovery;                       /**< Structure used to identify the DB Discovery module. */

static dm_application_instance_t m_app_handle;                             /**< Application identifier allocated by the Device Manager. */
static dm_handle_t               m_peer_handle;                            /**< Identifies the peer that is currently connected. */

static uint8_t nus_isinit = 0;
static ble_nus_t                 m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static ble_hrs_t                 m_hrs;                                      /**< Structure used to identify the heart rate service. */

static ble_uuid_t m_adv_uuids1[] = {{BLE_UUID_HEART_RATE_SERVICE,         BLE_UUID_TYPE_BLE},
                                    {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
                                    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */

static ble_uuid_t m_adv_uuids2[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */
           
                                   
                                   
APP_TIMER_DEF(m_sec_req_timer_id);                       /**< Security request timer. The timer lets us start pairing request if one does not arrive from the Central. */
APP_TIMER_DEF(m_sec_hrm);
APP_TIMER_DEF(m_sec_cad);
#ifdef USE_TUNES
APP_TIMER_DEF(m_sec_play);
APP_TIMER_DEF(m_sec_tune);
#endif

#ifdef BLE_DFU_APP_SUPPORT    
static ble_dfu_t                 m_dfus;                                    /**< Structure used to identify the DFU service. */
#endif // BLE_DFU_APP_SUPPORT  


static uint8_t m_network_key[] = HRMRX_NETWORK_KEY;             /**< ANT PLUS network key. */

/** @snippet [ANT BSC RX Instance] */
#define WHEEL_CIRCUMFERENCE         2070                                                            /**< Bike wheel circumference [mm] */
#define BSC_EVT_TIME_FACTOR         1024                                                            /**< Time unit factor for BSC events */
#define BSC_RPM_TIME_FACTOR         60                                                              /**< Time unit factor for RPM unit */
#define BSC_MS_TO_KPH_NUM           36                                                              /**< Numerator of [m/s] to [kph] ratio */
#define BSC_MS_TO_KPH_DEN           10                                                              /**< Denominator of [m/s] to [kph] ratio */
#define BSC_MM_TO_M_FACTOR          1000                                                            /**< Unit factor [m/s] to [mm/s] */
#define BSC_SPEED_UNIT_FACTOR       (BSC_MS_TO_KPH_DEN * BSC_MM_TO_M_FACTOR)                        /**< Speed unit factor */
#define SPEED_COEFFICIENT           (WHEEL_CIRCUMFERENCE * BSC_EVT_TIME_FACTOR * BSC_MS_TO_KPH_NUM) /**< Coefficient for speed value calculation */
#define CADENCE_COEFFICIENT         (BSC_EVT_TIME_FACTOR * BSC_RPM_TIME_FACTOR)                     /**< Coefficient for cadence value calculation */

void ant_bsc_evt_handler(ant_bsc_profile_t * p_profile, ant_bsc_evt_t event);
BSC_DISP_CHANNEL_CONFIG_DEF(m_ant_bsc,
                            BSC_RX_CHANNEL_NUMBER,
                            WILDCARD_TRANSMISSION_TYPE,
                            BSC_DEVICE_TYPE,
                            BSC_DEVICE_NUMBER,
                            ANTPLUS_NETWORK_NUMBER,
                            BSC_MSG_PERIOD_4Hz);
BSC_DISP_PROFILE_CONFIG_DEF(m_ant_bsc, ant_bsc_evt_handler);
ant_bsc_profile_t m_ant_bsc;

static int32_t accumulated_s_rev_cnt, previous_s_evt_cnt, prev_s_accumulated_rev_cnt,
               accumulated_s_evt_time, previous_s_evt_time, prev_s_accumulated_evt_time = 0;

static int32_t accumulated_c_rev_cnt, previous_c_evt_cnt, prev_c_accumulated_rev_cnt,
               accumulated_c_evt_time, previous_c_evt_time, prev_c_accumulated_evt_time = 0;


/** @snippet [ANT HRM RX Instance] */
HRM_DISP_CHANNEL_CONFIG_DEF(m_ant_hrm,
                            HRM_RX_CHANNEL_NUMBER,
                            WILDCARD_TRANSMISSION_TYPE,
                            HRM_DEVICE_NUMBER,
                            ANTPLUS_NETWORK_NUMBER,
                            HRM_MSG_PERIOD_4Hz);
ant_hrm_profile_t           m_ant_hrm;

/** @snippet [ANT GLASSES TX Instance] */
ant_glasses_profile_t       m_ant_glasses;

const ant_channel_config_t  ant_tx_channel_config  = GLASSES_TX_CHANNEL_CONFIG(GLASSES_TX_CHANNEL_NUMBER, 5,
                                                    GLASSES_DEVICE_NUMBER, ANTPLUS_NETWORK_NUMBER);

nrf_drv_wdt_channel_id wdt_channel_id;

static uint8_t is_hrm_init = 0;
static uint8_t is_cad_init = 0;


#ifdef USE_TUNES
static uint8_t isTunePlaying = 0;
static uint8_t whichTune = 0;
static uint32_t iTune = 0;
static uint8_t pwm_ready = 0;
void play_mario(void * p_context);

static void pwm_start(uint32_t period_us);
static void play_tune(void);
#endif

#if (LEDS_NUMBER > 0)
const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;
#else
const uint8_t leds_list;
#endif

#define RB_SIZE 100
static uint8_t read_byte[RB_SIZE];
static uint8_t glasses_payload[8];
static uint16_t status_byte = 0;
static uint16_t marque_byte = 0;

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t *p_file_name) {
  
  if (error_code == NRF_SUCCESS) return;
  
#if (NRF_LOG_USES_RTT==1)
  SEGGER_RTT_printf(0, "Erreur: %u ligne%u %s!!\n", (unsigned int)error_code, (unsigned int)line_num, p_file_name); 
#endif
	
#ifdef DEBUG
	sprintf((char *)read_byte, "$ERR,%u,%u\n\r", (unsigned int)error_code, (unsigned int)line_num);
	
	read_byte[15] = '\0';
	read_byte[16] = '\0';
	if (nus_isinit) {
	uint32_t err_code = ble_nus_string_send(&m_nus, read_byte, 15);
       if (err_code != NRF_ERROR_INVALID_STATE)
       {
         APP_ERROR_CHECK(err_code);
       }
	}
#endif
  
  if (p_file_name) {
    printf("$DBG,1,%u,%u,%s\n\r", (unsigned int)error_code, (unsigned int)line_num, p_file_name);
  } else {
    printf("$DBG,0,%u,%u\n\r", (unsigned int)error_code, (unsigned int)line_num);
  }

#if LEDS_NUMBER > 0
  LEDS_OFF(LEDS_MASK);
  while (0) {
    nrf_delay_ms(300);
    LEDS_INVERT(LEDS_MASK);
  }
#endif
}

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

#ifdef USE_TUNES
static void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    pwm_ready = 1;
}



static void sec_tune(void * p_context) {
   play_tune();
}
#endif

uint8_t encode (uint8_t byte) {
  
  uint32_t err_code;
  
   switch (byte) {
     case '$':
       status_byte = 0;
       marque_byte = 1;
       memset(read_byte, 0, RB_SIZE);
     
       break;
     case '\r':
		 case '\n':
		 case '\0':
			 if (status_byte < 1 || marque_byte == 0) return 0;
       LOG("%s\n\r", read_byte);
#ifdef DEBUG
		   if (nus_isinit) {
				 err_code = ble_nus_string_send(&m_nus, read_byte, status_byte);
				 if (err_code != NRF_ERROR_INVALID_STATE)
				 {
					 APP_ERROR_CHECK(err_code);
				 }     
       }			 
#endif			 
#ifdef USE_TUNES
		   if (read_byte[0]=='T' && read_byte[1]=='U') {
				 switch(read_byte[2]) {
				   case '0':
             whichTune = 0;
						 err_code = app_timer_start(m_sec_play, PLAY_DELAY, NULL);
						 break;
					 case '1':
             whichTune = 1;
             err_code = app_timer_start(m_sec_play, PLAY_DELAY, NULL);
						 break;
				 }
         
         status_byte = 0;
         marque_byte = 0;
				 return 0;
			 }
#endif
       marque_byte = 0;
       status_byte = 0;
       return 1;
       
     default:
       if (status_byte < RB_SIZE - 10 && marque_byte == 1) {
         read_byte[status_byte] = byte;
         status_byte++;
       } else {
         status_byte = 0;
         marque_byte = 0;
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


void uart_evt_handle(app_uart_evt_t * p_event)
{
    uint8_t uart_byte = 0;
  
    if (p_event->evt_type == APP_UART_DATA_READY)
    {
      
#ifdef DEBUG
			if (nus_isinit)
			  ble_nus_string_send(&m_nus, "Rd", 2);
#endif
      
      // get data
      while(app_uart_get(&uart_byte) != NRF_SUCCESS) {;}
	  
	  
      if (encode(uart_byte)) {
        transmit_order ();
      }

    }
    
  
    if (1) {
      if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
      {
        APP_ERROR_CHECK(p_event->data.error_communication);
      }
      else if (p_event->evt_type == APP_UART_FIFO_ERROR)
      {
        APP_ERROR_CHECK(p_event->data.error_code);
      }
      else if (p_event->evt_type == APP_UART_TX_EMPTY)
      {
        
      }
    }
}

static void hrm_connect(void * p_context)
{
  uint32_t err_code = NRF_SUCCESS;
  
  err_code = ant_hrm_disp_open(&m_ant_hrm);
  APP_ERROR_CHECK(err_code);
}

static void cad_connect(void * p_context)
{
  uint32_t err_code = NRF_SUCCESS;
  
  err_code = ant_bsc_disp_open(&m_ant_bsc);
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
                    sd_ant_channel_id_get (BSC_RX_CHANNEL_NUMBER, 
                          &pusDeviceNumber, &pucDeviceType, &pucTransmitType);
                    printf("$ANCS,0,CAD 0x%x connected\n\r", pusDeviceNumber);
                    if (pusDeviceNumber) is_cad_init = 1;
                  }
                  ant_bsc_disp_evt_handler(&m_ant_bsc, p_ant_evt);
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
                  ant_hrm_disp_evt_handler(&m_ant_hrm, p_ant_evt);
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
				break;
      
			case BSC_RX_CHANNEL_NUMBER:
				ant_evt_cad (p_ant_evt);
				break;
      
      case GLASSES_TX_CHANNEL_NUMBER:
        ant_evt_glasses (p_ant_evt);
        break;
      
			default:
				break;
		}

#if LEDS_NUMBER > 0
    LEDS_INVERT(BSP_LED_2_MASK);
#endif
}



__STATIC_INLINE float calculate_speed(int32_t rev_cnt, int32_t evt_time)
{
    static float computed_speed   = 0;

    if (rev_cnt != previous_s_evt_cnt)
    {
        accumulated_s_rev_cnt  += rev_cnt - previous_s_evt_cnt;
        accumulated_s_evt_time += evt_time - previous_s_evt_time;

        /* Process rollover */
        if (previous_s_evt_cnt > rev_cnt)
        {
            accumulated_s_rev_cnt += UINT16_MAX + 1;
        }
        if (previous_s_evt_time > evt_time)
        {
            accumulated_s_evt_time += UINT16_MAX + 1;
        }

        previous_s_evt_cnt  = rev_cnt;
        previous_s_evt_time = evt_time;

        computed_speed   = SPEED_COEFFICIENT *
                           (accumulated_s_rev_cnt  - prev_s_accumulated_rev_cnt) /
                           (accumulated_s_evt_time - prev_s_accumulated_evt_time)/
                           BSC_SPEED_UNIT_FACTOR;

        prev_s_accumulated_rev_cnt  = accumulated_s_rev_cnt;
        prev_s_accumulated_evt_time = accumulated_s_evt_time;
    }

    return (uint32_t) computed_speed;
}

static uint32_t calculate_cadence(int32_t rev_cnt, int32_t evt_time)
{
    static uint32_t computed_cadence = 0;

    if (rev_cnt != previous_c_evt_cnt)
    {
        accumulated_c_rev_cnt  += rev_cnt - previous_c_evt_cnt;
        accumulated_c_evt_time += evt_time - previous_c_evt_time;

        /* Process rollover */
        if (previous_c_evt_cnt > rev_cnt)
        {
            accumulated_c_rev_cnt += UINT16_MAX + 1;
        }
        if (previous_c_evt_time > evt_time)
        {
            accumulated_c_evt_time += UINT16_MAX + 1;
        }

        previous_c_evt_cnt  = rev_cnt;
        previous_c_evt_time = evt_time;

        computed_cadence = CADENCE_COEFFICIENT *
                           (accumulated_c_rev_cnt  - prev_c_accumulated_rev_cnt) /
                           (accumulated_c_evt_time - prev_c_accumulated_evt_time);

        prev_c_accumulated_rev_cnt  = accumulated_c_rev_cnt;
        prev_c_accumulated_evt_time = accumulated_c_evt_time;
    }

    return (uint32_t) computed_cadence;
}


void ant_bsc_evt_handler(ant_bsc_profile_t * p_profile, ant_bsc_evt_t event)
{
    unsigned int _cadence;
	  float _speed;
  
  
    switch (event)
    {
        case ANT_BSC_PAGE_0_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_1_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_2_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_3_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_4_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_5_UPDATED:
            /* Log computed value */
            break;

        case ANT_BSC_COMB_PAGE_0_UPDATED:
          
            _speed = calculate_speed(p_profile->BSC_PROFILE_speed_rev_count, p_profile->BSC_PROFILE_speed_event_time);
        
            _cadence = calculate_cadence(p_profile->BSC_PROFILE_cadence_rev_count, p_profile->BSC_PROFILE_cadence_event_time);
                         
			printf("$CAD,%u,%u\n\r", (unsigned int)_cadence, (unsigned int)(_speed * 100.));
				
#if (NRF_LOG_USES_RTT==1)
        log_rtt_printf(0, "Evenement BSC speed=%u cad=%u\n", _speed, _cadence);
#endif
            break;

        default:
            break;
    }
}


/**@brief Handle received ANT+ HRM data.
 * 
 * @param[in]   p_profile       Pointer to the ANT+ HRM profile instance.
 * @param[in]   event           Event related with ANT+ HRM Display profile. 
 */
static void ant_hrm_evt_handler(ant_hrm_profile_t * p_profile, ant_hrm_evt_t event)
{
	  uint32_t            err_code;
    static uint32_t     s_previous_beat_count  = 0;    // Heart beat count from previously received page
    uint16_t            beat_time              = p_profile->page_0.beat_time;
    uint32_t            beat_count             = p_profile->page_0.beat_count;
    uint32_t            computed_heart_rate    = p_profile->page_0.computed_heart_rate;
	  uint16_t rrInterval;
    uint16_t rrInterval_ms;
  
    switch (event)
    {
        case ANT_HRM_PAGE_0_UPDATED:
            /* fall through */
        case ANT_HRM_PAGE_1_UPDATED:
            /* fall through */
        case ANT_HRM_PAGE_2_UPDATED:
            /* fall through */
        case ANT_HRM_PAGE_3_UPDATED:
            break;
        case ANT_HRM_PAGE_4_UPDATED:
					
				    // Notify the received heart rate measurement
						err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, computed_heart_rate);
						if (
								(err_code != NRF_SUCCESS)
								&&
								(err_code != NRF_ERROR_INVALID_STATE)
								&&
								(err_code != NRF_ERROR_NO_MEM)
								&&
								(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
						)
						{
								APP_ERROR_HANDLER(err_code);
						}
          
#if (NRF_LOG_USES_RTT==1)
            log_rtt_printf(0, "Evenement HR BPM=%u\n", (unsigned int)computed_heart_rate);
#endif
        
            // Ensure that there is only one beat between time intervals.
            if ((beat_count - s_previous_beat_count) == 1)
            {
                uint16_t prev_beat = p_profile->page_4.prev_beat;
							
							  rrInterval = (beat_time - prev_beat);
                rrInterval_ms = rrInterval * 1000. / 1024.;
							
							  printf("$HRM,%u,%u\n\r",
                      (unsigned int)computed_heart_rate,
                      (unsigned int)rrInterval_ms);
                
                // Subtracting the event time gives the R-R interval
                //ble_hrs_rr_interval_add(&m_hrs, beat_time - prev_beat);
#if (NRF_LOG_USES_RTT==1)
                log_rtt_printf(0, "Evenement HR RR=%u\n", (unsigned int)rrInterval_ms);
#endif
											
								ble_hrs_rr_interval_add(&m_hrs, rrInterval_ms);
								if ((err_code != NRF_SUCCESS)
										&&
										(err_code != NRF_ERROR_INVALID_STATE)
										&&
										(err_code != NRF_ERROR_NO_MEM)
										&&
										(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
								{
										APP_ERROR_HANDLER(err_code);
								}
            }
            
            s_previous_beat_count = beat_count;
            break;

        default:
            break;
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
  
    err_code = app_timer_create(&m_sec_hrm, APP_TIMER_MODE_SINGLE_SHOT, hrm_connect);
    APP_ERROR_CHECK(err_code);
  
    err_code = app_timer_create(&m_sec_cad, APP_TIMER_MODE_SINGLE_SHOT, cad_connect);
    APP_ERROR_CHECK(err_code);
  
#ifdef USE_TUNES
    err_code = app_timer_create(&m_sec_play, APP_TIMER_MODE_SINGLE_SHOT, play_mario);
    APP_ERROR_CHECK(err_code);
  
    err_code = app_timer_create(&m_sec_tune, APP_TIMER_MODE_SINGLE_SHOT, sec_tune);
    APP_ERROR_CHECK(err_code);
#endif
}



#ifdef BLE_DFU_APP_SUPPORT
/**@brief Function for stopping advertising.
 */
static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for loading application-specific context after establishing a secure connection.
 *
 * @details This function will load the application context and check if the ATT table is marked as 
 *          changed. If the ATT table is marked as changed, a Service Changed Indication
 *          is sent to the peer if the Service Changed CCCD is set to indicate.
 *
 * @param[in] p_handle The Device Manager handle that identifies the connection for which the context 
 *                     should be loaded.
 */
static void app_context_load(dm_handle_t const * p_handle)
{
    uint32_t                 err_code;
    static uint32_t          context_data;
    dm_application_context_t context;

    context.len    = sizeof(context_data);
    context.p_data = (uint8_t *)&context_data;

    err_code = dm_application_context_get(p_handle, &context);
    if (err_code == NRF_SUCCESS)
    {
        // Send Service Changed Indication if ATT table has changed.
        if ((context_data & (DFU_APP_ATT_TABLE_CHANGED << DFU_APP_ATT_TABLE_POS)) != 0)
        {
            err_code = sd_ble_gatts_service_changed(m_conn_handle, APP_SERVICE_HANDLE_START, BLE_HANDLE_MAX);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
            {
                APP_ERROR_HANDLER(err_code);
            }
        }

        err_code = dm_application_context_delete(p_handle);
        APP_ERROR_CHECK(err_code);
    }
    else if (err_code == DM_NO_APP_CONTEXT)
    {
        // No context available. Ignore.
    }
    else
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/** @snippet [DFU BLE Reset prepare] */
/**@brief Function for preparing for system reset.
 *
 * @details This function implements @ref dfu_app_reset_prepare_t. It will be called by 
 *          @ref dfu_app_handler.c before entering the bootloader/DFU.
 *          This allows the current running application to shut down gracefully.
 */
static void reset_prepare(void)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);
    
    sd_power_gpregret_set(BOOTLOADER_DFU_START);

    nrf_delay_ms(500);
}
/** @snippet [DFU BLE Reset prepare] */
#endif // BLE_DFU_APP_SUPPORT


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
                                          
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
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
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
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
    app_error_handler(event_result, 0, (uint8_t*)"device_mgr");

    switch (p_evt->event_id)
    {
        case DM_EVT_CONNECTION:
            m_peer_handle = (*p_handle);
            err_code      = app_timer_start(m_sec_req_timer_id, SECURITY_REQUEST_DELAY, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case DM_EVT_LINK_SECURED:
          
#ifdef BLE_DFU_APP_SUPPORT
            app_context_load(p_handle);
#endif
            
            //err_code = ble_db_discovery_start(&m_ble_db_discovery, p_evt->event_param.p_gap_param->conn_handle);
            //APP_ERROR_CHECK(err_code);
            break; 

        default:
            break;

    }
    return NRF_SUCCESS;
}


/**@brief Function for initializing the Device Manager.
 *
 * @param[in] p_erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool p_erase_bonds)
{
    uint32_t                err_code;
    dm_init_param_t         init_data = {.clear_persistent_data = true};
    dm_application_param_t  register_param;
    
    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_data);
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
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST:
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW:
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            //APP_ERROR_CHECK(err_code);
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
            

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
				
#if LEDS_NUMBER > 0
            LEDS_OFF(BSP_LED_1_MASK);
            uint8_t var = 20;
            LEDS_OFF(LEDS_MASK);
            while (var--) {
              nrf_delay_ms(200);
              LEDS_INVERT(LEDS_MASK);
            }
#else
            nrf_delay_ms(100);
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
    app_error_handler(err_code, 0, (uint8_t*)"on_ble_evt");
}




/**@brief Function for handling bsp events.
 */
void bsp_evt_handler(bsp_event_t evt)
{
    uint32_t err_code = NRF_SUCCESS;
  
#if (NRF_LOG_USES_RTT==1)
  SEGGER_RTT_printf(0, "Event: %u\n", evt); 
#endif

  
    switch (evt)
    {
        case BSP_EVENT_KEY_0:
            printf("$BTN,0\n\r");
            break;
        case BSP_EVENT_KEY_1:
            printf("$BTN,1\n\r");
            break;
        case BSP_EVENT_KEY_2:
            printf("$BTN,2\n\r");
            break;
        case BSP_EVENT_KEY_3:
            whichTune = 0;
						err_code = app_timer_start(m_sec_play, PLAY_DELAY, NULL);
            break;
        case BSP_EVENT_KEY_4:
            whichTune = 0;
						err_code = app_timer_start(m_sec_play, PLAY_DELAY, NULL);
            break;
        default:
            return; // no implementation needed
    }

}

#ifdef USE_TUNES
static void pwm_start(uint32_t period_us) {
  
    /* 2-channel PWM, 200Hz, output on DK LED pins. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(period_us, 24, 0);
    
    /* Switch the polarity of the second channel. */
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
  
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

void play_mario(void * p_context) {
   
   iTune = NOTES_NB;
#ifdef DEBUG
	 ble_nus_string_send(&m_nus, "Musique !", 9);
#endif
	 if (isTunePlaying) {
		 isTunePlaying = 0;
		 pwm_stop();
		 nrf_delay_ms(10);
	 }
#ifndef DEBUG
   play_tune ();
#endif
}

static void play_tune (void) {
  
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
#endif

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
    //ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    
#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [Propagating BLE Stack events to DFU Service] */
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
#endif // BLE_DFU_APP_SUPPORT
  
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
	  ble_hrs_on_ble_evt(&m_hrs, p_ble_evt);
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
  
    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));

    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;

    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;

    
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for System events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);

}



/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ant_stack_init(void)
{
    uint32_t err_code;
  
    err_code = softdevice_ant_evt_handler_set(ant_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    //
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
    err_code = ant_hrm_disp_init(&m_ant_hrm,
                                 HRM_DISP_CHANNEL_CONFIG(m_ant_hrm),
                                 ant_hrm_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = ant_hrm_disp_open(&m_ant_hrm);
    APP_ERROR_CHECK(err_code);

    
    // CAD
    err_code = ant_bsc_disp_init(&m_ant_bsc,
                                 BSC_DISP_CHANNEL_CONFIG(m_ant_bsc),
                                 BSC_DISP_PROFILE_CONFIG(m_ant_bsc));

    err_code = ant_bsc_disp_open(&m_ant_bsc);
    APP_ERROR_CHECK(err_code);
  
  
    // GLASSES
    err_code = ant_glasses_init(&m_ant_glasses, &ant_tx_channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = ant_glasses_open(&m_ant_glasses);
    APP_ERROR_CHECK(err_code);

/** @snippet [ANT HRM RX Profile Setup] */
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
  
#if LEDS_NUMBER > 0
  LEDS_INVERT(LEDS_MASK);
#endif
  
  for (uint32_t i = 0; i < length; i++)
    {
      encode (p_data[i]);
			while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
		encode ('\n');
    while(app_uart_put('\n') != NRF_SUCCESS);

}

/**@brief Function for initializing the services
*/
static void service_init(void)
{
    uint32_t          err_code;
    ble_nus_init_t nus_init;
	  ble_hrs_init_t hrs_init;
	  uint8_t        body_sensor_location;
	
	  // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_CHEST;
	
	  memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_hrm_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_bsl_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_bsl_attr_md.write_perm);

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
    
#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [DFU BLE Service initialization] */
    ble_dfu_init_t   dfus_init;

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.error_handler = NULL;
    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.revision      = DFU_REVISION;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);

    dfu_app_reset_prepare_set(reset_prepare);
    dfu_app_dm_appl_instance_set(m_app_handle);
    /** @snippet [DFU BLE Service initialization] */
#endif // BLE_DFU_APP_SUPPORT
    
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
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids1) / sizeof(m_adv_uuids1[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids1;
  
    advdata.uuids_solicited.uuid_cnt = sizeof(m_adv_uuids2) / sizeof(m_adv_uuids2[0]);
    advdata.uuids_solicited.p_uuids  = m_adv_uuids2;

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
                       uart_evt_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing buttons and LEDs.
 *
 * @param[out] p_erase_bonds  True if the clear bonds button was pressed to wake the application up.
 */
void buttons_leds_init(bool * p_erase_bonds)
{
    nrf_gpio_cfg_output(GPIO_BUTTON);
    // pin to ground
    nrf_gpio_pin_clear(GPIO_BUTTON);
  
    // BSP_INIT_BUTTONS | BSP_INIT_LED
    uint32_t err_code = bsp_init(BSP_INIT_BUTTONS | BSP_INIT_LED,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_evt_handler);
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
    //uint32_t err_code = ble_db_discovery_init();
    //APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief WDT events handler.
 */
void wdt_event_handler(void)
{
#if LEDS_NUMBER > 0
    LEDS_INVERT(LEDS_MASK);
#endif
}


/**@brief Function for application main entry.
 */
int main(void)
{
    bool     erase_bonds = true;
    uint32_t err_code;
  
#if (NRF_LOG_USES_RTT==1)
    SEGGER_RTT_printf(0, "Start !!\n"); 
#endif
  
    //Configure WDT.
#if (WDT_ENABLED == 1)
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&wdt_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
#endif
  
    timers_init();
    uart_init();
    buttons_leds_init(&erase_bonds);
  
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // BLE init
#ifdef BLE_STACK_SUPPORT_REQD
    ble_stack_init();
    device_manager_init(erase_bonds);
    db_discovery_init();
    scheduler_init();
    gap_params_init();
    service_init();
    advertising_init();
    conn_params_init();
#endif

	
    // ANT init
    ant_stack_init();
    ant_profile_setup();
    
    // Start execution.
#ifdef BLE_STACK_SUPPORT_REQD
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
#endif
    
		nus_isinit = 1;

#if LEDS_NUMBER > 0
    LEDS_OFF(LEDS_MASK);
#endif

    // Enter main loop.
    for (;;)
    {
        // power manage
        app_sched_execute();
        power_manage();
        
        // WDT feed
#if (WDT_ENABLED == 1)
        nrf_drv_wdt_channel_feed(wdt_channel_id);
#endif
    }

}


/**
 * @}
 */

