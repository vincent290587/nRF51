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

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
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

#include "ant_stack_config.h"
#include "ant_glasses.h"
#include "ant_interface.h"
#include "ant_state_indicator.h"


#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            10                  /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         5                                           /**< Size of timer operation queues. */

#define ANT_DELAY                      APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)

// Multiple ANT channel settings
#define ANT_STACK_TOTAL_CHANNELS_ALLOCATED  2               // (SIZE_OF_NONENCRYPTED_ANT_CHANNEL bytes)
#define ANT_STACK_ENCRYPTED_CHANNELS        0               // (SIZE_OF_ENCRYPTED_ANT_CHANNEL bytes)
#define ANT_STACK_TX_BURST_QUEUE_SIZE       128             // (128 bytes)

#define GLASSES_RX_CHANNEL_NUMBER   0x00

#define WILDCARD_TRANSMISSION_TYPE  0x00                        /**< Wildcard transmission type. */
#define WILDCARD_DEVICE_NUMBER      0x0000                        /**< Wildcard device number. */                 

#define ANTPLUS_NETWORK_NUMBER      0x00                        /**< Network number. */
#define HRMRX_NETWORK_KEY           {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45}    /**< The default network key used. */

#define UART_TX_BUF_SIZE                1024                                        /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                1                                          /**< UART RX buffer size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump. Can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE, \
                                            BLE_STACK_HANDLER_SCHED_EVT_SIZE) /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                10                                    /**< Maximum number of events in the scheduler queue. */


#ifdef TRACE_DEBUG
#define LOG printf
#else
#define LOG(...)
#endif // TRACE_HRM_GENERAL_ENABLE 

static uint8_t m_network_key[] = HRMRX_NETWORK_KEY;             /**< ANT PLUS network key. */

// timer
static app_timer_id_t       m_sec_glasses;
static app_timer_id_t       m_sec_leds;

/** @snippet [ANT HRM RX Instance] */
ant_glasses_profile_t       m_ant_glasses;

const ant_channel_config_t  ant_rx_channel_config  = GLASSES_RX_CHANNEL_CONFIG(GLASSES_RX_CHANNEL_NUMBER, WILDCARD_TRANSMISSION_TYPE,
                                                    GLASSES_DEVICE_NUMBER, ANTPLUS_NETWORK_NUMBER);

uint8_t is_glasses_init = 0;

#if (LEDS_NUMBER > 0)
const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;
#else
const uint8_t leds_list;
#endif


static uint32_t led_period;
static uint8_t led_mask;
static uint8_t is_led_off = 0;


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
    if (0) {
      if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
      {
        APP_ERROR_HANDLER(p_event->data.error_communication);
      }
      else if (p_event->evt_type == APP_UART_FIFO_ERROR)
      {
        APP_ERROR_HANDLER(p_event->data.error_code);
      } else if (p_event->evt_type == APP_UART_DATA_READY)
      {
  
      }
    }
}

float min(float val1, float val2) {
  if (val1 <= val2) return val1;
  else return val2;
}

float max(float val1, float val2) {
  if (val1 <= val2) return val2;
  else return val1;
}

float regFenLim(float val_, float b1_i, float b1_f, float b2_i, float b2_f) {
  
  float x, res;
  // calcul x
  x = (val_ - b1_i) / (b1_f - b1_i);
  
  // calcul valeur: x commun
  res = x * (b2_f - b2_i) + b2_i;
  if (res < min(b2_i,b2_f)) res = min(b2_i,b2_f);
  if (res > max(b2_i,b2_f)) res = max(b2_i,b2_f);
  return res;
}

void action_reception(ant_glasses_trans *trans)
{
  uint32_t err_code = NRF_SUCCESS;
  
  
  LOG("Action %d %f\n\r", trans->led_mask, trans->avance);

  led_mask = 1<<leds_list[trans->led_mask];
  
  if (trans->avance < 20.) {
    is_led_off = 0;
    led_period = regFenLim(trans->avance, 2., 20., 100., 2000.);
  } else {
    is_led_off = 1;
  }
  
  
}


void ant_evt_glasses (ant_evt_t * p_ant_evt)
{
    uint32_t err_code = NRF_SUCCESS;
    ant_glasses_trans trans;
  
    memset(&trans, 0, sizeof(ant_glasses_trans));
    
    switch (p_ant_evt->event)
					{
							case EVENT_TX:
                  break;
              case EVENT_RX:
                  ant_glasses_rx_evt_handle(&m_ant_glasses, p_ant_evt, &trans);
                  action_reception(&trans);
                  //LEDS_INVERT(BSP_LED_0_MASK);
									break;
							case EVENT_RX_FAIL:
							case EVENT_RX_FAIL_GO_TO_SEARCH:
                  //LEDS_INVERT(BSP_LED_2_MASK);
                  LOG("RX fail\n\r");
									break;
              case EVENT_RX_SEARCH_TIMEOUT:
                  break;
							case EVENT_CHANNEL_CLOSED:
                  LOG("Reconnexion...\n\r");
                  is_glasses_init = 0;
									//err_code = app_timer_stop(m_sec_glasses);
                  LEDS_ON(LEDS_MASK);
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
      case GLASSES_RX_CHANNEL_NUMBER:
        ant_evt_glasses (p_ant_evt);
        break;
      
			default:
				break;
		}

}

static void glasses_connect(void * p_context)
{
  uint32_t err_code = NRF_SUCCESS;
  
  err_code = ant_glasses_open(&m_ant_glasses);
  APP_ERROR_CHECK(err_code);
}

static void leds_blink ()
{
  uint32_t err_code = NRF_SUCCESS;
  uint32_t delay;
  
  // temps led eteinte
  delay = regFenLim(led_period, 0., 2500., 50., 250.);
  
  LEDS_ON(LEDS_MASK);
  if (is_led_off == 0) {
    LEDS_OFF(led_mask);
    nrf_delay_ms(delay);
    LEDS_ON(led_mask);
  }
  
  LOG("LED period= %d\n\r", led_period);
  
  err_code = app_timer_start(m_sec_leds, APP_TIMER_TICKS(led_period, APP_TIMER_PRESCALER), NULL);
  
  return;
}


/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    uint32_t err_code;
  
    // Initialize timer module, making it use the scheduler.
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);
  
    err_code = app_timer_create(&m_sec_glasses, APP_TIMER_MODE_SINGLE_SHOT, glasses_connect);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sec_leds, APP_TIMER_MODE_SINGLE_SHOT, leds_blink);
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
            break;

        default:
            break;
    }
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
}



/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

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
  
    // GLASSES
    err_code = ant_glasses_init(&m_ant_glasses, &ant_rx_channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = ant_glasses_open(&m_ant_glasses);
    APP_ERROR_CHECK(err_code);

/** @snippet [ANT HRM RX Profile Setup] */
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
static void buttons_leds_init(void)
{
    //bsp_event_t startup_event;
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
  uint32_t err_code;
  
    LEDS_OFF(LEDS_MASK);
    // Initialize.
    app_trace_init();
    timers_init();
    uart_init();
    buttons_leds_init();
    
    stack_init();
  
    scheduler_init();
    
    
    LOG("Reboot\n");
  
    is_led_off = 1;
    led_period = 2000;
    err_code = app_timer_start(m_sec_leds, APP_TIMER_TICKS(led_period, APP_TIMER_PRESCALER), NULL);
    
    ant_profile_setup();
  
    LEDS_ON(LEDS_MASK);
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

