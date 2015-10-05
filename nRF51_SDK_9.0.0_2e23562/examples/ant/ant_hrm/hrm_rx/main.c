/*
This software is subject to the license described in the license.txt file included with this software distribution. 
You may not use this file except in compliance with this license. 
Copyright © Dynastream Innovations Inc. 2015
All rights reserved.
*/

/**@file
 * @defgroup ant_hrm_rx_example ANT HRM RX example
 * @{
 * @ingroup nrf_ant_hrm
 *
 * @brief Example of ANT HRM RX profile.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_error.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_delay.h"
#include "nrf_sdm.h"
#include "nrf_gpio.h"
#include "bsp.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "ant_stack_config.h"
#include "softdevice_handler.h"
#include "ant_hrm.h"
#include "app_trace.h"
#include "ant_interface.h"
#include "ant_state_indicator.h"

// Multiple ANT channel settings
#define ANT_STACK_TOTAL_CHANNELS_ALLOCATED  2               // (SIZE_OF_NONENCRYPTED_ANT_CHANNEL bytes)
#define ANT_STACK_ENCRYPTED_CHANNELS        0               // (SIZE_OF_ENCRYPTED_ANT_CHANNEL bytes)
#define ANT_STACK_TX_BURST_QUEUE_SIZE       128             // (128 bytes)


#define APP_TIMER_PRESCALER         0x00                        /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS        BSP_APP_TIMERS_NUMBER       /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE     0x04                        /**< Size of timer operation queues. */

#define HRM_RX_CHANNEL_NUMBER       0x00                        /**< Channel number assigned to HRM profile. */
#define CAD_RX_CHANNEL_NUMBER       0x01

#define WILDCARD_TRANSMISSION_TYPE  0x00                        /**< Wildcard transmission type. */
#define WILDCARD_DEVICE_NUMBER      0x00                        /**< Wildcard device number. */

#define ANTPLUS_NETWORK_NUMBER      0x00                        /**< Network number. */
#define HRMRX_NETWORK_KEY           {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45}    /**< The default network key used. */

static uint8_t m_network_key[] = HRMRX_NETWORK_KEY;             /**< ANT PLUS network key. */

/** @snippet [ANT HRM RX Instance] */
ant_hrm_profile_t           m_ant_hrm, m_ant_cad;
const ant_channel_config_t  ant_rx_channel_config = HRM_RX_CHANNEL_CONFIG(HRM_RX_CHANNEL_NUMBER, WILDCARD_TRANSMISSION_TYPE,
                                                    WILDCARD_DEVICE_NUMBER, ANTPLUS_NETWORK_NUMBER, HRM_MSG_PERIOD_4Hz);

const ant_channel_config_t  ant_rx_channel_config2 = CAD_RX_CHANNEL_CONFIG(CAD_RX_CHANNEL_NUMBER, WILDCARD_TRANSMISSION_TYPE,
                                                    WILDCARD_DEVICE_NUMBER, ANTPLUS_NETWORK_NUMBER, 0x1F96);
/** @snippet [ANT HRM RX Instance] */

const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;


	static uint8_t i = 0;
/**@brief Function for dispatching a ANT stack event to all modules with a ANT stack event handler.
 *
 * @details This function is called from the ANT Stack event interrupt handler after a ANT stack
 *          event has been received.
 *
 * @param[in] p_ant_evt  ANT stack event.
 */
void ant_evt_dispatch(ant_evt_t * p_ant_evt)
{
	  ant_hrm_rx_evt_handle(&m_ant_cad, p_ant_evt);
    ant_hrm_rx_evt_handle(&m_ant_hrm, p_ant_evt);
	  ant_state_indicator_evt_handle(p_ant_evt);
	  LEDS_OFF(LEDS_MASK);
	  nrf_delay_ms(1);
	  LEDS_ON(1 << leds_list[i]);
	  i++;
	  if (i>=LEDS_NUMBER) {
			i=0;
		}
	  //LEDS_INVERT(0 | 1 << leds_list[i]);
}

/**@brief Function for the Timer, Tracer and BSP initialization.
 */
void utils_setup(void)
{
    uint32_t err_code;

    app_trace_init();
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
    err_code = bsp_init(BSP_INIT_LED, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Function for handling SoftDevice asserts.
 *
 * @param[in] pc          Value of the program counter.
 * @param[in] line_num    Line number where the assert occurred.
 * @param[in] p_file_name Pointer to the file name.
 */
void softdevice_assert_callback(uint32_t pc, uint16_t line_num, const uint8_t * p_file_name)
{
	while (1) {
		app_trace_log("erreur\n\r");
		LEDS_INVERT(LEDS_MASK);
    nrf_delay_ms(150);
	}
}



/**@brief Function for ANT stack initialization.
 *
 * @details Initializes the SoftDevice and the ANT event interrupt.
 */
static void softdevice_setup(void)
{
    uint32_t err_code;
 
    err_code = softdevice_ant_evt_handler_set(ant_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    err_code = softdevice_handler_init(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL, 0, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = ant_stack_static_config();
    APP_ERROR_CHECK(err_code);

    err_code = sd_ant_network_address_set(ANTPLUS_NETWORK_NUMBER, m_network_key);
    APP_ERROR_CHECK(err_code);

	
	
}

/**@brief Function for HRM profile initialization.
 *
 * @details Initializes the HRM profile and open ANT channel.
 */
static void profile_setup(void)
{
/** @snippet [ANT HRM RX Profile Setup] */
    uint32_t err_code;
		


    err_code = ant_hrm_init(&m_ant_hrm, &ant_rx_channel_config, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = ant_hrm_open(&m_ant_hrm);
    APP_ERROR_CHECK(err_code);

    //err_code = ant_state_indicator_channel_opened();
    //APP_ERROR_CHECK(err_code);
	
	
	// CAD
	  err_code = ant_cad_init(&m_ant_cad, &ant_rx_channel_config2, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = ant_hrm_open(&m_ant_cad);
    APP_ERROR_CHECK(err_code);

    //err_code = ant_state_indicator_channel_opened();
    //APP_ERROR_CHECK(err_code);


/** @snippet [ANT HRM RX Profile Setup] */
}

/**@brief Function for application main entry, does not return.
 */


int main(void)
{
    uint32_t err_code;

	  LEDS_CONFIGURE(LEDS_MASK);
	  LEDS_ON(LEDS_MASK);
	  nrf_delay_ms(1500);
	  LEDS_OFF(LEDS_MASK);
	
	  utils_setup();
	
	   // Toggle LEDs.
    while (false)
    {
        for (int i = 0; i < LEDS_NUMBER; i++)
        {
					  printf("Boucle\n\r");
					  LEDS_INVERT(LEDS_MASK);
            nrf_delay_ms(500);
        }
    }
		
    
    softdevice_setup();
		
	  //ant_state_indicator_init(m_ant_cad.channel_number, HRM_RX_CHANNEL_TYPE);
    //ant_state_indicator_init(m_ant_hrm.channel_number, HRM_RX_CHANNEL_TYPE);
    profile_setup();

    for (;;)
    {   
        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
}
/**
 *@}
 **/
