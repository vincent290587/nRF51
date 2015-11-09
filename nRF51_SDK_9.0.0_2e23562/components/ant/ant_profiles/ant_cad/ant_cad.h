
#ifndef ANT_CAD_H__
#define ANT_CAD_H__

#include <stdint.h>
#include <stdbool.h>
#include "ant_parameters.h"
#include "ant_stack_handler_types.h"
#include "ant_channel_config.h"


#define CAD_DEVICE_TYPE     0x79u                       ///< Device type reserved for ANT+ heart rate monitor.
#define CAD_ANTPLUS_RF_FREQ 0x39u                       ///< Frequency, decimal 57 (2457 MHz).

#define CAD_EXT_ASSIGN      0x00 
#define CAD_MSG_PERIOD      0x1F86u


#define CAD_RX_CHANNEL_CONFIG(CHANNEL_NUMBER, TRANSMISSION_TYPE, DEVICE_NUMBER, NETWORK_NUMBER, HRM_MSG_PERIOD) \
    {                                                                                                           \
        .channel_number     = (CHANNEL_NUMBER),                                                                 \
        .channel_type       = CHANNEL_TYPE_SLAVE_RX_ONLY,                                                              \
        .ext_assign         = CAD_EXT_ASSIGN,                                                                   \
        .rf_freq            = CAD_ANTPLUS_RF_FREQ,                                                              \
        .transmission_type  = (TRANSMISSION_TYPE),                                                              \
        .device_type        = CAD_DEVICE_TYPE,                                                                  \
        .device_number      = (DEVICE_NUMBER),                                                                  \
        .channel_period     = (CAD_MSG_PERIOD),                                                                 \
        .network_number     = (NETWORK_NUMBER),                                                                 \
    }
    
typedef struct
{
    uint8_t cad_evt_time_LSB;
    uint8_t cad_evt_time_MSB;
  
    uint8_t cum_cad_evt_time_LSB;
    uint8_t cum_cad_evt_time_MSB;
  
    uint8_t speed_evt_time_LSB;
    uint8_t speed_evt_time_MSB;
  
    uint8_t rev_cnt_evt_time_LSB;
    uint8_t rev_cnt_evt_time_MSB;
}ant_cad_page0_data_layout_t;
    
    /**@brief CAD profile structure. */
typedef struct
{
    uint8_t              channel_number;    ///< Channel number assigned to the profile.
    uint8_t            * p_cb;              ///< Pointer to internal control block.
    ant_cad_page0_data_layout_t data;
}ant_cad_profile_t;
    
uint32_t ant_cad_init(ant_cad_profile_t * p_profile, ant_channel_config_t const * p_channel_config);

uint32_t ant_cad_open(ant_cad_profile_t * p_profile);

void ant_cad_rx_evt_handle(ant_cad_profile_t * p_profile, ant_evt_t * p_ant_event);

#endif
