
#ifndef ANT_GLASSES_H__
#define ANT_GLASSES_H__

#include <stdint.h>
#include <stdbool.h>
#include "ant_parameters.h"
#include "ant_stack_handler_types.h"
#include "ant_channel_config.h"

#define GLASSES_DEVICE_NUMBER   0xFDDA

#define GLASSES_DEVICE_TYPE     0x2Du                       ///< Device type
#define GLASSES_ANTPLUS_RF_FREQ 0x32u                       ///< Frequency, decimal 57 (2457 MHz).

#define GLASSES_EXT_ASSIGN      0x00 
#define GLASSES_MSG_PERIOD      0x2AF8u


#define GLASSES_RX_CHANNEL_CONFIG(CHANNEL_NUMBER, TRANSMISSION_TYPE, DEVICE_NUMBER, NETWORK_NUMBER) \
    {                                                                                                           \
        .channel_number     = (CHANNEL_NUMBER),                                                                 \
        .channel_type       = CHANNEL_TYPE_SLAVE_RX_ONLY,                                                              \
        .ext_assign         = GLASSES_EXT_ASSIGN,                                                                   \
        .rf_freq            = GLASSES_ANTPLUS_RF_FREQ,                                                              \
        .transmission_type  = (TRANSMISSION_TYPE),                                                              \
        .device_type        = GLASSES_DEVICE_TYPE,                                                                  \
        .device_number      = (DEVICE_NUMBER),                                                                  \
        .channel_period     = (GLASSES_MSG_PERIOD),                                                                 \
        .network_number     = (NETWORK_NUMBER),                                                                 \
    }
    
#define GLASSES_TX_CHANNEL_CONFIG(CHANNEL_NUMBER, TRANSMISSION_TYPE, DEVICE_NUMBER, NETWORK_NUMBER) \
    {                                                                                                           \
        .channel_number     = (CHANNEL_NUMBER),                                                                 \
        .channel_type       = CHANNEL_TYPE_MASTER_TX_ONLY,                                                              \
        .ext_assign         = GLASSES_EXT_ASSIGN,                                                                   \
        .rf_freq            = GLASSES_ANTPLUS_RF_FREQ,                                                              \
        .transmission_type  = (TRANSMISSION_TYPE),                                                              \
        .device_type        = GLASSES_DEVICE_TYPE,                                                                  \
        .device_number      = (DEVICE_NUMBER),                                                                  \
        .channel_period     = (GLASSES_MSG_PERIOD),                                                                 \
        .network_number     = (NETWORK_NUMBER),                                                                 \
    }
    

    
typedef struct
{
    uint8_t led_mask;
    uint8_t avance[7];
}ant_glasses_data_layout_t;

    
typedef struct
{
    uint8_t led_mask;
    float avance;
}ant_glasses_trans;

    
    /**@brief GLASSES profile structure. */
typedef struct
{
    uint8_t              channel_number;    ///< Channel number assigned to the profile.
    uint8_t            * p_cb;              ///< Pointer to internal control block.
    ant_glasses_data_layout_t data;
}ant_glasses_profile_t;
    
uint32_t ant_glasses_init(ant_glasses_profile_t * p_profile, ant_channel_config_t const * p_channel_config);

uint32_t ant_glasses_open(ant_glasses_profile_t * p_profile);

void ant_glasses_rx_evt_handle(ant_glasses_profile_t * p_profile, ant_evt_t * p_ant_event, ant_glasses_trans *trans);

void ant_glasses_tx_evt_handle(ant_glasses_profile_t * p_profile, ant_evt_t * p_ant_event, uint8_t p_message_payload[8]);

#endif
