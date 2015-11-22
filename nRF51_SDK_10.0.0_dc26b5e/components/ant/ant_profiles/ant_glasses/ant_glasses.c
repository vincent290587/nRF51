
#include <string.h>
#include "nrf_assert.h"
#include "app_error.h"
#include "ant_interface.h"
#include "app_util.h"
#include "ant_glasses.h"
#include "app_error.h"
#include "app_trace.h"
#include "ant_hrm_page_logger.h"

#define HIGH_BYTE(word)              (uint8_t)((word >> 8u) & 0x00FFu)                                                 /**< Get high byte of a uint16_t. */
#define LOW_BYTE(word)               (uint8_t)(word & 0x00FFu)                                                         /**< Get low byte of a uint16_t. */

#define TX_BUFFER_SIZE               8u                                                                                /**< Transmit buffer size. */
                                                                       /**< Index of the cumulative speed revolution count MSB field in the data page. */

#define RTC_COUNTER_FREQ             1024u                                                                             /**< Desired RTC COUNTER frequency is 1024Hz (1/1024s period). */
#define RTC_PRESCALER                (ROUNDED_DIV(APP_TIMER_CLOCK_FREQ, RTC_COUNTER_FREQ) - 1u)                        /**< Computed value of the RTC prescaler register. */


uint32_t ant_glasses_init(ant_glasses_profile_t * p_profile, ant_channel_config_t const * p_channel_config)
 {
	uint32_t err_code;
    ASSERT(p_channel_config != NULL);
    ASSERT(p_profile != NULL);
  
    p_profile->channel_number = p_channel_config->channel_number;

    LOG_ANT("ANT GLASSES channel %u init\n\r", p_channel_config->channel_number);
		err_code = ant_channel_init(p_channel_config);
		return err_code;
}

uint32_t ant_glasses_open(ant_glasses_profile_t * p_profile)
{
    LOG_ANT("ANT GLASSES channel %u open\n\r", p_profile->channel_number);
    return sd_ant_channel_open(p_profile->channel_number);
}



void ant_glasses_tx_evt_handle(uint8_t channel_nb, ant_evt_t * p_ant_event, uint8_t p_message_payload[8])
{
        uint32_t err_code;
        switch (p_ant_event->event)
        {
            case EVENT_TX:
                err_code = sd_ant_broadcast_message_tx(channel_nb, 8*sizeof(uint8_t), p_message_payload);
                APP_ERROR_CHECK(err_code);
                break;

            default:
                break;
        }
}

static void decode_glasses_rx_message(uint8_t * p_message_payload, ant_glasses_trans *trans)
{
  float tmp;
  
  const ant_glasses_data_layout_t *msg  = (ant_glasses_data_layout_t *)p_message_payload;
  
  memcpy(&tmp, msg->avance, sizeof(float));
  
  if (trans) {
    trans->led_mask = msg->led_mask;
    trans->avance = (float)msg->avance[0];
    trans->avance += (float)msg->avance[1] / 100.;
  }
  
  LOG_ANT("Message: mask/%d avance/%.2f\n\r", msg->led_mask[0], tmp);

  return;
}

void ant_glasses_rx_evt_handle(ant_glasses_profile_t * p_profile, ant_evt_t * p_ant_event, ant_glasses_trans *trans)
{
   ANT_MESSAGE * p_message = (ANT_MESSAGE *)p_ant_event->evt_buffer;
  
        switch (p_ant_event->event)
        {
            case EVENT_RX:
                switch(p_message->ANT_MESSAGE_ucMesgID)
                {
                    case MESG_BROADCAST_DATA_ID:
                    case MESG_ACKNOWLEDGED_DATA_ID:
                    case MESG_BURST_DATA_ID:
                        decode_glasses_rx_message(p_message->ANT_MESSAGE_aucPayload, trans);
                    break;

                  default:
                        // No implementation needed
                    break;
                }
                break;
            default:
                break;
        }
		
}
