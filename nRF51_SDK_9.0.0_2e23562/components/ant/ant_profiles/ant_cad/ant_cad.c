


#include "nrf_assert.h"
#include "app_error.h"
#include "ant_interface.h"
#include "app_util.h"
#include "ant_cad.h"
#include "app_error.h"
#include "app_trace.h"
#include "ant_hrm_page_logger.h"

#define HIGH_BYTE(word)              (uint8_t)((word >> 8u) & 0x00FFu)                                                 /**< Get high byte of a uint16_t. */
#define LOW_BYTE(word)               (uint8_t)(word & 0x00FFu)                                                         /**< Get low byte of a uint16_t. */

#define TX_BUFFER_SIZE               8u                                                                                /**< Transmit buffer size. */

#define CADENCE_EVENT_TIME_LSB_INDEX 0                                                                                 /**< Index of the cadence event time LSB field in the data page. */
#define CADENCE_EVENT_TIME_MSB_INDEX 1u                                                                                /**< Index of the cadence event time MSB field in the data page. */
#define CUMULATIVE_CADENCE_LSB_INDEX 2u                                                                                /**< Index of the cumulative cadence revolution count LSB field in the data page. */
#define CUMULATIVE_CADENCE_MSB_INDEX 3u                                                                                /**< Index of the cumulative cadence revolution count MSB field in the data page. */
#define SPEED_EVENT_TIME_LSB_INDEX   4u                                                                                /**< Index of the speed event time LSB field in the data page. */
#define SPEED_EVENT_TIME_MSB_INDEX   5u                                                                                /**< Index of the speed event time MSB field in the data page. */
#define CUMULATIVE_SPEED_LSB_INDEX   6u                                                                                /**< Index of the cumulative speed revolution count LSB field in the data page. */
#define CUMULATIVE_SPEED_MSB_INDEX   7u                                                                                /**< Index of the cumulative speed revolution count MSB field in the data page. */

#define RTC_COUNTER_FREQ             1024u                                                                             /**< Desired RTC COUNTER frequency is 1024Hz (1/1024s period). */
#define RTC_PRESCALER                (ROUNDED_DIV(APP_TIMER_CLOCK_FREQ, RTC_COUNTER_FREQ) - 1u)                        /**< Computed value of the RTC prescaler register. */

#define SPEED_RPM                    90u                                                                               /**< Speed rotations per minute. */
#define SPEED_EVENT_INTERVAL         ((60 * RTC_COUNTER_FREQ) / SPEED_RPM)                                             /**< Used speed event interval in units of 1/1024 seconds. */
#define SPEED_EVENT_INTERVAL_TICKS   APP_TIMER_TICKS(ROUNDED_DIV((60u * RTC_COUNTER_FREQ), SPEED_RPM), RTC_PRESCALER)  /**< Speed event interval in timer tick units. */

#define CADENCE_RPM                  25u                                                                               /**< Cadence rotations per minute. */
#define CADENCE_EVENT_INTERVAL       ((60u * RTC_COUNTER_FREQ) / CADENCE_RPM)                                          /**< Used cadence event interval in units of 1/1024 seconds. */
#define CADENCE_EVENT_INTERVAL_TICKS APP_TIMER_TICKS(ROUNDED_DIV((60u * RTC_COUNTER_FREQ), CADENCE_RPM), RTC_PRESCALER)/**< Speed event interval in timer tick units. */

//static uint8_t  m_tx_buffer[TX_BUFFER_SIZE];                                                                           /**< Power main data page transmit buffer. */
static uint32_t m_cadence_event_time       = 0;                                                                        /**< Cadence event time tracker. */
static uint32_t m_cadence_revolution_count = 1u;                                                                       /**< Cadence revolution count tracker. */
static uint32_t m_speed_event_time         = 0;                                                                        /**< Speed event time tracker. */
static uint32_t m_speed_revolution_count   = 1u;     /**< Speed revolution count tracker. */

static uint32_t m_cadence_event_time_prev       = 0;                                                                        /**< Cadence event time tracker. */
static uint32_t m_cadence_revolution_count_prev = 1u;                                                                       /**< Cadence revolution count tracker. */
static uint32_t m_speed_event_time_prev         = 0;                                                                        /**< Speed event time tracker. */
static uint32_t m_speed_revolution_count_prev  = 1u;     /**< Speed revolution count tracker. */

typedef struct
{
    ant_cad_page0_data_layout_t         page;
}ant_cad_message_layout_t;


uint32_t ant_cad_init(ant_cad_profile_t * p_profile, ant_channel_config_t const * p_channel_config)
{
	uint32_t err_code;
    ASSERT(p_channel_config != NULL);
    ASSERT(p_profile != NULL);
  
    p_profile->channel_number = p_channel_config->channel_number;

    LOG_ANT("ANT CAD channel %u init\n\r", p_channel_config->channel_number);
		err_code = ant_channel_init(p_channel_config);
		return err_code;
}

uint32_t ant_cad_open(ant_cad_profile_t * p_profile)
{
    LOG_ANT("ANT CAD channel %u open\n\r", p_profile->channel_number);
    return sd_ant_channel_open(p_profile->channel_number);
}

static void decode_cad_rx_message(ant_cad_profile_t * p_profile, uint8_t * p_message_payload)
{
	LOG_ANT("CAD message\r\n");
  
  const ant_cad_message_layout_t *payload  = (ant_cad_message_layout_t *)p_message_payload;
  
  m_cadence_event_time = payload->page.cad_evt_time_MSB * 256 + payload->page.cad_evt_time_LSB;
  m_cadence_revolution_count = payload->page.cum_cad_evt_time_MSB * 256 + payload->page.cum_cad_evt_time_LSB;
  m_speed_event_time = payload->page.speed_evt_time_MSB * 256 + payload->page.speed_evt_time_LSB;
  m_speed_revolution_count = payload->page.rev_cnt_evt_time_MSB * 256 + payload->page.rev_cnt_evt_time_LSB;
  
  uint32_t d_cad_time = m_cadence_event_time - m_cadence_event_time_prev;
  uint32_t d_cad_nb = m_cadence_revolution_count - m_cadence_revolution_count_prev;
  
  uint32_t d_speed_time = m_speed_event_time - m_speed_event_time_prev;
  uint32_t d_speed_nb = m_speed_revolution_count - m_speed_revolution_count_prev;
  
  float tmpcad, tmpspeed;

  if (d_cad_nb && d_speed_nb) {
    tmpcad = 60. * (float)d_cad_nb / (float)d_cad_time;
    tmpcad *= 1024.;
  
    tmpspeed = 3.6 * 2000. * (float)d_speed_nb / (float)d_speed_time;
    tmpspeed *= 1024;
  
    m_cadence_event_time_prev = m_cadence_event_time;
    m_cadence_revolution_count_prev = m_cadence_revolution_count;
  
    m_speed_event_time_prev = m_speed_event_time;
    m_speed_revolution_count_prev = m_speed_revolution_count;
  
    printf("$CAD,%u,%.1f\n\r", (unsigned int)tmpcad, tmpspeed);
  }
}


void ant_cad_rx_evt_handle(ant_cad_profile_t * p_profile, ant_evt_t * p_ant_event)
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
                        decode_cad_rx_message(p_profile, p_message->ANT_MESSAGE_aucPayload);
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




