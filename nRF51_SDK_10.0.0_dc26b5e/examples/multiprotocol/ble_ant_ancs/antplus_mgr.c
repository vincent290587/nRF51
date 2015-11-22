#include "myIncludes.h"

#include "antplus_mgr.h"

                            
static int32_t accumulated_s_rev_cnt, previous_s_evt_cnt, prev_s_accumulated_rev_cnt,
               accumulated_s_evt_time, previous_s_evt_time, prev_s_accumulated_evt_time = 0;

static int32_t accumulated_c_rev_cnt, previous_c_evt_cnt, prev_c_accumulated_rev_cnt,
               accumulated_c_evt_time, previous_c_evt_time, prev_c_accumulated_evt_time = 0;

static uint32_t s_previous_beat_count = 0;    // Heart beat count from previously received page
static uint32_t computed_speed   = 0;


static uint8_t is_hrm_init = 0;
static uint8_t is_bsc_init = 0;
APP_TIMER_DEF(m_sec_hrm);
APP_TIMER_DEF(m_sec_bsc);


/**@brief Handle received ANT data message.
 * 
 * @param[in]  p_evt_buffer   The buffer containing received data. 
 */
static void hrm_data_messages_handle(uint8_t * p_evt_buffer)
{
    uint32_t        err_code;
    uint32_t        current_page;
    uint8_t         beat_count;
    uint8_t         computed_heart_rate;
    uint16_t        beat_time;
    uint16_t        rr_interval;

    // Decode the default page data present in all pages
    beat_time           = uint16_decode(&p_evt_buffer[ANT_BUFFER_INDEX_MESG_DATA + 4]);
    beat_count          = (uint8_t)p_evt_buffer[ANT_BUFFER_INDEX_MESG_DATA + 6];
    computed_heart_rate = (uint8_t)p_evt_buffer[ANT_BUFFER_INDEX_MESG_DATA + 7];

    // Decode page specific data
    current_page = p_evt_buffer[ANT_BUFFER_INDEX_MESG_DATA];
    switch (current_page & ~ANT_HRM_TOGGLE_MASK)
    {
        case ANT_HRM_PAGE_4:
            // Ensure that there is only one beat between time intervals.
            if ((beat_count - s_previous_beat_count) == 1)
            {
                uint16_t prev_beat = uint16_decode(&p_evt_buffer[ANT_BUFFER_INDEX_MESG_DATA + 2]);
                
                // Subtracting the event time gives the R-R interval
                rr_interval = beat_time - prev_beat;
                
                printf("$HRM,%u,%u\n\r", computed_heart_rate, rr_interval);
            }
            s_previous_beat_count = beat_count;
            
            break;
          
        case ANT_HRM_PAGE_0:
        case ANT_HRM_PAGE_1:
        case ANT_HRM_PAGE_2:
        case ANT_HRM_PAGE_3:
        default:
            // No implementation needed.
            break;
    }
    
}




__STATIC_INLINE uint32_t calculate_speed(int32_t rev_cnt, int32_t evt_time)
{

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


/**@brief Handle received ANT data message.
 * 
 * @param[in]  p_evt_buffer   The buffer containing received data. 
 */
void ant_bsc_evt_handler(uint8_t * p_evt_buffer)
{
  //char buffer[30];
  uint32_t speed=0, cadence=0;
  
  ant_bsc_combined_page0_data_t page0;
  memcpy(&page0, p_evt_buffer + ANT_BUFFER_INDEX_MESG_DATA, 8);
  
  speed = calculate_speed(page0.speed_rev_count, page0.speed_event_time);
  
  cadence = calculate_cadence(page0.cadence_rev_count, page0.cadence_event_time);
  

    
}


/**@brief ANT RX event handler.
 *
 * @param[in]   p_ant_evt   Event received from the stack.
 */
static void on_hrm_evt_rx(ant_evt_t * p_ant_evt)
{
    uint32_t message_id = p_ant_evt->evt_buffer[ANT_BUFFER_INDEX_MESG_ID];

    switch (message_id)
    {
        case MESG_BROADCAST_DATA_ID:
        case MESG_ACKNOWLEDGED_DATA_ID:
            hrm_data_messages_handle(p_ant_evt->evt_buffer);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief ANT RX event handler.
 *
 * @param[in]   p_ant_evt   Event received from the stack.
 */
static void on_bsc_evt_rx(ant_evt_t * p_ant_evt)
{
    uint32_t message_id = p_ant_evt->evt_buffer[ANT_BUFFER_INDEX_MESG_ID];

    switch (message_id)
    {
        case MESG_BROADCAST_DATA_ID:
        case MESG_ACKNOWLEDGED_DATA_ID:
            ant_bsc_evt_handler(p_ant_evt->evt_buffer);
            break;

        default:
            // No implementation needed.
            break;
    }
}


static void hrm_connect(void * p_context)
{
  uint32_t err_code = sd_ant_channel_open(HRM_RX_CHANNEL_NUMBER);
    if (err_code != NRF_SUCCESS && err_code != NRF_ANT_ERROR_CHANNEL_IN_WRONG_STATE)
    {
        APP_ERROR_HANDLER(err_code);
    }
}

static void bsc_connect(void * p_context)
{
  uint32_t err_code = sd_ant_channel_open(BSC_RX_CHANNEL_NUMBER);
    if (err_code != NRF_SUCCESS && err_code != NRF_ANT_ERROR_CHANNEL_IN_WRONG_STATE)
    {
        APP_ERROR_HANDLER(err_code);
    }
}

void ant_evt_hrm (ant_evt_t * p_ant_evt)
{
    uint32_t err_code = NRF_SUCCESS;
    
    uint16_t pusDeviceNumber = 0;
    uint8_t pucDeviceType    = 0;
    uint8_t pucTransmitType  = 0;
  
    //char buffer[30];

    switch (p_ant_evt->event)
					{
							case EVENT_RX:
                  if (!is_hrm_init) {
                    sd_ant_channel_id_get (HRM_RX_CHANNEL_NUMBER, 
                          &pusDeviceNumber, &pucDeviceType, &pucTransmitType);
                    printf("$ANCS,0,HRM 0x%x connected\n", pusDeviceNumber);
                    //sprintf(buffer, "Connexion HRM 0x%x\n", pusDeviceNumber);
                    ////SEGGER_RTT_WriteString(0, buffer);
                    if (pusDeviceNumber) is_hrm_init = 1;
                  }
                  on_hrm_evt_rx(p_ant_evt);
									break;
							case EVENT_RX_FAIL:
									break;
							case EVENT_RX_FAIL_GO_TO_SEARCH:
              case EVENT_RX_SEARCH_TIMEOUT:
									break;
							case EVENT_CHANNEL_CLOSED:
                  //SEGGER_RTT_WriteString(0, "Reconnexion HRM...\n");
                  is_hrm_init = 0;
                  err_code = app_timer_start(m_sec_hrm, ANT_DELAY, NULL);
                  break;
					}
          
    APP_ERROR_CHECK(err_code);
}

void ant_evt_bsc (ant_evt_t * p_ant_evt)
{
    uint32_t err_code = NRF_SUCCESS;
    
    uint16_t pusDeviceNumber = 0;
    uint8_t pucDeviceType    = 0;
    uint8_t pucTransmitType  = 0;
  
    //char buffer[30];

    switch (p_ant_evt->event)
					{
							case EVENT_RX:
                  if (!is_bsc_init) {
                    sd_ant_channel_id_get (BSC_RX_CHANNEL_NUMBER, 
                          &pusDeviceNumber, &pucDeviceType, &pucTransmitType);
                    printf("$ANCS,0,BSC 0x%x connected\n", pusDeviceNumber);
                    //sprintf(buffer, "Connexion BSC 0x%x\n", pusDeviceNumber);
                    ////SEGGER_RTT_WriteString(0, buffer);
                    if (pusDeviceNumber) is_bsc_init = 1;
                  }
                  on_bsc_evt_rx(p_ant_evt);
									break;
							case EVENT_RX_FAIL:
									break;
							case EVENT_RX_FAIL_GO_TO_SEARCH:
              case EVENT_RX_SEARCH_TIMEOUT:
									break;
							case EVENT_CHANNEL_CLOSED:
                  //SEGGER_RTT_WriteString(0, "Reconnexion BSC...\n");
                  is_bsc_init = 0;
                  err_code = app_timer_start(m_sec_bsc, ANT_DELAY, NULL);
                  break;
					}
          
    APP_ERROR_CHECK(err_code);
}

/**@brief Initialize the ANT HRM reception.
 */
void ant_hrm_rx_init(void)
{
    uint32_t err_code;
    err_code = sd_ant_channel_assign(HRM_RX_CHANNEL_NUMBER,
                                     HRM_DISP_CHANNEL_TYPE,
                                     ANTPLUS_NETWORK_NUMBER,
                                     HRM_EXT_ASSIGN);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ant_channel_id_set(HRM_RX_CHANNEL_NUMBER,
                                     HRM_DEVICE_NUMBER,
                                     HRM_DEVICE_TYPE,
                                     ANT_HRMRX_TRANS_TYPE);
    APP_ERROR_CHECK(err_code);
    
    err_code = sd_ant_channel_radio_freq_set(HRM_RX_CHANNEL_NUMBER, HRM_ANTPLUS_RF_FREQ);
    APP_ERROR_CHECK(err_code);
    
    err_code = sd_ant_channel_period_set(HRM_RX_CHANNEL_NUMBER, HRM_MSG_PERIOD_4Hz);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_create(&m_sec_hrm, APP_TIMER_MODE_SINGLE_SHOT, hrm_connect);
    APP_ERROR_CHECK(err_code);
}


/**@brief Initialize the ANT HRM reception.
 */
void ant_bsc_rx_init(void)
{
    uint32_t err_code;
    err_code = sd_ant_channel_assign(BSC_RX_CHANNEL_NUMBER,
                                     BSC_DISP_CHANNEL_TYPE,
                                     ANTPLUS_NETWORK_NUMBER,
                                     HRM_EXT_ASSIGN);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ant_channel_id_set(BSC_RX_CHANNEL_NUMBER,
                                     BSC_DEVICE_NUMBER,
                                     BSC_COMBINED_DEVICE_TYPE,
                                     ANT_HRMRX_TRANS_TYPE);
    APP_ERROR_CHECK(err_code);
    
    err_code = sd_ant_channel_radio_freq_set(BSC_RX_CHANNEL_NUMBER, BSC_ANTPLUS_RF_FREQ);
    APP_ERROR_CHECK(err_code);
    
    err_code = sd_ant_channel_period_set(BSC_RX_CHANNEL_NUMBER, BSC_MSG_PERIOD_COMBINED);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_create(&m_sec_bsc, APP_TIMER_MODE_SINGLE_SHOT, bsc_connect);
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
				ant_evt_bsc (p_ant_evt);
				break;
      
      case GLASSES_TX_CHANNEL_NUMBER:
        //ant_evt_glasses (p_ant_evt);
        break;
      
			default:
				break;
		}

}


void ant_init(void) {
  
    uint32_t err_code;
  
    // ANT plus init
    err_code = softdevice_ant_evt_handler_set(ant_evt_dispatch);
    APP_ERROR_CHECK(err_code);
  
    err_code = ant_plus_key_set(ANTPLUS_NETWORK_NUMBER);
    APP_ERROR_CHECK(err_code);
    
    err_code = ant_stack_static_config();
    APP_ERROR_CHECK(err_code);
  
    uint8_t m_ant_network_key[] = ANTPLUS_NETWORK_KEY;

    err_code = sd_ant_network_address_set(ANTPLUS_NETWORK_NUMBER, m_ant_network_key);
    APP_ERROR_CHECK(err_code);
    
    ant_hrm_rx_init();
    ant_bsc_rx_init();
  
    hrm_connect(NULL);
    bsc_connect(NULL);
}

