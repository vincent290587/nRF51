
#include "myIncludes.h"

#define ANT_HRM_TOGGLE_MASK             0x80                                         /**< HRM Page Toggle Bit Mask. */
#define ANT_HRM_PAGE_0                  0                                            /**< HRM page 0 constant. */
#define ANT_HRM_PAGE_1                  1                                            /**< HRM page 1 constant. */
#define ANT_HRM_PAGE_2                  2                                            /**< HRM page 2 constant. */
#define ANT_HRM_PAGE_3                  3                                            /**< HRM page 3 constant. */
#define ANT_HRM_PAGE_4                  4                                            /**< HRM page 4 constant. */
#define ANT_BUFFER_INDEX_MESG_ID        0x01                                         /**< Index for Message ID. */
#define ANT_BUFFER_INDEX_MESG_DATA      0x03                                         /**< Index for Data. */

#define WHEEL_CIRCUMFERENCE         2000                                                            /**< Bike wheel circumference [mm] */
#define BSC_EVT_TIME_FACTOR         1024                                                            /**< Time unit factor for BSC events */
#define BSC_RPM_TIME_FACTOR         60                                                              /**< Time unit factor for RPM unit */
#define BSC_MS_TO_KPH_NUM           36                                                              /**< Numerator of [m/s] to [kph] ratio */
#define BSC_MS_TO_KPH_DEN           10                                                              /**< Denominator of [m/s] to [kph] ratio */
#define BSC_MM_TO_M_FACTOR          1000                                                            /**< Unit factor [m/s] to [mm/s] */
#define BSC_SPEED_UNIT_FACTOR       (BSC_MS_TO_KPH_DEN * BSC_MM_TO_M_FACTOR)                        /**< Speed unit factor */
#define SPEED_COEFFICIENT           (WHEEL_CIRCUMFERENCE * BSC_EVT_TIME_FACTOR * BSC_MS_TO_KPH_NUM) /**< Coefficient for speed value calculation */
#define CADENCE_COEFFICIENT         (BSC_EVT_TIME_FACTOR * BSC_RPM_TIME_FACTOR)                     /**< Coefficient for cadence value calculation */


#define ANT_DELAY                    APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)


#define ANTPLUS_NETWORK_NUMBER          0x00                        /**< Network number. */
#define ANTPLUS_NETWORK_KEY             {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45}    /**< The default network key used. */
#define ANT_HRMRX_TRANS_TYPE            0                                            /**< Transmission Type. */

#define HRM_RX_CHANNEL_NUMBER       0x00                        /**< Channel number assigned to HRM profile. */
#define BSC_RX_CHANNEL_NUMBER       0x01
#define GLASSES_TX_CHANNEL_NUMBER   0x02

#define WILDCARD_DEVICE_NUMBER      0x0000                        /**< Wildcard device number. */
#define HRM_DEVICE_NUMBER           0x0D22                        
#define BSC_DEVICE_NUMBER           0xB02B         

typedef struct
{
    int32_t     accumulated_s_rev_cnt;
    int32_t     previous_s_evt_cnt;
    int32_t     prev_s_accumulated_rev_cnt;
    int32_t     accumulated_s_evt_time;
  int32_t       previous_s_evt_time;
  int32_t       prev_s_accumulated_evt_time;
} ant_bsc_prev_t;


void ant_hrm_rx_init(void);
void ant_bsc_rx_init(void);
void ant_evt_hrm (ant_evt_t * p_ant_evt);
void ant_evt_bsc (ant_evt_t * p_ant_evt);
void ant_evt_dispatch(ant_evt_t * p_ant_evt);
void ant_init(void);

extern void nus_send(char * text);
