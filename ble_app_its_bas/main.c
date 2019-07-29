/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * @brief Blinky Sample Application main file.
 *
 * This file contains the source code for a sample server application using the LED Button service.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "app_uart.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"

#include "nrf52_dk.h"
#include "app_object.h"
#include "counter.h"

#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "app_scheduler.h"
#include "nrf_delay.h"

#include "ble_image_transfer_service.h"
#include "ble_bas.h"

#include "nrf_drv_saadc.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define ADVERTISING_LED BSP_BOARD_LED_0 /**< Is on when device is advertising. */
#define CONNECTED_LED BSP_BOARD_LED_1   /**< Is on when device has connected. */
#define LEDBUTTON_LED BSP_BOARD_LED_2   /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON BSP_BUTTON_0   /**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME "ITS_Node" /**< Name of device. Will be included in the advertising data. */

#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG 1  /**< A tag identifying the SoftDevice BLE configuration. */

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2000)//120000)                 /**< Battery level measurement interval (ticks). This value corresponds to 120 seconds. */

#define APP_ADV_INTERVAL 1600//64                                    /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(7.5, UNIT_1_25_MS) /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(7.5, UNIT_1_25_MS) /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY 0                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(6000, UNIT_10_MS)   /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(50000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(20000)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                        /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50) /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE 40 /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE 20 /**< Maximum number of events in the scheduler queue. */
#endif

#define TX_POWER_LEVEL (4) /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */


#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                                     /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                                       /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  270                                     /**< Typical forward voltage drop of the diode . */
#define ADC_RES_10BIT                   1024                                    /**< Maximum digital value for 10-bit ADC conversion. */


#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

APP_TIMER_DEF(m_battery_timer_id);                      /**< Battery measurement timer. */

BLE_ITS_DEF(m_its, NRF_SDH_BLE_PERIPHERAL_LINK_COUNT); /**< BLE IMAGE TRANSFER service instance. */
BLE_BAS_DEF(m_bas);                                     /**< Battery service instance. */

NRF_BLE_GATT_DEF(m_gatt); /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);   /**< Context for the Queued Write module.*/

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;           /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];            /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX]; /**< Buffer for storing an encoded scan data. */

static uint16_t m_ble_its_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static uint32_t received_count = 0;
static uint32_t received_bytes = 0;


static bool m_battery_timer_is_running = false;

static nrf_saadc_value_t adc_buf[2];


static void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t * p_evt);


/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
        .adv_data =
        {
                .p_data = m_enc_advdata,
                .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX
        },
        .scan_rsp_data =
        {
                .p_data = m_enc_scan_response_data,
                .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX

        }
};


/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 *
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
        ((((ADC_VALUE) *ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)



/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
        if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
        {
                nrf_saadc_value_t adc_result;
                uint16_t batt_lvl_in_milli_volts;
                uint8_t percentage_batt_lvl;
                uint32_t err_code;

                adc_result = p_event->data.done.p_buffer[0];

                err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
                APP_ERROR_CHECK(err_code);

                batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                          DIODE_FWD_VOLT_DROP_MILLIVOLTS;
                percentage_batt_lvl = battery_level_in_percent(batt_lvl_in_milli_volts);

                NRF_LOG_INFO("Battery in %03d%%", percentage_batt_lvl);

                err_code = ble_bas_battery_level_update(&m_bas, percentage_batt_lvl, BLE_CONN_HANDLE_ALL);
                if ((err_code != NRF_SUCCESS) &&
                    (err_code != NRF_ERROR_INVALID_STATE) &&
                    (err_code != NRF_ERROR_RESOURCES) &&
                    (err_code != NRF_ERROR_BUSY) &&
                    (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                    )
                {
                        APP_ERROR_HANDLER(err_code);
                }
                nrf_drv_saadc_uninit();
        }

}

/**@brief Function for configuring ADC to do battery level conversion.
 */
static void adc_configure(void)
{
        ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
        APP_ERROR_CHECK(err_code);

        nrf_saadc_channel_config_t config =
                NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
        err_code = nrf_drv_saadc_channel_init(0, &config);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], 1);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], 1);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *          This function will start the ADC.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
        UNUSED_PARAMETER(p_context);
        adc_configure();
        ret_code_t err_code;
        err_code = nrf_drv_saadc_sample();
        APP_ERROR_CHECK(err_code);
        
        NRF_LOG_INFO("%s", __func__);
}


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
        app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
        bsp_board_init(BSP_INIT_LEDS);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
        // Initialize timer module, making it use the scheduler
        ret_code_t err_code = app_timer_init();
        APP_ERROR_CHECK(err_code);

        // Create battery timer.
        err_code = app_timer_create(&m_battery_timer_id,
                                    APP_TIMER_MODE_REPEATED,
                                    battery_level_meas_timeout_handler);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for changing the tx power.
 */
static void tx_power_set(void)
{
        ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, TX_POWER_LEVEL);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
        ret_code_t err_code;
        ble_gap_conn_params_t gap_conn_params;
        ble_gap_conn_sec_mode_t sec_mode;

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

        err_code = sd_ble_gap_device_name_set(&sec_mode,
                                              (const uint8_t *)DEVICE_NAME,
                                              strlen(DEVICE_NAME));
        APP_ERROR_CHECK(err_code);

        memset(&gap_conn_params, 0, sizeof(gap_conn_params));

        gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
        gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
        gap_conn_params.slave_latency = SLAVE_LATENCY;
        gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

        err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
        APP_ERROR_CHECK(err_code);

        ble_gap_addr_t ble_address = {.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
                                      .addr_id_peer = 0,
                                      .addr = {0xC3, 0x11, 0x99, 0x33, 0x44, 0xFF}};
        err_code = sd_ble_gap_addr_set(&ble_address);
        APP_ERROR_CHECK(err_code);
}

static void its_evt_handler(ble_its_t *p_its, ble_its_evt_t const *p_its_evt)
{
        switch (p_its_evt->evt_type)
        {
        case BLE_ITS_EVT_ITS_RX_EVT:

                // Read the object header
                received_bytes = 0;
                received_count = 0;
                {
                        ble_its_img_info_t image_info;
                        memcpy(&image_info, p_its_evt->p_data, p_its_evt->data_len);
                        NRF_LOG_INFO("Image file = %04x", image_info.file_size_bytes);
                        NRF_LOG_INFO("Image CRC32 = %04x", image_info.crc32);
                }
                break;

        case BLE_ITS_EVT_ITS_RX_DATA_EVT:
                received_bytes += p_its_evt->data_len;
                NRF_LOG_INFO("count = %x, %x", received_count++, received_bytes);
                //NRF_LOG_INFO("BLE_ITS_EVT_ITS_RX_DATA_EVT");
                break;

        default:
                break;
        }
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt)
{
        uint32_t data_length;
        if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
        {
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                m_ble_its_max_data_len = data_length;
                NRF_LOG_INFO("gatt_event: ATT MTU is set to 0x%X (%d)", data_length, data_length);
        }
        else if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED))
        {
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH - 4;
                m_ble_its_max_data_len = data_length;
                NRF_LOG_INFO("gatt_event: Data len is set to 0x%X (%d)", data_length, data_length);
        }
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
        ret_code_t err_code;

        err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
        ret_code_t err_code;
        ble_advdata_t advdata;
        ble_advdata_t srdata;

        ble_uuid_t adv_uuids[] = {{BLE_UUID_ITS_SERVICE, m_its.uuid_type}};

        // Build and set advertising data.
        memset(&advdata, 0, sizeof(advdata));

        advdata.name_type = BLE_ADVDATA_FULL_NAME;
        advdata.include_appearance = true;
        advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

        memset(&srdata, 0, sizeof(srdata));
        srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
        srdata.uuids_complete.p_uuids = adv_uuids;

        err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
        APP_ERROR_CHECK(err_code);

        err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
        APP_ERROR_CHECK(err_code);

        ble_gap_adv_params_t adv_params;

        // Set advertising parameters.
        memset(&adv_params, 0, sizeof(adv_params));

        adv_params.primary_phy = BLE_GAP_PHY_1MBPS;
        adv_params.duration = APP_ADV_DURATION;
        adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
        adv_params.p_peer_addr = NULL;
        adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
        adv_params.interval = APP_ADV_INTERVAL;

        err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
        APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Battery Service.
 */
static void bas_init(void)
{
        ret_code_t err_code;
        ble_bas_init_t bas_init_obj;

        memset(&bas_init_obj, 0, sizeof(bas_init_obj));

        bas_init_obj.evt_handler          = on_bas_evt;
        bas_init_obj.support_notification = true;
        bas_init_obj.p_report_ref         = NULL;
        bas_init_obj.initial_batt_level   = 100;

        bas_init_obj.bl_rd_sec        = SEC_OPEN;
        bas_init_obj.bl_cccd_wr_sec   = SEC_OPEN;
        bas_init_obj.bl_report_rd_sec = SEC_OPEN;

        err_code = ble_bas_init(&m_bas, &bas_init_obj);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
        ret_code_t err_code;
        nrf_ble_qwr_init_t qwr_init = {0};
        ble_its_init_t its_init;

        // Initialize Queued Write Module.
        qwr_init.error_handler = nrf_qwr_error_handler;

        err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
        APP_ERROR_CHECK(err_code);

        // Initialize NUS.
        memset(&its_init, 0, sizeof(its_init));

        // Initialize ITS.
        its_init.evt_handler = its_evt_handler;
        err_code = ble_its_init(&m_its, &its_init);
        APP_ERROR_CHECK(err_code);

        bas_init();
}

/**@brief Function for handling the Battery Service events.
 *
 * @details This function will be called for all Battery Service events which are passed to the
 |          application.
 *
 * @param[in] p_bas  Battery Service structure.
 * @param[in] p_evt  Event received from the Battery Service.
 */
static void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t * p_evt)
{
        ret_code_t err_code;

        switch (p_evt->evt_type)
        {
        case BLE_BAS_EVT_NOTIFICATION_ENABLED:
                // Start battery timer

                if (m_battery_timer_is_running == false)
                {
                        err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
                        APP_ERROR_CHECK(err_code);
                        m_battery_timer_is_running = true;
                }

                break; // BLE_BAS_EVT_NOTIFICATION_ENABLED

        case BLE_BAS_EVT_NOTIFICATION_DISABLED:
                if (m_battery_timer_is_running)
                {
                        err_code = app_timer_stop(m_battery_timer_id);
                        APP_ERROR_CHECK(err_code);
                        m_battery_timer_is_running = false;
                }

                break; // BLE_BAS_EVT_NOTIFICATION_DISABLED

        default:
                // No implementation needed.
                break;
        }
}

static void application_battery_timer_start(void)
{
        ret_code_t err_code = NRF_SUCCESS;
        if (m_battery_timer_is_running == false)
        {
                err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
                APP_ERROR_CHECK(err_code);
                m_battery_timer_is_running = true;
        }
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
        ret_code_t err_code;

        if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
        {
                err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
                APP_ERROR_CHECK(err_code);
        }
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
        ret_code_t err_code;
        ble_conn_params_init_t cp_init;

        memset(&cp_init, 0, sizeof(cp_init));

        cp_init.p_conn_params = NULL;
        cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
        cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
        cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
        cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
        cp_init.disconnect_on_fail = false;
        cp_init.evt_handler = on_conn_params_evt;
        cp_init.error_handler = conn_params_error_handler;

        err_code = ble_conn_params_init(&cp_init);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
        ret_code_t err_code;

        err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
        APP_ERROR_CHECK(err_code);

        bsp_board_led_on(ADVERTISING_LED);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
        ret_code_t err_code;

        // For readability.
        ble_gap_evt_t const *p_gap_evt = &p_ble_evt->evt.gap_evt;

        switch (p_ble_evt->header.evt_id)
        {
        case BLE_GAP_EVT_CONNECTED:
                NRF_LOG_INFO("Connected");
                bsp_board_led_on(CONNECTED_LED);
                bsp_board_led_off(ADVERTISING_LED);
                m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
                APP_ERROR_CHECK(err_code);
                err_code = app_button_enable();
                APP_ERROR_CHECK(err_code);

                tx_power_set();

                break;

        case BLE_GAP_EVT_DISCONNECTED:
                NRF_LOG_INFO("Connection 0x%x has been disconnected. Reason: 0x%X",
                             p_gap_evt->conn_handle,
                             p_gap_evt->params.disconnected.reason);
                bsp_board_led_off(CONNECTED_LED);
                m_conn_handle = BLE_CONN_HANDLE_INVALID;
                err_code = app_button_disable();
                APP_ERROR_CHECK(err_code);
                advertising_start();
                break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
                // Pairing not supported
                err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                       BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                       NULL,
                                                       NULL);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
                NRF_LOG_DEBUG("PHY update request.");
                ble_gap_phys_t const phys =
                {
                        .rx_phys = BLE_GAP_PHY_AUTO,
                        .tx_phys = BLE_GAP_PHY_AUTO,
                };
                err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
                APP_ERROR_CHECK(err_code);
        }
        break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
                // No system attributes have been stored.
                err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTC_EVT_TIMEOUT:
                // Disconnect on GATT Client timeout event.
                NRF_LOG_DEBUG("GATT Client Timeout.");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTS_EVT_TIMEOUT:
                // Disconnect on GATT Server timeout event.
                NRF_LOG_DEBUG("GATT Server Timeout.");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

        default:
                // No implementation needed.
                break;
        }
}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
        APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
        ret_code_t err_code;

        err_code = nrf_sdh_enable_request();
        APP_ERROR_CHECK(err_code);

        // Configure the BLE stack using the default settings.
        // Fetch the start address of the application RAM.
        uint32_t ram_start = 0;
        err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
        APP_ERROR_CHECK(err_code);

        // Enable BLE stack.
        err_code = nrf_sdh_ble_enable(&ram_start);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
        APP_ERROR_CHECK(err_code);

        // Register a handler for BLE events.
        NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
        ret_code_t err_code;

        switch (pin_no)
        {
        case LEDBUTTON_BUTTON:
                break;
        default:
                APP_ERROR_HANDLER(pin_no);
                break;
        }
}

/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
        ret_code_t err_code;

        //The array must be static because a pointer to it will be saved in the button handler module.
        static app_button_cfg_t buttons[] =
        {
                {LEDBUTTON_BUTTON, false, BUTTON_PULL, button_event_handler}
        };

        err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                                   BUTTON_DETECTION_DELAY);
        APP_ERROR_CHECK(err_code);
}

static void log_init(void)
{
        ret_code_t err_code = NRF_LOG_INIT(NULL);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
        ret_code_t err_code;
        err_code = nrf_pwr_mgmt_init();
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
        app_sched_execute();
        while (NRF_LOG_PROCESS())
                ;
        nrf_pwr_mgmt_run();
}

/**@brief Function for application main entry.
 */
int main(void)
{

        bool erase_bonds;
        ret_code_t err_code;

        /* enable instruction cache */
        NRF_NVMC->ICACHECNF = (NVMC_ICACHECNF_CACHEEN_Enabled << NVMC_ICACHECNF_CACHEEN_Pos) +
                              (NVMC_ICACHECNF_CACHEPROFEN_Disabled << NVMC_ICACHECNF_CACHEPROFEN_Pos);

        // Initialize.
        log_init();
        leds_init();
        timers_init();
        buttons_init();
        power_management_init();
        ble_stack_init();
        
        scheduler_init();
        gap_params_init();
        gatt_init();
        services_init();
        advertising_init();
        conn_params_init();

        // Start execution.
        NRF_LOG_INFO("Throughput Example : Peripheral ITS.");
        advertising_start();

        application_battery_timer_start();

        // Enter main loop.
        for (;;)
        {
                idle_state_handle();
        }
}

/**
 * @}
 */
