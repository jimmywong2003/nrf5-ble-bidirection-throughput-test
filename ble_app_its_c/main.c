/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
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
 * @brief BLE LED Button Service central and client application main file.
 *
 * This file contains the source code for a sample client application using the LED Button service.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "app_timer.h"
#include "app_uart.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"

#include "ble_image_transfer_service_c.h"

#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"
#include "ble_conn_state.h"

#include "app_scheduler.h"

#include "app_object.h"
#include "counter.h"

#include "nrf52_dk.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE 20 /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE 10 /**< Maximum number of events in the scheduler queue. */
#endif

#define APP_BLE_CONN_CFG_TAG 1  /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define UART_TX_BUF_SIZE 256 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256 /**< UART RX buffer size. */

#define CENTRAL_SCANNING_LED BSP_BOARD_LED_0  /**< Scanning LED will be on when the device is scanning. */
#define CENTRAL_CONNECTED_LED BSP_BOARD_LED_1 /**< Connected LED will be on when the device is connected. */
#define LEDBUTTON_LED BSP_BOARD_LED_2         /**< LED to indicate a change of state of the the Button characteristic on the peer. */

#define SCAN_INTERVAL 0x00A0 /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW 0x0050   /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION 0x0000 /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(7.5, UNIT_1_25_MS) /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(7.5, UNIT_1_25_MS) /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY 0                                          /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT MSEC_TO_UNITS(6000, UNIT_10_MS)      /**< Determines supervision time-out in units of 10 milliseconds. */

#define LEDBUTTON_BUTTON_PIN BSP_BUTTON_0 /**< Button that will write to the LED characteristic of the peer */
#define SEND_INFO_BUTTON_PIN BSP_BUTTON_3

#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50) /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define ECHOBACK_BLE_UART_DATA 0 /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */
#define TX_POWER_LEVEL (0)       /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */

NRF_BLE_SCAN_DEF(m_scan); /**< Scanning module instance. */
NRF_BLE_GATT_DEF(m_gatt); /**< GATT module instance. */

BLE_ITS_C_ARRAY_DEF(m_ble_its_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT); /**< BLE Nordic Image Transfer Service (ITS) client instance. */

BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT); /**< Database discovery module instances. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */

static uint16_t m_ble_its_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static char const m_target_periph_name[] = "ITS_Node"; /**< Name of the device we try to connect to. This name is searched in the scan report data*/

static serial_file_object_t m_file_object = {0};

static uint32_t m_counter_get = 0;

static void PrepareDataSent(void)
{
        uint32_t data_length = sizeof(nrf52);
        memset(&m_file_object, 0, sizeof(serial_file_object_t));
        m_file_object.filesize = data_length;
        m_file_object.crc32 = crc32_compute(nrf52, data_length, NULL);
        m_file_object.mtusize = m_ble_its_max_data_len;
        m_file_object.p_data = nrf52;
        NRF_LOG_INFO("File size = %08x", m_file_object.filesize);
        NRF_LOG_INFO("file object crc32 = %04x", m_file_object.crc32);
}

/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
        app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
        bsp_board_init(BSP_INIT_LEDS);
}

/**@brief Function to start scanning.
 */
static void scan_start(void)
{
        ret_code_t err_code;

        err_code = nrf_ble_scan_start(&m_scan);
        APP_ERROR_CHECK(err_code);
}

static void ble_its_c_evt_handler(ble_its_c_t *p_ble_its_c, ble_its_c_evt_t const *p_ble_its_evt)
{
        ret_code_t err_code;
        uint32_t receive_byte = 0;

        switch (p_ble_its_evt->evt_type)
        {
        case BLE_ITS_C_EVT_DISCOVERY_COMPLETE:
                NRF_LOG_DEBUG("ITS Service: Discovery complete.");
                err_code = ble_its_c_handles_assign(p_ble_its_c, p_ble_its_evt->conn_handle, &p_ble_its_evt->handles);
                APP_ERROR_CHECK(err_code);

                NRF_LOG_DEBUG("ITS Notification is enabled!!");
                err_code = ble_its_c_tx_notif_enable(p_ble_its_c);
                APP_ERROR_CHECK(err_code);

                NRF_LOG_DEBUG("ITS Image Notification is enabled !!!");
                err_code = ble_its_c_img_info_notif_enable(p_ble_its_c);
                APP_ERROR_CHECK(err_code);

                NRF_LOG_INFO("Connected to device with Nordic ITS Service.");
                break;

        case BLE_ITS_C_EVT_ITS_TX_EVT:
                NRF_LOG_INFO("ITS Receive the number of bytes = %04d", receive_byte);
                //NRF_LOG_DEBUG("BLE_ITS_C_EVT_ITS_TX_EVT %04d", receive_byte);
                break;

        case BLE_ITS_C_EVT_ITS_IMG_INFO_EVT:
                //NRF_LOG_INFO("ITS Image Info: the number of bytes = %04d", receive_byte);
                //NRF_LOG_DEBUG("BLE_ITS_C_EVT_ITS_IMG_INFO_EVT %04d", receive_byte);
                break;

        case BLE_ITS_C_EVT_ITS_RX_COMPLETE_EVT:
                NRF_LOG_INFO("RX COMPLETE");
                // m_counter_get = counter_get();
                counter_stop();
                uint32_t time_ms = counter_get();
                uint32_t bit_count = (m_file_object.filesize * 8);
                float throughput_kbps = ((bit_count / (time_ms / 1000.f)) / 1000.f);

                NRF_LOG_INFO("Done.");
                NRF_LOG_INFO("=============================");
                NRF_LOG_INFO("Time: %u.%.2u seconds elapsed.", (time_ms / 1000), (time_ms % 1000));
                NRF_LOG_INFO("Throughput: " NRF_LOG_FLOAT_MARKER " Kbps.",
                             NRF_LOG_FLOAT(throughput_kbps));
                NRF_LOG_INFO("=============================");
                NRF_LOG_INFO("Sent %u bytes of ATT payload.", m_file_object.filesize);
                NRF_LOG_INFO("Retrieving amount of bytes received from peer...");
                //                NRF_LOG_INFO("Data is sent!");

                break;

        case BLE_ITS_C_EVT_DISCONNECTED:
                NRF_LOG_INFO("Disconnected.");
                //scan_start();
                break;
        }
}

/**@brief Function for changing the tx power.
 */
static void tx_power_set(void)
{
        ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, TX_POWER_LEVEL);
        APP_ERROR_CHECK(err_code);
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
        // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
        // discovery, update LEDs status and resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED:
        {
                NRF_LOG_INFO("Connection 0x%x established, starting DB discovery.",
                             p_gap_evt->conn_handle);

                APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);

                err_code = ble_its_c_handles_assign(&m_ble_its_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle, NULL);
                APP_ERROR_CHECK(err_code);

                err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle], p_gap_evt->conn_handle);
                APP_ERROR_CHECK(err_code);
        }
        break;

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
                NRF_LOG_INFO("LBS central link 0x%x disconnected (reason: 0x%x)",
                             p_gap_evt->conn_handle,
                             p_gap_evt->params.disconnected.reason);

                if (ble_conn_state_central_conn_count() == 0)
                {
                        err_code = app_button_disable();
                        APP_ERROR_CHECK(err_code);

                        // Turn off the LED that indicates the connection.
                        bsp_board_led_off(CENTRAL_CONNECTED_LED);
                }

                scan_start();
        }
        break;

        case BLE_GAP_EVT_TIMEOUT:
        {
                // We have not specified a timeout for scanning, so only connection attemps can timeout.
                if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
                {
                        NRF_LOG_DEBUG("Connection request timed out.");
                }
        }
        break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
                // Accept parameters requested by peer.
                err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                        &p_gap_evt->params.conn_param_update_request.conn_params);
                APP_ERROR_CHECK(err_code);
        }
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

        case BLE_GATTC_EVT_TIMEOUT:
        {
                // Disconnect on GATT Client timeout event.
                NRF_LOG_DEBUG("GATT Client Timeout.");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
        }
        break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
                // Disconnect on GATT Server timeout event.
                NRF_LOG_DEBUG("GATT Server Timeout.");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
        }
        break;

        case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:
                //NRF_LOG_INFO("BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE");
                break;

        default:
                // No implementation needed.
                break;
        }
}

static void its_c_init(void)
{
        ret_code_t err_code;
        ble_its_c_init_t its_init;

        its_init.evt_handler = ble_its_c_evt_handler;
        for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
        {
                err_code = ble_its_c_init(&m_ble_its_c[i], &its_init);
                APP_ERROR_CHECK(err_code);
        }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
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

        ble_cfg_t ble_cfg;
        // Configure the GATTS attribute table.
        memset(&ble_cfg, 0x00, sizeof(ble_cfg));
        ble_cfg.gap_cfg.role_count_cfg.periph_role_count = NRF_SDH_BLE_PERIPHERAL_LINK_COUNT;
        ble_cfg.gap_cfg.role_count_cfg.central_role_count = NRF_SDH_BLE_CENTRAL_LINK_COUNT;
        ble_cfg.gap_cfg.role_count_cfg.qos_channel_survey_role_available = false; /* Enable channel survey role */

        err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, &ram_start);
        if (err_code != NRF_SUCCESS)
        {
                NRF_LOG_ERROR("sd_ble_cfg_set() returned %s when attempting to set BLE_GAP_CFG_ROLE_COUNT.",
                              nrf_strerror_get(err_code));
        }

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
        case LEDBUTTON_BUTTON_PIN:
                break;

        case SEND_INFO_BUTTON_PIN:
                if (button_action == APP_BUTTON_PUSH)
                {
                        NRF_LOG_INFO("SEND_INFO_BUTTON_PIN");
                        PrepareDataSent();

                        counter_start();
                        err_code = ble_its_c_send_object(&m_ble_its_c[0], nrf52, sizeof(nrf52), m_ble_its_max_data_len);
                        APP_ERROR_CHECK(err_code);
                }
                break;

        default:
                APP_ERROR_HANDLER(pin_no);
                break;
        }
}

/**@brief Function for handling Scaning events.
 *
 * @param[in]   p_scan_evt   Scanning event.
 */
static void scan_evt_handler(scan_evt_t const *p_scan_evt)
{
        ret_code_t err_code;

        switch (p_scan_evt->scan_evt_id)
        {

        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
                err_code = p_scan_evt->params.connecting_err.err_code;
                APP_ERROR_CHECK(err_code);
                break;

        case NRF_BLE_SCAN_EVT_CONNECTED:
        {
                ble_gap_evt_connected_t const *p_connected =
                    p_scan_evt->params.connected.p_connected;
                // Scan is automatically stopped by the connection.
                NRF_LOG_INFO("Connecting to target 0x%02x%02x%02x%02x%02x%02x",
                             p_connected->peer_addr.addr[0],
                             p_connected->peer_addr.addr[1],
                             p_connected->peer_addr.addr[2],
                             p_connected->peer_addr.addr[3],
                             p_connected->peer_addr.addr[4],
                             p_connected->peer_addr.addr[5]);
        }
        break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
                NRF_LOG_INFO("Scan timed out.");
                scan_start();
        }
        break;
        default:
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
                {LEDBUTTON_BUTTON_PIN, false, BUTTON_PULL, button_event_handler},
                {SEND_INFO_BUTTON_PIN, false, BUTTON_PULL, button_event_handler},
            };

        err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                                   BUTTON_DETECTION_DELAY);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t *p_evt)
{
        NRF_LOG_DEBUG("call to ble_lbs_on_db_disc_evt for instance %d and link 0x%x!",
                      p_evt->conn_handle,
                      p_evt->conn_handle);
        ble_its_c_on_db_disc_evt(&m_ble_its_c[p_evt->conn_handle], p_evt);
}

/**@brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
        ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the log.
 */
static void log_init(void)
{
        ret_code_t err_code = NRF_LOG_INIT(NULL);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing the timer.
 */
static void timer_init(void)
{
        ret_code_t err_code = app_timer_init();
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Power manager. */
static void power_management_init(void)
{
        ret_code_t err_code;
        err_code = nrf_pwr_mgmt_init();
        APP_ERROR_CHECK(err_code);
}

static void scan_init(void)
{
        ret_code_t err_code;
        nrf_ble_scan_init_t init_scan;

        memset(&init_scan, 0, sizeof(init_scan));

        init_scan.connect_if_match = true;
        init_scan.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;

        err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
        APP_ERROR_CHECK(err_code);

        // Setting filters for scanning.
        err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt)
{
        if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
        {
                NRF_LOG_INFO("ATT MTU exchange completed.");
                m_ble_its_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_its_max_data_len, m_ble_its_max_data_len);
        }
        else if (p_evt->evt_id == NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED)
        {
                //m_ble_its_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                NRF_LOG_INFO("Data length updated to %u bytes.", p_evt->params.data_length);
        }
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
        ret_code_t err_code;

        err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details Handle any pending log operation(s), then sleep until the next event occurs.
 */
static void idle_state_handle(void)
{
        app_sched_execute();
        if (NRF_LOG_PROCESS() == false)
        {
                nrf_pwr_mgmt_run();
        }
}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
        APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

int main(void)
{
        ret_code_t err_code;
        // Initialize.
        log_init();
        timer_init();
        counter_init();
        leds_init();
        buttons_init();
        power_management_init();
        ble_stack_init();
        scheduler_init();
        scan_init();
        gatt_init();
        db_discovery_init();
        its_c_init();

        // Start execution.
        NRF_LOG_INFO("Example : Multi-link Central LBS + NUS");
        scan_start();

        err_code = app_button_enable();
        APP_ERROR_CHECK(err_code);

        // Enter main loop.
        for (;;)
        {
                idle_state_handle();
        }
}
