/* 
*
*  BLE custom service example  based on Nordic app template code
*
*
*  Created by : Wesley Marques
*/




#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdm.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "nrf_fstorage.h"
#include "fds.h"
#include "peer_manager.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_backend_rtt.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "boards.h"
#include "SEGGER_RTT.h"
#include "mycustom_service.h"


#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2    /**< Reply when unsupported features are requested. */

#define DEVICE_NAME                     "MyDeviceSDK14"                       /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "MyCustom Project"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL_FAST           300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_TIMEOUT_IN_SECONDS_FAST 0                                     /**< The advertising timeout in units of seconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (5 seconds). */
#define SLAVE_LATENCY                   4                                      /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(32000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(1000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(10000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_BLE_CONN_CFG_TAG        1                                   /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO       1                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_SOC_OBSERVER_PRIO       1                                   /**< Applications' SoC observer priority. You shoulnd't need to modify this value. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static uint16_t       m_conn_handle = BLE_CONN_HANDLE_INVALID;                  /**< Handle of the current connection. */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);  


static uint8_t custom_value          = 0;
static int timeCounter = 0;
static int maxCounter = 0;
#define TIMER_INTERVAL APP_TIMER_TICKS(1000)   

APP_TIMER_DEF(m_app_timer_id);

#ifdef BSP_BUTTON_0
    #define PIN_IN BSP_BUTTON_0
#endif

#ifndef PIN_IN
    #error "Please indicate input pin"
#endif

#ifdef BSP_LED_0
    #define PIN_OUT_1 BSP_LED_1
    #define PIN_OUT_2 BSP_LED_2
    #define PIN_OUT_3 BSP_LED_3
    #define PIN_OUT_4 BSP_LED_4
#endif

#ifndef PIN_OUT_1
    #error "Please indicate output pin"
#endif


// YOUR_JOB: Use UUIDs for service(s) used in your application.
//static ble_uuid_t m_adv_uuids[] = {{CUSTOM_SERVICE_UUID, m_cus.uuid_type }}; /**< Universally unique service identifiers. */
// ble_uuid_t m_adv_uuids[] = {{CUSTOM_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN }}; /**< Universally unique service identifiers. */
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {BLE_UUID_CUSTOM_SERVICE, BLE_UUID_TYPE_BLE}
};
uint8_t char2_data[CHARACTERISTIC_SIZE];
 
static void advertising_start(bool erase_bonds);
static void application_timers_start(void);
static void application_timers_stop(void);



void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{    
   NRF_LOG_DEBUG("CLICKED BSP_BUTTON_0");   
}


static void gpio_pin_high(int pin) {
    nrf_drv_gpiote_out_set(pin);    
    nrf_drv_gpiote_out_toggle(pin);
}

static void gpio_pin_low(int pin) {
    nrf_drv_gpiote_out_clear(pin);    
    nrf_drv_gpiote_out_toggle(pin);     
}

static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
        
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(BSP_LED_1, &out_config);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_IN, true);

}


/**  MY CUSTOM FUNCTIONS **/
void update_data()
{
    //Send back some data
    for(int i=1;i<CHARACTERISTIC_SIZE;i++){
        char2_data[i] = 0;
    }
    char2_data[0] = maxCounter;
    //char2_data[1] = timeCounter;
    ret_code_t err_code = mycustom_service_update_data(m_conn_handle,char2_data);
    APP_ERROR_CHECK(err_code);
   
}  

void timer_signal_handler(void* p_context)
{ 
    update_data();

    timeCounter++;  
    SEGGER_RTT_printf(0, "TIMER %d\n", timeCounter); 

    if (timeCounter%2 == 0) {
        gpio_pin_high(PIN_OUT_1);
    } else {
        gpio_pin_low(PIN_OUT_1);
    }

    if (timeCounter >= maxCounter) {
        application_timers_stop();
        gpio_pin_low(PIN_OUT_1);
    }

    
}

  

static void on_ble_evt(ble_evt_t * p_ble_evt)
{
   
    uint8_t i=0;
    switch (p_ble_evt->header.evt_id)
            {

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_DEBUG("BLE_GAP_EVT_CONNECTED");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_DEBUG("BLE_GAP_EVT_DISCONNECTED");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        case BLE_GAP_EVT_TIMEOUT:
            NRF_LOG_DEBUG("BLE_GAP_EVT_TIMEOUT");
        case BLE_GATTS_EVT_WRITE:
            NRF_LOG_DEBUG("BLE_GATTS_EVT_WRITE");
            for(i=0;i<p_ble_evt->evt.gatts_evt.params.write.len;i++) 
            {   
                uint8_t signal = p_ble_evt->evt.gatts_evt.params.write.data[i];
                NRF_LOG_DEBUG("Data Receipt %d: 0x%x", i, signal);
                maxCounter = signal;
                timeCounter = 0;
                application_timers_start(); 
            } 
        default:
            // No implementation needed.
            break;
       }     
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            SEGGER_RTT_printf(0,"\nConnected to a previously bonded device.\r\n");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            SEGGER_RTT_printf(0,"\nConnection secured: role: %d, conn_handle: 0x%x, procedure: %d.\r\n",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            SEGGER_RTT_printf(0,"\nPM_EVT_CONN_SEC_FAILED");
    
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {   
            SEGGER_RTT_printf(0,"\nPM_EVT_CONN_SEC_CONFIG_REQ");
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                SEGGER_RTT_printf(0,"\nPM_EVT_STORAGE_FULL GC FAILED");
            } else {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {   
            SEGGER_RTT_printf(0,"\nPM_EVT_PEERS_DELETE_SUCCEEDED");
            advertising_start(false);
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            SEGGER_RTT_printf(0,"\nPM_EVT_LOCAL_DB_CACHE_APPLY_FAILED");
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            SEGGER_RTT_printf(0,"\nPM_EVT_PEER_DATA_UPDATE_FAILED");
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            SEGGER_RTT_printf(0,"\nPM_EVT_PEER_DELETE_FAILED");
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            SEGGER_RTT_printf(0,"\nPM_EVT_PEERS_DELETE_FAILED");
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}



/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);  
    // Create timers.
    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_signal_handler);
    APP_ERROR_CHECK(err_code); 

    // err_code = app_timer_create(&m_app_timer_id2, APP_TIMER_MODE_REPEATED, timer_update_data_handler);
    // APP_ERROR_CHECK(err_code); 
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;
    err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code); 
    timeCounter = 0;
    SEGGER_RTT_printf(0, "application_timers_start ... (Err_code: %d).\r\n",err_code); 

}

static void application_timers_stop(void)
{
    uint32_t err_code;
    err_code = app_timer_stop(m_app_timer_id);
    APP_ERROR_CHECK(err_code); 

    SEGGER_RTT_printf(0, "application_timers_stop ... (Err_code: %d).\r\n",err_code); 
    timeCounter = 0;

   
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    err_code = mycustom_service_init();
    APP_ERROR_CHECK(err_code);
    
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    // ret_code_t err_code;

    // if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    // {
    //     err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    //     APP_ERROR_CHECK(err_code);
    // }
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
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            SEGGER_RTT_printf(0,"\nBLE_ADV_EVT_FAST.\r\n");
            break;

        case BLE_ADV_EVT_SLOW:
            SEGGER_RTT_printf(0,"\nBLE_ADV_EVT_SLOW.\r\n");
            break;    

        case BLE_ADV_EVT_IDLE:
            SEGGER_RTT_printf(0,"\nBLE_ADV_EVT_IDLE.\r\n");
            //GO TO SLEEP MODE
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_handler(ble_evt_t * p_ble_evt, void * p_context)
{

    ret_code_t  err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        {
            SEGGER_RTT_printf(0,"\nConnected.");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        }  break;        
        case BLE_GAP_EVT_ADV_REPORT:
        {
            SEGGER_RTT_printf(0,"\nBLE_GAP_EVT_ADV_REPORT.");
        } break; // BLE_GAP_EVT_ADV_REPORT

        case BLE_GAP_EVT_DISCONNECTED:
        {
            SEGGER_RTT_printf(0,"\n BLE_GAP_EVT_DISCONNECTED Disconnected, reason 0x%x.",
                         p_ble_evt->evt.gap_evt.params.disconnected.reason);

    
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                SEGGER_RTT_printf(0,"\nScan timed out.");
                //scan_start();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
               SEGGER_RTT_printf(0,"\nConnection Request timed out.");
            } else {
                SEGGER_RTT_printf(0,"\nBLE_GAP_EVT_TIMEOUT");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            SEGGER_RTT_printf(0,"\nGATT Client Timeout.");
            // err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
            //                                  BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            SEGGER_RTT_printf(0,"\nGATT Server Timeout.");
            // err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
            //                                  BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
    
    on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt, void * p_context)
{
    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt, &m_advertising);

    switch (sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
            /* fall through */
        case NRF_EVT_FLASH_OPERATION_ERROR:

            break;

        default:
            // No implementation needed.
            break;
    }
}

void softdevice_assert_callback(uint32_t id, uint32_t pc, uint32_t info)
{
   SEGGER_RTT_printf(0, "softdevice_assert_callback \n");
}
/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    // err_code = sd_softdevice_enable(&clock_lf_cfg, softdevice_assert_callback);
    // SEGGER_RTT_printf(0, "-->sd_softdevice_enable %d\n", err_code);
    // APP_ERROR_CHECK(err_code);
    err_code = nrf_sdh_enable_request();
    SEGGER_RTT_printf(0, "-->nrf_sdh_enable_request %d\n", err_code);
    APP_ERROR_CHECK(err_code);
    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);
    SEGGER_RTT_printf(0, "-->nrf_sdh_ble_default_cfg_set %d\n", err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
    SEGGER_RTT_printf(0, "-->nrf_sdh_ble_enable %d\n", err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    SEGGER_RTT_printf(0, "-->NRF_SDH_BLE_OBSERVER %d\n", err_code);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_DEBUG("Erase bonds!\r\n");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
   ret_code_t             err_code;
    ble_advertising_init_t init;

    NRF_LOG_DEBUG("advertising_init!\r\n");

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL_FAST;
    init.config.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS_FAST;
   
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

    NRF_LOG_DEBUG("advertising_init END!\r\n");
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{   
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    nrf_log_backend_rtt_init();

}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;
    uint32_t err_code;

    // Initialize.
    log_init();

    SEGGER_RTT_printf(0, "\n");
    SEGGER_RTT_printf(0,"=======================\n");
    SEGGER_RTT_printf(0,"======My Custom BLE Device====\n");
    SEGGER_RTT_printf(0,DEVICE_NAME);
    SEGGER_RTT_printf(0, "\n");
    SEGGER_RTT_printf(0,"=======================\n");

    SEGGER_RTT_printf(0,"->timers_init().\n");
    timers_init();
    SEGGER_RTT_printf(0,"->ble_stack_init().\n");
    ble_stack_init();
    SEGGER_RTT_printf(0,"->gap_params_init().\n");
    gap_params_init();
    SEGGER_RTT_printf(0,"->gatt_init().\n");
    gatt_init();
    SEGGER_RTT_printf(0,"->advertising_init().\n");
    advertising_init();
    SEGGER_RTT_printf(0,"->services_init().\n");
    services_init();
    SEGGER_RTT_printf(0,"->conn_params_init().\n");
    conn_params_init();
    SEGGER_RTT_printf(0,"->peer_manager_init().\n");
    peer_manager_init();
    SEGGER_RTT_printf(0,"->gpio_init().\n");
    gpio_init();
    
    // Start execution.
  
   
    advertising_start(erase_bonds);
 
    SEGGER_RTT_printf(0,"\n");
    SEGGER_RTT_printf(0,"=======================\n");
    SEGGER_RTT_printf(0,"===My Custom BLE Device initialized==\n");
    SEGGER_RTT_printf(0,"=======================\n");  

     // Enter main loop.
    for (;;)
    {   
        power_manage();
      
    }
}


/**
 * @}
 */