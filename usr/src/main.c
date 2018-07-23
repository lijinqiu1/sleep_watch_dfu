/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
/** @example examples/ble_peripheral/ble_app_hrs/main.c
 *
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_nus.h"
#ifdef BLE_DFU_APP_SUPPORT
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#endif // BLE_DFU_APP_SUPPORT
#include "ble_conn_params.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
#include "nrf_delay.h"

#include "lis3dh_driver.h"
#include "led.h"
#include "event.h"
#include "button.h"
#include "adc.h"
#include "pwm.h"
#include "calender.h"
#include "tilt.h"
#include "battery.h"
#include "queue.h"
#include "pwm.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define DEVICE_NAME                      "Pregn"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE            BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */
#define APP_ADV_INTERVAL                 64                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       30                                        /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS             (4)                  /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL      APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                81                                         /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                100                                        /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT          1                                          /**< Increment between each simulated battery level measurement. */

#define HEART_RATE_MEAS_INTERVAL         APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER) /**< Heart rate measurement interval (ticks). */
#define MIN_HEART_RATE                   140                                        /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                   300                                        /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT             10                                         /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

#define RR_INTERVAL_INTERVAL             APP_TIMER_TICKS(300, APP_TIMER_PRESCALER)  /**< RR interval interval (ticks). */
#define MIN_RR_INTERVAL                  100                                        /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL                  500                                        /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT            1                                          /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define SENSOR_CONTACT_DETECTED_INTERVAL APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Sensor Contact Detected toggle interval (ticks). */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(20, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(75, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#ifdef BLE_DFU_APP_SUPPORT
#define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                    0x01                                       /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
#define APP_SERVICE_HANDLE_START         0x000C                                     /**< Handle of first application specific service when when service changed characteristic is present. */
#define BLE_HANDLE_MAX                   0xFFFF                                     /**< Max handle value in BLE. */

STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                                     /** When having DFU Service support in application the Service Changed Characteristic should always be present. */
#endif // BLE_DFU_APP_SUPPORT


static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
static ble_nus_t                         m_nus;                                      /**< Structure to identify the Nordic UART Service. */


static dm_application_instance_t         m_app_handle;                              /**< Application identifier allocated by device manager */

static ble_uuid_t                        m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */
#ifdef BLE_DFU_APP_SUPPORT    
static ble_dfu_t                         m_dfus;                                    /**< Structure used to identify the DFU service. */
#endif // BLE_DFU_APP_SUPPORT    


//周期性时间
static app_timer_id_t                    m_period_timer_id;

typedef enum
{
	CMD_SET_TIME = 0x01,
	CMD_REQUEST_DATA,
	CMD_SEND_DATA,
	CMD_SEND_DATA_COMPLETED,
	CMD_SET_ALARM,
	CMD_DEVICE_BOND,
	CMD_GET_BATTERY,
}MESSAGE_CMD_ID_t;

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
	app_trace_log("error 0x%x,line %d, %s\n",error_code, line_num, p_file_name);
	while(1);
}
//报文处理函数
static void message_process(uint8_t *ch)
{
	uint8_t cmd_id;
	UTCTimeStruct tm;
	uint32_t err_code;
	uint8_t data_array[20];
    uint16_t battery;
	if (ch[0] != 0xA5)
	{
		return ;
	}
	cmd_id = ch[2];

	switch(cmd_id)
	{
	case CMD_SET_TIME:
		//设置时间
		tm.year = 2000 + ch[3];
		tm.month = ch[4];
		tm.day = ch[5];
		tm.hour = ch[6];
		tm.minutes = ch[7];
		tm.seconds = ch[8];
		TimeSeconds = ConvertUTCSecs(&tm);
		//发送响应报文
		data_array[0] = 0xA5;
		data_array[1] = 0x01;
		data_array[2] = CMD_SET_TIME;
		data_array[3] = 0x80;
		err_code = ble_nus_string_send(&m_nus, data_array, 4);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
		break;
	case CMD_REQUEST_DATA:
		//开始上传数据
		g_event_status |= EVENT_DATA_SENDING;
		break;
	case CMD_SET_ALARM:
		//设置干涉条件
		//干涉条件有6个
		memcpy(system_params.time,&ch[3],0x06);
		//发送响应报文
		data_array[0] = 0xA5;
		data_array[1] = 0x01;
		data_array[2] = CMD_SET_ALARM;
		data_array[3] = 0x80;
		err_code = ble_nus_string_send(&m_nus, data_array, 4);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
		system_params_save(&system_params);
		break;
	case CMD_DEVICE_BOND:

		if(ch[3] == 0x01)
		{
			if(system_params.device_bonded != 1)
			{
				//设备绑定
				system_params.device_bonded = 0x01;
				memcpy((char *)system_params.mac_add,&ch[4],11);
				app_trace_log("save phone :");
				app_trace_dump(system_params.mac_add,11);
				system_params_save(&system_params);
				g_status_bond_info_received = true;
				//发送响应报文
				data_array[0] = 0xA5;
				data_array[1] = 0x01;
				data_array[2] = CMD_DEVICE_BOND;
				data_array[3] = 0x80;
				err_code = ble_nus_string_send(&m_nus, data_array, 4);
		        if (err_code != NRF_ERROR_INVALID_STATE)
		        {
		            APP_ERROR_CHECK(err_code);
		        }
			}
			else
			{
				if(memcmp(&ch[4],(char *)system_params.mac_add,11)==0)
				{
					g_status_bond_info_received = true;
				}
			}
		}
		else if (ch[3] == 0x02)
		{
			//设备解绑
			system_params.device_bonded = 0x00;
			memset((char *)system_params.mac_add,0xFF,11);
			system_params_save(&system_params);
			//发送响应报文
			data_array[0] = 0xA5;
			data_array[1] = 0x01;
			data_array[2] = CMD_DEVICE_BOND;
			data_array[3] = 0x80;
			err_code = ble_nus_string_send(&m_nus, data_array, 4);
	        if (err_code != NRF_ERROR_INVALID_STATE)
	        {
	            APP_ERROR_CHECK(err_code);
	        }
		}
		break;
	case CMD_GET_BATTERY:
		data_array[0] = 0xA5;
		data_array[1] = 0x02;
		data_array[2] = CMD_GET_BATTERY;
        battery = battery_get_value();
        if(battery < BATTER_VALUE_20)
        {
            data_array[3] = 0;
        }
        else if(battery < BATTER_VALUE_40)
        {
            data_array[3] = 20;
        }
        else if(battery < BATTER_VALUE_60)
        {
            data_array[3] = 40;
        }
        else if(battery < BATTER_VALUE_80)
        {
            data_array[3] = 60;
        }
        else if(battery < BATTER_VALUE_100)
        {
            data_array[3] = 80;
        }
        else
        {
            data_array[3] = 100;
        }
		data_array[4] = 0x80;
		err_code = ble_nus_string_send(&m_nus, data_array, 5);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
		break;
	default:
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



/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code = 0;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
	err_code = app_timer_create(&m_period_timer_id,
		                        APP_TIMER_MODE_REPEATED,
		                        period_cycle_process_handler);
	APP_ERROR_CHECK(err_code);
	
	err_code = app_timer_create(&m_pwm_timer_id,
		                        APP_TIMER_MODE_REPEATED,
		                        pwm_cycle_process_handler);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

//    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
//    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
	app_trace_dump(p_data,length);
	if ((p_data[0] == 0xA5) && (p_data[length - 1] == 0x80))
	{
		//memcpy(rec_data_buffer,p_data,length);
        //rec_data_length = length;
        queue_message_push(p_data);
		g_event_status |= EVENT_MESSAGE_RECEIVED;
	}
}

#ifdef BLE_DFU_APP_SUPPORT
/**@brief Function for stopping advertising.
 */
static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for loading application-specific context after establishing a secure connection.
 *
 * @details This function will load the application context and check if the ATT table is marked as 
 *          changed. If the ATT table is marked as changed, a Service Changed Indication
 *          is sent to the peer if the Service Changed CCCD is set to indicate.
 *
 * @param[in] p_handle The Device Manager handle that identifies the connection for which the context 
 *                     should be loaded.
 */
static void app_context_load(dm_handle_t const * p_handle)
{
    uint32_t                 err_code;
    static uint32_t          context_data;
    dm_application_context_t context;

    context.len    = sizeof(context_data);
    context.p_data = (uint8_t *)&context_data;

    err_code = dm_application_context_get(p_handle, &context);
    if (err_code == NRF_SUCCESS)
    {
        // Send Service Changed Indication if ATT table has changed.
        if ((context_data & (DFU_APP_ATT_TABLE_CHANGED << DFU_APP_ATT_TABLE_POS)) != 0)
        {
            err_code = sd_ble_gatts_service_changed(m_conn_handle, APP_SERVICE_HANDLE_START, BLE_HANDLE_MAX);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
            {
                APP_ERROR_HANDLER(err_code);
            }
        }

        err_code = dm_application_context_delete(p_handle);
        APP_ERROR_CHECK(err_code);
    }
    else if (err_code == DM_NO_APP_CONTEXT)
    {
        // No context available. Ignore.
    }
    else
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/** @snippet [DFU BLE Reset prepare] */
/**@brief Function for preparing for system reset.
 *
 * @details This function implements @ref dfu_app_reset_prepare_t. It will be called by 
 *          @ref dfu_app_handler.c before entering the bootloader/DFU.
 *          This allows the current running application to shut down gracefully.
 */
static void reset_prepare(void)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);

    nrf_delay_ms(500);
}
/** @snippet [DFU BLE Reset prepare] */
#endif // BLE_DFU_APP_SUPPORT


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
	uint32_t       err_code;
    ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [DFU BLE Service initialization] */
    ble_dfu_init_t   dfus_init;

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.error_handler = NULL;
    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.revision      = DFU_REVISION;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);

    dfu_app_reset_prepare_set(reset_prepare);
    dfu_app_dm_appl_instance_set(m_app_handle);
    /** @snippet [DFU BLE Service initialization] */
#endif // BLE_DFU_APP_SUPPORT
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;
	UTCTimeStruct utc;    
	utc.seconds = 0x00;  
	utc.minutes = 0x00;  
	utc.hour    = 0x00;  
	utc.day     = 0x00; 
	utc.month   = 0x00;  
	utc.year    = 2000;  
	Set_Clock( ConvertUTCSecs( &utc ) );  
    // Start application timers.
	
	err_code = app_timer_start(m_period_timer_id,APP_TIMER_TICKS(1000,APP_TIMER_PRESCALER),NULL);
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
    uint32_t err_code;

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
    uint32_t               err_code;
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
//static void sleep_mode_enter(void)
//{
//    uint32_t err_code;

//    // Go to system-off mode (this function will not return; wakeup will cause a reset).
//    err_code = sd_power_system_off();
//    APP_ERROR_CHECK(err_code);
//}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
//    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;
        case BLE_ADV_EVT_IDLE:
//            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = 0;

    switch (p_ble_evt->header.evt_id)
            {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			g_event_status |= EVENT_BLE_CONNECTED;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
			g_event_status |= EVENT_BLE_DISCONNECTED;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
		case BLE_GAP_EVT_TIMEOUT:
			if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
			{
                g_status_adv = false;
				app_trace_log("adv timeout!\n");
			}
			break;
        default:
            // No implementation needed.
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
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [Propagating BLE Stack events to DFU Service] */
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    /** @snippet [Propagating BLE Stack events to DFU Service] */
#endif // BLE_DFU_APP_SUPPORT
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

#if defined(S110) || defined(S130) || defined(S310)
    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#if defined(S130) || defined(S310)
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
#endif

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);

#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }
#endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
	float Tilt;
    bool erase_bonds = false;
    uint8_t data_array_temp[20];
    queue_items_t item;

    // Initialize.
    timers_init();
    ble_stack_init();
	app_trace_init();
    device_manager_init(erase_bonds);
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
	
	LIS3DH_Init();
	button_init();
	leds_init();
	battery_init();
	battery_get_value();
    battery_manager();
	//flash初始化
	queue_init();
    //通信报文缓存队列
    queue_message_init();
	//马达驱动初始化
    alarm_init();
	
    // Start execution.
    application_timers_start();
//    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
//    APP_ERROR_CHECK(err_code);
	app_trace_log("start!\n");
	
//	uint8_t data;
//	LIS3DH_GetWHO_AM_I(&data);
//	app_trace_log("who am i %x\n",data);
//	pwm_moto_test();
//	queue_test();
    // Enter main loop.
    for (;;)
    {
		if (g_event_status & EVENT_LIS3DH_VALUE)
		{
			Tilt = calculateTilt();
			sleep_manage(Tilt);
			//存储角度值
			if (g_event_status & EVENT_TILT_PUSH)
			{
				save_Tilt(Tilt);
				g_event_status &= ~ EVENT_TILT_PUSH;
			}
			g_event_status &= ~EVENT_LIS3DH_VALUE;
		}
		if (g_event_status & EVENT_KEY_PRESS_SHOT)
		{
			if ((g_status_work != true) && (battery_value >= BATTERY_VALUE_LOW))
			{
				if (g_status_adv == true)
				{
					g_event_status |= EVENT_ADV_STOP;
                    if(battery_get_charege_status() == BATTERY_NOT_CHARGE)
					    g_event_status |= EVENT_BEGIN_WORK;
                    app_trace_log("adv stop %d\n",__LINE__);
				}
				else if ((g_status_ble_connect == true)
					&&(g_status_data_send != true))
				{
					g_event_status |= EVENT_BLE_SHUT_CONNECT;
					if (battery_get_charege_status() == BATTERY_NOT_CHARGE)
						g_event_status |= EVENT_BEGIN_WORK;
                    app_trace_log("connect true %d\n",__LINE__);
				}
                else if (battery_get_charege_status() == BATTERY_NOT_CHARGE)
                {
                    g_event_status |= EVENT_BEGIN_WORK;
                }
			}
			else
			{
				g_event_status |= EVENT_END_WORK;
			}
			g_event_status &= ~EVENT_KEY_PRESS_SHOT;

		}
		else if(g_event_status & EVENT_KEY_PRESS_LONG)
		{
			if ((g_status_work != true) &&
					(g_status_ble_connect != true) &&
					(g_status_adv    != true))
			{
				g_event_status |= EVENT_ADV_START;
                app_trace_log("adv start \n");
			}
			g_event_status &= ~(EVENT_KEY_PRESS_LONG);
		}
        if (g_event_status & EVENT_BEGIN_WORK)
        {//开始工作
			app_trace_log("device_bonded : %d\n",system_params.device_bonded);
            if (system_params.device_bonded == 1)
            {
            	leds_process_init(LED_WORK_BEGIN);
    			// begin to work
    			g_status_work = true;
    			//初始化角度值
    			g_status_tilt_init_flag = true;
                app_trace_log("work start \n");
            }
            g_event_status &= ~(EVENT_BEGIN_WORK);
        }

        if (g_event_status & EVENT_END_WORK)
        {//停止工作
            leds_process_init(LED_WORK_END);
			// stop work
			g_status_work = false;
			//初始化角度值
			g_status_tilt_init_flag = false;
            if (g_status_alarm_status)
            {
                g_status_alarm_status = false;
				alarm_case();
            }
			//数据发送完成同步队列信息
			g_event_status |= EVENT_DATA_SYNC;
            g_event_status &= ~(EVENT_END_WORK);
            app_trace_log("work stop \n");
        }

        if (g_event_status & EVENT_DATA_SENDING)
        {//开始发送数据
    		g_status_data_send = true;
            leds_process_init(LED_WORK_BLE_DATA_SENDING);
            g_event_status &= ~(EVENT_DATA_SENDING);
        }

		if (g_event_status & EVENT_MESSAGE_RECEIVED)
		{//有数据接收
		    if (queue_message_pop(rec_data_buffer))
		    {
			    g_event_status &= ~(EVENT_MESSAGE_RECEIVED);
            }
			else
			{
				message_process(rec_data_buffer);
    		    if (rec_data_buffer[2] == 0x02)
    		    {
                    app_trace_log("begin send data\n");
                }
			}
		}

		if (g_event_status & EVENT_ADV_START)
		{//开始广播
			if ((g_status_work != true) && (g_status_adv!= true))
			{
				ble_advertising_start(BLE_ADV_MODE_FAST);
				leds_process_init(LED_WORK_BLE_ADV);
				g_status_adv = true;
			}
			g_event_status &= ~(EVENT_ADV_START);
		}

		if (g_event_status & EVENT_ADV_STOP)
		{//停止广播
		    if (g_status_adv)
		    {
    			err_code = sd_ble_gap_adv_stop();
    			APP_ERROR_CHECK(err_code);
    			g_status_adv = false;
		    }
			g_event_status &= ~(EVENT_ADV_STOP);
		}

		if (g_event_status & EVENT_BLE_SHUT_CONNECT)
		{//关闭蓝牙连接
			#if !defined(DEBUG_APP)
            
			err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
			APP_ERROR_CHECK(err_code);
			#endif
			app_trace_log("%s %d EVENT_BLE_SHUT_CONNECT\r\n",__FUNCTION__,__LINE__);
			g_event_status &= ~EVENT_BLE_SHUT_CONNECT;
		}

		if (g_event_status & EVENT_BLE_CONNECTED)
		{//蓝牙连接
			g_status_adv = false;
			g_status_ble_connect = true;
			leds_process_init(LED_WORK_BLE_CONNECTED);
			g_event_status &= ~EVENT_BLE_CONNECTED;
            app_trace_log("%s %d connected \n",__FUNCTION__,__LINE__);
		}
		if (g_event_status & EVENT_BLE_DISCONNECTED)
		{//蓝牙连接断开
			g_status_ble_connect = false;
			g_event_status &= ~EVENT_BLE_DISCONNECTED;
            app_trace_log("%s %d disconnected \n",__FUNCTION__,__LINE__);
		}
		if (g_event_status & EVENT_DATA_SYNC)
		{//数据同步
			queue_sync();
			g_event_status &= ~EVENT_DATA_SYNC;
		}

        if (g_event_status & EVENT_BATTRY_VALUE)
        {//更新电池电量
            battery_get_value();
            g_event_status &= ~EVENT_BATTRY_VALUE;
        }

		if (g_event_status & EVENT_DATA_FULL)
		{
			leds_process_init(LED_WORK_DATA_FULL);
			g_event_status &= ~EVENT_DATA_FULL;
		}
		
		if (g_event_status & EVENT_KEY_RESET)
		{
			app_trace_log("EVENT_KEY_RESET g_event_status 0x%x\n",g_event_status);
			system_params.device_bonded = 0x00;
			system_params.mac_add[0] = 0xff;
			system_params.mac_add[1] = 0xff;
			system_params.mac_add[2] = 0xff;
			system_params.mac_add[3] = 0xff;
			system_params.mac_add[4] = 0xff;
			system_params.mac_add[5] = 0xff;
			system_params.mac_add[6] = 0xff;
			system_params.mac_add[7] = 0xff;
			system_params.mac_add[8] = 0xff;
			system_params.mac_add[9] = 0xff;
			system_params.mac_add[10] = 0xff;
            system_params_save(&system_params);
			g_event_status &= ~EVENT_KEY_RESET;
		}
		//数据发送
		if (g_status_data_send == true)
		{//开始发送数据
			if (g_status_ble_connect == true)
			{
				if (queue_pop(&item))
				{
					//发送完成
					g_status_data_send = false;
                    leds_process_cancel();
					g_event_status |= EVENT_DATA_SENDED;
					//发送传输完成报文
					data_array_temp[0] = 0xA5;
					data_array_temp[1] = 0x01;
					data_array_temp[2] = CMD_SEND_DATA_COMPLETED;
					data_array_temp[3] = 0x80;
					app_trace_dump(data_array_temp,4);
					err_code = ble_nus_string_send(&m_nus, data_array_temp, 4);
			        if (err_code != NRF_ERROR_INVALID_STATE)
			        {
			            APP_ERROR_CHECK(err_code);
			        }
					//数据发送完成同步队列信息
					g_event_status |= EVENT_DATA_SYNC;
                    app_trace_log("data send end\n");
				}
				else
				{
					data_array_temp[0] = 0xA5;
					data_array_temp[1] = 0x09;
					data_array_temp[2] = CMD_SEND_DATA;
					data_array_temp[3] = item.year;
					data_array_temp[4] = item.mon;
					data_array_temp[5] = item.day;
					data_array_temp[6] = item.hour;
					data_array_temp[7] = item.min;
					data_array_temp[8] = item.second;
					data_array_temp[9] = (uint8_t)(item.angle & 0x00ff);
					data_array_temp[10] = (uint8_t)((item.angle >> 8) & 0x00ff);
					data_array_temp[11] = 0x80;
					//app_trace_dump(data_array_temp,12);
					app_trace_log("%d-%d-%d:%d-%d-%d\n",item.year,item.mon,item.day,item.hour,item.min,item.second);
					err_code = ble_nus_string_send(&m_nus, data_array_temp, 12);
			        while (err_code != NRF_SUCCESS)
			        {
			            err_code = ble_nus_string_send(&m_nus, data_array_temp, 12);
			        }
					nrf_delay_ms(100);
                    leds_process_flash_fast();
                    app_trace_log("data sending\n");
				}
			}
			else
			{
				g_status_data_send = false;
			}
		}
        power_manage();
    }
}
