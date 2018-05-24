#ifndef __LED_H__
#define __LED_H__

#ifdef LEG_DEBUG_LOG
#deifne LED_LOG 			app_trace_log
#else
#define LED_LOG(...)
#endif

#define LED_RED                              7                                           /**< LED to indicate advertising state. */
#define LED_BLUE                             6                                           /**< LED to indicate connected state. */
#define LED_FLASH_RATE_ON                 0xff
#define LED_FLASH_RATE_BREATH             0xfe
#define LED_FLASH_RATE_1HZ                0xfd
#define LED_FLASH_RATE_FAST               0xfc
#define LED_FLASH_RATE_OFF                0xfb
#define LED_FLASH_LED_RED                 0x01
#define LED_FLASH_LED_BLUE                0x02
#define LED_FLASH_LED_BOTH                0x03
#define LED_FLASH_LED_NONE                0x00
#define LED_FLASH_TIME_LONG               0xff
#define LED_FLASH_PERIOD_ONE_SHOT         0xff

typedef struct LED_FLASH_METHOD
{
	uint8_t flash_rate; //��˸Ƶ�� 0xff ����, 0xfe ������, 0xfd 1Hz, 0xfc flash fast,
	uint8_t flash_time;  //��˸ʱ�� ��λs 0xff flash forever
	uint8_t flash_period; //��˸����  ��λ����
	uint8_t flash_led;    //��˸��led��1:led_red 2:led_green 3:red and green 0:both red and green off
	uint32_t flash_end_time;
}Led_Flash_Method_t;

typedef enum LED_WORK_STATUS
{
    LED_WORK_POWER_USB_DISCONNECTED,
    LED_WORK_POWER_CHARGE_COMPLETE,
    LED_WORK_POWER_CHARGING,
    LED_WORK_POWER_OFF,
    LED_WORK_POWER_LOW,
    LED_WORK_DATA_FULL,
    LED_WORK_BLE_DATA_SEND_ERROR,
    LED_WORK_BLE_DATA_SENDING,
    LED_WORK_BLE_CONNECTED,
    LED_WORK_BLE_ADV,
    LED_WORK_BEGIN,
    LED_WORK_END,
	LED_WORK_IDLE,
	LED_WORK_MAX,
}Led_Work_Status_t;

void leds_init(void);
void leds_process_init(Led_Work_Status_t status);
void leds_process(void);
void leds_process_cancel(void);
void leds_process_flash_breath(void);
void leds_process_flash_fast(void);
Led_Work_Status_t leds_get_cur_status(void);


#endif

