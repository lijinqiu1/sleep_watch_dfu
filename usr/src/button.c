#include "button.h"
#include "boards.h"
#include "event.h"
#include "queue.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "app_trace.h"
#include <string.h>

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
//gpiote
#define MAX_USERS						1

//gpiote user identifier
static app_gpiote_user_id_t gpiote_user_id;

//按键防抖
static app_timer_id_t                    m_key_tiemr_id;
#define SECURITY_REQUEST_DELAY           APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)  /**< Delay after connection until Security Request is sent, if necessary (ticks). */


void key_req_timeout_handler(void * p_context)
{
    static uint16_t key_count = 0;
    if (nrf_gpio_pin_read(BUTTON_1) == 0)
    {
        key_count ++;
		BUTTON_LOG("key_count %d\n",key_count);
		if (key_count >= 30)
        {//进入dfu模式
			BUTTON_LOG("NVIC_SystemReset %s\n",__FUNCTION__);
            NVIC_SystemReset();
        }
        else if (key_count >= 15)
        {//恢复出厂设置
			BUTTON_LOG("set EVENT_KEY_RESET\n");
			g_event_status |= EVENT_KEY_RESET;
        }
        else if (key_count >= 5)
        {
            g_event_status |= EVENT_KEY_PRESS_LONG;
        }
        app_timer_start(m_key_tiemr_id,APP_TIMER_TICKS(1000,APP_TIMER_PRESCALER),NULL);
    }
    else
    {
		if (key_count < 5)
			g_event_status |= EVENT_KEY_PRESS_SHOT;
        key_count = 0;
    }
}

//gpiote handler function
static void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
{
	if (event_pins_high_to_low & (uint32_t)(1<<BUTTON_1))
	{
        /*
		if (g_status_data_send == false) {
			//数据传输功能时按键无效
			//button 1 pressed
			g_status_key_pressed = true;
			
		}
		*/
        app_timer_start(m_key_tiemr_id,APP_TIMER_TICKS(100,APP_TIMER_PRESCALER),NULL);
	}
}

void button_init(void)
{
    uint32_t err_code = 0;
	uint32_t low_to_high_bitmask = (uint32_t)(1 << BUTTON_1);
	uint32_t high_to_low_bitmask = (uint32_t)(1 << BUTTON_1);

	nrf_gpio_cfg_sense_input(BUTTON_1,
						 BUTTON_PULL,
						 NRF_GPIO_PIN_SENSE_LOW);
	//初始化gpio引脚中断
	APP_GPIOTE_INIT(MAX_USERS);
	err_code = app_gpiote_user_register(&gpiote_user_id,
										low_to_high_bitmask,
										high_to_low_bitmask,
										gpiote_event_handler);
	APP_ERROR_CHECK(err_code);

	err_code = app_gpiote_user_enable(gpiote_user_id);
	APP_ERROR_CHECK(err_code);
	
	//按键防抖定时器
	err_code = app_timer_create(&m_key_tiemr_id,
		                        APP_TIMER_MODE_SINGLE_SHOT,
		                        key_req_timeout_handler);
	APP_ERROR_CHECK(err_code);

}
