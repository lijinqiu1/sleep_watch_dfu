#include <stdint.h>
#include "nordic_common.h"
#include "nrf.h"
#include "softdevice_handler.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "pwm.h"
#include "queue.h"
#include "app_trace.h"
#include "led.h"
#include "app_pwm.h"

#define APP_TIMER_PRESCALER 0

static uint8_t pwm_led_status = 0;
static const uint8_t moto_strong[4] = {MOTO_STRONG_ONE,MOTO_STRONG_TWO,
									MOTO_STRONG_THREE,MOTO_STRONG_FULL};
static uint8_t moto_strong_index = 0;
static uint8_t moto_time = 0; //记录震动时长
static uint8_t moto_count = 0; //记录震动次数

APP_PWM_INSTANCE(PWM1,1);
APP_PWM_INSTANCE(PWM2,2);

app_timer_id_t                    m_pwm_timer_id;
	
void pwm_cycle_process_handler(void * p_context)
{
	uint32_t value;
	uint32_t timeout = 0;
	static uint8_t i = 0;
	i ++;
	if (i > 40)
	{
		i = 0;
	}
    value = (i < 20) ? (i * 5) : (100 - (i - 20) * 5);
        
	/* Se the duty cycle - keep trying until PWM is ready. */
	while (app_pwm_channel_duty_set(&PWM1, 0, value) == NRF_ERROR_BUSY)
	{
		if (timeout++ > 200)
		{
			break;
		}
	}
	if (timeout++)
	{
		PWM_LOG("set value timeout!\n");
	}
}	
void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
}

/** @brief Function for handling timer 2 peripheral interrupts.
 */

void pwm_led_start(uint32_t led)
{
	PWM_LOG("entery %s\n",__FUNCTION__);
    if (pwm_led_status != 0)
    {
        return ;
    }
	pwm_led_status = 1;
	ret_code_t err_code;
//	nrf_gpio_pin_dir_set(LED_BLUE, NRF_GPIO_PIN_DIR_OUTPUT); nrf_gpio_pin_clear(LED_BLUE);
	
	/* 1-channel PWM, 200 Hz, output on DK LED pins. */
	app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(5000L, LED_BLUE);
	/* Switch the polarity of the second channel. */
	pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
	/* Initialize and enable PWM. */
	err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
	APP_ERROR_CHECK(err_code);
	app_pwm_enable(&PWM1);
	
	err_code = app_timer_start(m_pwm_timer_id,APP_TIMER_TICKS(25,APP_TIMER_PRESCALER),NULL);
    APP_ERROR_CHECK(err_code);
	PWM_LOG("leave %s\n",__FUNCTION__);
}

void pwm_led_stop(void)
{
	ret_code_t err_code;
	PWM_LOG("entery %s\n",__FUNCTION__);
    if (pwm_led_status == 0)
    {
        return;
    }
	
	pwm_led_status = 0;
	err_code = app_timer_stop(m_pwm_timer_id);
    APP_ERROR_CHECK(err_code);
	
	app_pwm_disable(&PWM1);
	nrf_drv_gpiote_out_task_disable(LED_BLUE); 
	err_code = app_pwm_uninit(&PWM1);
	APP_ERROR_CHECK(err_code);
	PWM_LOG("unint %s\n",__FUNCTION__);
	nrf_gpio_cfg_output(LED_BLUE);
	nrf_gpio_pin_set(LED_BLUE);
}

void pwm_moto_test(void)
{
	pwm_moto_init();
	while (app_pwm_channel_duty_set(&PWM2, 0, 40) == NRF_ERROR_BUSY);
}

void pwm_moto_init(void)
{
	ret_code_t err_code;
	PWM_LOG("entery %s\n",__FUNCTION__);
	/* 1-channel PWM, 200 Hz, output on DK LED pins. */
	app_pwm_config_t pwm2_cfg = APP_PWM_DEFAULT_CONFIG_1CH(10000L, PWM_MOTO_PIN);
	/* Switch the polarity of the second channel. */
	pwm2_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_LOW;
	/* Initialize and enable PWM. */
	err_code = app_pwm_init(&PWM2,&pwm2_cfg,pwm_ready_callback);
	APP_ERROR_CHECK(err_code);
	app_pwm_enable(&PWM2);
	PWM_LOG("leave %s\n",__FUNCTION__);
}

static void pwm_moto_deinit(void)
{
	ret_code_t err_code;
		
	app_pwm_disable(&PWM2);
	nrf_drv_gpiote_out_task_disable(PWM_MOTO_PIN);
	err_code = app_pwm_uninit(&PWM2);
	APP_ERROR_CHECK(err_code);
	
	nrf_gpio_cfg_output(PWM_MOTO_PIN);
	nrf_gpio_pin_clear(PWM_MOTO_PIN);
}


void alarm_init(void)
{
	moto_time = 0;
	moto_count = 0;
	moto_strong_index = 0;
    PWM_LOG("%s!\n",__FUNCTION__);
}

void alarm_process(void)
{
	if (moto_count < 10)
	{
		switch(moto_time)
		{
		case 0:
    		pwm_moto_init();
			if (system_params.moto_strong == MOTO_LEVEL_AUTO)
			{
				while (app_pwm_channel_duty_set(&PWM2, 0, moto_strong[moto_strong_index]) == NRF_ERROR_BUSY);
//				while (app_pwm_channel_duty_set(&PWM2, 0, MOTO_STRONG_FULL) == NRF_ERROR_BUSY);
                PWM_LOG("%s index %d time %d count %d!\n",__FUNCTION__,moto_strong_index,moto_time,moto_count);
				if (moto_strong_index < 3)
				{
					moto_strong_index++;
				}
			}
			moto_time++;
			break;
		case 3:
			pwm_moto_deinit();
            PWM_LOG("%s index %d time %d count %d!\n",__FUNCTION__,moto_strong_index,moto_time,moto_count);
			moto_time++;
			break;
		case 8:
            PWM_LOG("%s index %d time %d count %d!\n",__FUNCTION__,moto_strong_index,moto_time,moto_count);
			moto_time = 0;
			moto_count++;
			break;
		default:
			moto_time++;
			break;
		}
	}
}

void alarm_case(void)
{
	pwm_moto_deinit();
    PWM_LOG("%s!\n",__FUNCTION__);
}
