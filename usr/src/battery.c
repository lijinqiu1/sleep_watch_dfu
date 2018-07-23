#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "battery.h"
#include "adc.h"
#include "led.h"
#include "app_trace.h"
#include "event.h"

static Battery_Charge_Status_e battery_charge_cur_status = BATTERY_NOT_CHARGE;
static Battery_Charge_Status_e battery_charge_last_status = BATTERY_NOT_CHARGE;
uint16_t battery_value;

void battery_init(void)
{
    nrf_gpio_cfg_input(BQ24040_CHG_PIN, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(BQ24040_PG_PIN, NRF_GPIO_PIN_PULLUP);
    adc_init();
}

void battery_manager(void)
{
    if (!nrf_gpio_pin_read(BQ24040_PG_PIN))
    {
	   BATTERY_LOG("battery_value %d\n",battery_value);
       if((!nrf_gpio_pin_read(BQ24040_CHG_PIN)) && (battery_value < BATTER_VALUE_100))
       {//充电中
            battery_charge_cur_status = BATTERY_CHARGING;
            battery_charge_last_status = battery_charge_cur_status;
			leds_process_init(LED_WORK_POWER_CHARGING);
       }
       else
       {//充电完成
            battery_charge_cur_status = BATTERY_CHARGE_COMPLETE;
            battery_charge_last_status = battery_charge_cur_status;
			leds_process_init(LED_WORK_POWER_CHARGE_COMPLETE);
       }
    }
    else
    {
		if (battery_value < BATTER_VALUE_OFF)
		{
			leds_process_init(LED_WORK_POWER_OFF);
			battery_charge_cur_status = BATTERY_VALUE_LOW;
		}
		else 
	    if (battery_value < BATTER_VALUE_LOW)
		{
			battery_charge_cur_status = BATTERY_VALUE_LOW;
            if (battery_charge_cur_status != battery_charge_last_status)
            {
				if (battery_charge_last_status == BATTERY_CHARGING)
				{
					leds_process_cancel();
				}
                battery_charge_last_status = battery_charge_cur_status;
			    leds_process_init(LED_WORK_POWER_LOW);
				g_event_status |= EVENT_END_WORK;
            }
		}
		else
		{
        	battery_charge_cur_status = BATTERY_NOT_CHARGE;
            if (battery_charge_cur_status != battery_charge_last_status)
            {
                battery_charge_last_status = battery_charge_cur_status;
			    leds_process_cancel();
            }
		}
    }
}

Battery_Charge_Status_e battery_get_charege_status(void)
{
    return battery_charge_cur_status;
}

uint16_t battery_get_value(void)
{
    battery_value = adc_start();
	return battery_value;
}
