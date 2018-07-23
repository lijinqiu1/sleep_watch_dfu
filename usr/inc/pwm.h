#ifndef __PWM_H__
#define __PWM_H__

#include "app_timer.h"
#include "app_trace.h"
#define PWM_DEBUG_LOG

#ifdef PWM_DEBUG_LOG
#define PWM_LOG 			app_trace_log
#else
#define PWM_LOG(...)
#endif

//pwm
#define MAX_SAMPLE_LEVELS (256UL)  /**< Maximum number of sample levels. */
#define TIMER_PRESCALERS   6U       /**< Prescaler setting for timer. */

#define MOTO_STRONG_ONE      30
#define MOTO_STRONG_TWO      40
#define MOTO_STRONG_THREE    50
#define MOTO_STRONG_FULL     60

#define PWM_LED_PIN          12
#define PWM_MOTO_PIN         29

#define PWM_STATUS_READY     0x00
#define PWM_LED_MODE         0x01
#define PWM_MOTO_MODE        0x02

#define DEFAULT_MOTO_STRONG MOTO_LEVEL_AUTO

typedef enum pwm_moto_level
{
	MOTO_LEVEL_0,
	MOTO_LEVEL_25,
	MOTO_LEVEL_50,
	MOTO_LEVEL_75,
	MOTO_LEVEL_100,
	MOTO_LEVEL_AUTO,
}PwmMotoLevel_t;

extern app_timer_id_t                    m_pwm_timer_id;

void alarm_init(void);
void alarm_process(void);
void alarm_case(void);

void pwm_moto_init(void);
void pwm_led_start(uint32_t led);
void pwm_led_stop(void);
void pwm_cycle_process_handler(void * p_context);
void pwm_moto_test(void);
#endif
