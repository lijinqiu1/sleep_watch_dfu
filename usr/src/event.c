#include "event.h"
#include "calender.h"
#include "queue.h"
#include "battery.h"
#include "pwm.h"
#include "adc.h"
#include "led.h"
#include "app_trace.h"
#include <math.h>

uint32_t g_event_status = 0;                                             //�¼��洢����
bool g_status_key_pressed = false;                                       //��¼�����¼�
bool g_status_adv = false;                                               //�㲥״̬
bool g_status_work = false;                                              //�豸����״̬
bool g_status_data_send = false;                                         //��������
bool g_status_ble_connect = false;                                       //��������״̬
bool g_status_alarm_status = false;                                      //����״̬
bool g_status_bond_info_received = false;                                //���յ�����Ϣ
bool g_status_tilt_init_flag = false;                                    //�Ƕ�ֵ��ʼ��
uint8_t rec_data_buffer[20];                                             //������յ�������

/***********************************˯����̬����***********************************/
void sleep_manage(float tilt)
{
	static uint8_t last_sleep_post = 0;
	static uint8_t cur_sleep_post = 0;
	static uint32_t cur_timeseconds = 0;
	static float cur_Tilt = 0; //��¼����ʱ�̵ĽǶ�ֵ
	static uint8_t alarm_cancel_count = 0;
	if(g_status_tilt_init_flag == true)
	{
		if ((ALARM_SLEEP_POSE_ONE_BEGIN <= tilt)&&
			(tilt <= ALARM_SLEEP_POSE_ONE_END))
		{
			last_sleep_post = cur_sleep_post = ALARM_SLEEP_POSE_ONE;
		}
		else if (tilt <= ALARM_SLEEP_POSE_TWO_END)
		{
			last_sleep_post = cur_sleep_post = ALARM_SLEEP_POSE_TWO;
		}
		else if (tilt <= ALARM_SLEEP_POSE_THREE_END)
		{
			last_sleep_post = cur_sleep_post = ALARM_SLEEP_POSE_THREE;
		}
		else if (tilt <= ALARM_SLEEP_POSE_FOUR_END)
		{
			last_sleep_post = cur_sleep_post = ALARM_SLEEP_POSE_FOUR;
		}
		else if (tilt <= ALARM_SLEEP_POSE_FIVE_END)
		{
			last_sleep_post = cur_sleep_post = ALARM_SLEEP_POSE_FIVE;
		}
		else if (tilt <= ALARM_SLEEP_POSE_SIX_END)
		{
			last_sleep_post = cur_sleep_post = ALARM_SLEEP_POSE_SIX;
		}
		cur_timeseconds = TimeSeconds;
		g_status_tilt_init_flag = false;
		alarm_cancel_count = 0;
	}
	else
	{
		if ((ALARM_SLEEP_POSE_ONE_BEGIN <= tilt)&&
			(tilt <= ALARM_SLEEP_POSE_ONE_END))
		{
			cur_sleep_post = ALARM_SLEEP_POSE_ONE;
		}
		else if (tilt <= ALARM_SLEEP_POSE_TWO_END)
		{
			cur_sleep_post = ALARM_SLEEP_POSE_TWO;
		}
		else if (tilt <= ALARM_SLEEP_POSE_THREE_END)
		{
			cur_sleep_post = ALARM_SLEEP_POSE_THREE;
		}
		else if (tilt <= ALARM_SLEEP_POSE_FOUR_END)
		{
			cur_sleep_post = ALARM_SLEEP_POSE_FOUR;
		}
		else if (tilt <= ALARM_SLEEP_POSE_FIVE_END)
		{
			cur_sleep_post = ALARM_SLEEP_POSE_FIVE;
		}
		else if (tilt <= ALARM_SLEEP_POSE_SIX_END)
		{
			cur_sleep_post = ALARM_SLEEP_POSE_SIX;
		}
		if ((last_sleep_post == cur_sleep_post) &&
			((TimeSeconds - cur_timeseconds ) >= (system_params.time[cur_sleep_post] * 60)))
		{
			if (g_status_alarm_status == false)
			{
				cur_Tilt = tilt;
				g_status_alarm_status = true;
				alarm_init();
			}
			else if ((g_status_alarm_status == true)&&
				(fabs(cur_Tilt - tilt) > 15.00) &&
			    (alarm_cancel_count ++ > 3))
			{//�Ƕȱ仯����15�ȣ��������
				app_trace_log("alarm_case tilt %f, last %f\n",cur_Tilt,tilt);
				cur_Tilt = tilt;
				alarm_cancel_count=0;
				g_status_alarm_status = false;
				cur_timeseconds = TimeSeconds;
				alarm_case();
			}
			else
			{
				alarm_cancel_count = 0;
			}
		}
		else if(last_sleep_post != cur_sleep_post)
		{//˯�˱任����ȡ��
		    if (g_status_alarm_status)
		    {
			    g_status_alarm_status = false;
				alarm_cancel_count = 0;
				app_trace_log("alarm_case sleep_post\n");
				alarm_case();
		    }
			last_sleep_post = cur_sleep_post;
			cur_timeseconds = TimeSeconds;
		}
		else
		{
			app_trace_log("sleep_post %d time %d\n",cur_sleep_post,TimeSeconds - cur_timeseconds);
		}
	}
}

void period_cycle_process_handler(void * p_context)
{
	static uint16_t angle_timer = 0;	//�ǶȲ���Ƶ��
	static uint16_t data_send_completed = 0; //���ڴ������ݽ�����ر���������
	static uint16_t battery_timer = 0; //��ص���
	static uint8_t one_shot_timer = 0;
	//ģ������
	TimeSeconds ++;

	if ((g_status_work == true))
	{//ʹ��������ٶȲ���
		if (angle_timer++ >= (ANGLE_SMAPLE_RATE-1))
		{
			angle_timer = 0;
			//�洢����
			g_event_status |= EVENT_TILT_PUSH;
		}
		g_event_status |= EVENT_LIS3DH_VALUE;
	}

	if (g_status_ble_connect == true)
	{
		if ((g_event_status & EVENT_DATA_SENDED) && (data_send_completed++ >= 30))
		{
			//���ݴ�����ɺ�30s�Ͽ���������
			data_send_completed = 0;
			g_event_status |= EVENT_BLE_SHUT_CONNECT;
			g_event_status&= ~EVENT_DATA_SENDED;
		}
	}

	//��ص���
	if (battery_timer ++ > 15)
	{
		g_event_status |= EVENT_BATTRY_VALUE;
		battery_timer = 0;
	}
    battery_manager();
	//���洢״̬
	if (queue_is_full())
	{
		g_event_status |= EVENT_DATA_FULL;
	}

	if ((g_status_ble_connect == true) &&
		(system_params.device_bonded == true) &&
		(g_status_bond_info_received == false) &&
		(one_shot_timer ++ > 30))
	{
		g_event_status |= EVENT_BLE_SHUT_CONNECT;
	}
	else if (g_status_ble_connect == false)
	{
		one_shot_timer = 0;
		g_status_bond_info_received = false;
	}

	if (g_status_alarm_status)
	{
		alarm_process();
	}
    //led ����
    leds_process();
}
