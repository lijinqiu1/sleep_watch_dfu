#ifndef __EVENT_H__
#define __EVENT_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef EVENT_DEBUG_LOG
#deifne EVENT_LOG 			app_trace_log
#else
#define EVENT_LOG(...)
#endif

#define LIS3DH_SMAPLE_RATE				1											/**< ������ٶȲ���Ƶ�� ��λ:�� >**/

#define ANGLE_SMAPLE_RATE				180									/**< �Ƕ����ݲ���Ƶ�� ��λ:�� 180>**/

/***********************************�¼�����***************************************/
#define EVENT_KEY_PRESSED               (uint32_t)(0x00000001 << 0)                  /**< �����¼� >**/
#define EVENT_MESSAGE_RECEIVED          (uint32_t)(0x00000001 << 1)					 /**< ͨ���¼� >**/
#define EVENT_DATA_SENDING				(uint32_t)(0x00000001 << 2)					 /**< ���ݷ����¼� >**/
#define EVENT_DATA_SENDED               (uint32_t)(0x00000001 << 3)					 /**< ���ݷ�������¼� >**/
#define EVENT_QUEUE_PUSH				(uint32_t)(0x00000001 << 4)                  /**< �Ƕȴ洢 >**/
#define EVENT_ALARM_HAPPEN              (uint32_t)(0x00000001 << 5)                  /**< �������� >**/
#define EVENT_ADV_START                 (uint32_t)(0x00000001 << 6)                  /**< ֹͣ�㲥 >**/
#define EVENT_ADV_STOP                  (uint32_t)(0x00000001 << 7)                  /**< ��ʼ�㲥 >**/
#define EVENT_BATTRY_VALUE              (uint32_t)(0x00000001 << 8)                  /**< ��ȡ��ص�ѹ >**/
#define EVENT_LIS3DH_VALUE              (uint32_t)(0x00000001 << 9)                  /**< ��ȡ3������ >**/
#define EVENT_TILT_PUSH                 (uint32_t)(0x00000001 <<10)                  /**< �Ƕ�ֵ�洢 >**/
#define EVENT_BLE_SHUT_CONNECT          (uint32_t)(0x00000001 <<11)                  /**< �Ͽ��������� >**/

#define EVENT_DATA_SYNC                 (uint32_t)(0x00000001 <<12)                  /**< ͬ���������� >**/
#define EVENT_POWER_LOW                 (uint32_t)(0x00000001 <<13)                  /**< ��ص�ѹ�� >**/
#define EVENT_POWER_CHARGING            (uint32_t)(0x00000001 <<14)                  /**< ��س�� >**/
#define EVENT_POWER_CHARGE_COMPLETE     (uint32_t)(0x00000001 <<15)                  /**< ��س����� >**/
#define EVENT_DATA_FULL                 (uint32_t)(0x00000001 <<16)                  /**< �洢������ >**/
#define EVENT_BLE_CONNECTED             (uint32_t)(0x00000001 <<17)                  /**< �������� >**/
#define EVENT_BEGIN_WORK                (uint32_t)(0x00000001 <<18)                  /**< ��ʼ���� >**/
#define EVENT_END_WORK                  (uint32_t)(0x00000001 <<19)                  /**< ֹͣ���� >**/
#define EVENT_BLE_DISCONNECTED          (uint32_t)(0x00000001 <<20)                  /**< �����Ͽ� >**/

#define EVENT_KEY_PRESS_SHOT            (uint32_t)(0x00000001 <<21)                  /**< �����̰� >**/
#define EVENT_KEY_PRESS_LONG            (uint32_t)(0x00000001 <<22)                  /**< �������� >**/
#define EVENT_KEY_RESET                 (uint32_t)(0x00000001 <<23)                  /**< �ָ��������� >**/

#define ALARM_SLEEP_POSE_ONE_BEGIN           0
#define ALARM_SLEEP_POSE_ONE_END            60
#define ALARM_SLEEP_POSE_TWO_BEGIN          60
#define ALARM_SLEEP_POSE_TWO_END            80
#define ALARM_SLEEP_POSE_THREE_BEGIN        80
#define ALARM_SLEEP_POSE_THREE_END         100
#define ALARM_SLEEP_POSE_FOUR_BEGIN        100
#define ALARM_SLEEP_POSE_FOUR_END          120
#define ALARM_SLEEP_POSE_FIVE_BEGIN        120
#define ALARM_SLEEP_POSE_FIVE_END          180
#define ALARM_SLEEP_POSE_SIX_BEGIN         180
#define ALARM_SLEEP_POSE_SIX_END           360

#define ALARM_SLEEP_POSE_ONE               0
#define ALARM_SLEEP_POSE_TWO               1
#define ALARM_SLEEP_POSE_THREE             2
#define ALARM_SLEEP_POSE_FOUR              3
#define ALARM_SLEEP_POSE_FIVE              4
#define ALARM_SLEEP_POSE_SIX               5

extern uint32_t g_event_status ;                                        //�¼��洢����
extern bool g_status_key_pressed;                                       //��¼�����¼�
extern bool g_status_adv;                                               //�㲥״̬
extern bool g_status_work;                                              //�豸����״̬
extern bool g_status_data_send;                                         //��������
extern bool g_status_ble_connect;                                       //��������״̬
extern bool g_status_alarm_status;                                      //����״̬
extern bool g_status_bond_info_received;                                //���յ�����Ϣ
extern bool g_status_tilt_init_flag;                                    //�Ƕ�ֵ��ʼ��
extern uint8_t rec_data_buffer[20];                                     //������յ�������

void sleep_manage(float tilt);
void period_cycle_process_handler(void * p_context);

#endif
