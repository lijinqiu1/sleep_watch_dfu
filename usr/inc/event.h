#ifndef __EVENT_H__
#define __EVENT_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef EVENT_DEBUG_LOG
#deifne EVENT_LOG 			app_trace_log
#else
#define EVENT_LOG(...)
#endif

#define LIS3DH_SMAPLE_RATE				1											/**< 三轴加速度采样频率 单位:秒 >**/

#define ANGLE_SMAPLE_RATE				180									/**< 角度数据采样频率 单位:秒 180>**/

/***********************************事件定义***************************************/
#define EVENT_KEY_PRESSED               (uint32_t)(0x00000001 << 0)                  /**< 按键事件 >**/
#define EVENT_MESSAGE_RECEIVED          (uint32_t)(0x00000001 << 1)					 /**< 通信事件 >**/
#define EVENT_DATA_SENDING				(uint32_t)(0x00000001 << 2)					 /**< 数据发送事件 >**/
#define EVENT_DATA_SENDED               (uint32_t)(0x00000001 << 3)					 /**< 数据发送完成事件 >**/
#define EVENT_QUEUE_PUSH				(uint32_t)(0x00000001 << 4)                  /**< 角度存储 >**/
#define EVENT_ALARM_HAPPEN              (uint32_t)(0x00000001 << 5)                  /**< 报警产生 >**/
#define EVENT_ADV_START                 (uint32_t)(0x00000001 << 6)                  /**< 停止广播 >**/
#define EVENT_ADV_STOP                  (uint32_t)(0x00000001 << 7)                  /**< 开始广播 >**/
#define EVENT_BATTRY_VALUE              (uint32_t)(0x00000001 << 8)                  /**< 获取电池电压 >**/
#define EVENT_LIS3DH_VALUE              (uint32_t)(0x00000001 << 9)                  /**< 获取3轴数据 >**/
#define EVENT_TILT_PUSH                 (uint32_t)(0x00000001 <<10)                  /**< 角度值存储 >**/
#define EVENT_BLE_SHUT_CONNECT          (uint32_t)(0x00000001 <<11)                  /**< 断开蓝牙连接 >**/

#define EVENT_DATA_SYNC                 (uint32_t)(0x00000001 <<12)                  /**< 同步队列数据 >**/
#define EVENT_POWER_LOW                 (uint32_t)(0x00000001 <<13)                  /**< 电池电压低 >**/
#define EVENT_POWER_CHARGING            (uint32_t)(0x00000001 <<14)                  /**< 电池充电 >**/
#define EVENT_POWER_CHARGE_COMPLETE     (uint32_t)(0x00000001 <<15)                  /**< 电池充电完成 >**/
#define EVENT_DATA_FULL                 (uint32_t)(0x00000001 <<16)                  /**< 存储队列满 >**/
#define EVENT_BLE_CONNECTED             (uint32_t)(0x00000001 <<17)                  /**< 蓝牙连接 >**/
#define EVENT_BEGIN_WORK                (uint32_t)(0x00000001 <<18)                  /**< 开始工作 >**/
#define EVENT_END_WORK                  (uint32_t)(0x00000001 <<19)                  /**< 停止工作 >**/
#define EVENT_BLE_DISCONNECTED          (uint32_t)(0x00000001 <<20)                  /**< 蓝牙断开 >**/

#define EVENT_KEY_PRESS_SHOT            (uint32_t)(0x00000001 <<21)                  /**< 按键短按 >**/
#define EVENT_KEY_PRESS_LONG            (uint32_t)(0x00000001 <<22)                  /**< 按键长按 >**/
#define EVENT_KEY_RESET                 (uint32_t)(0x00000001 <<23)                  /**< 恢复出厂设置 >**/

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

extern uint32_t g_event_status ;                                        //事件存储变量
extern bool g_status_key_pressed;                                       //记录按键事件
extern bool g_status_adv;                                               //广播状态
extern bool g_status_work;                                              //设备工作状态
extern bool g_status_data_send;                                         //发送数据
extern bool g_status_ble_connect;                                       //蓝牙连接状态
extern bool g_status_alarm_status;                                      //报警状态
extern bool g_status_bond_info_received;                                //接收到绑定信息
extern bool g_status_tilt_init_flag;                                    //角度值初始化
extern uint8_t rec_data_buffer[20];                                     //缓存接收到的数据

void sleep_manage(float tilt);
void period_cycle_process_handler(void * p_context);

#endif
