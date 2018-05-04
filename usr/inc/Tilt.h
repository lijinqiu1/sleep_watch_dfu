#ifndef __TILT_H__
#define __TILT_H__

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

float calculateTilt(void);
void save_Tilt(float Tilt);

#endif
