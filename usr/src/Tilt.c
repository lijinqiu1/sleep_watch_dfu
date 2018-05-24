#include "tilt.h"
#include "lis3dh_driver.h"
#include "calender.h"
#include "queue.h"
#include "app_trace.h"
#include <math.h>
static float g_cur_Tilt;                                                        //��ǰ��Ǳ仯ֵ

//*****************��Ǽ���**************************
#define PI 3.1415926
#if 0
//������ˮƽ��нǣ������Χ0~180
static float calculateTilt_A(float ax, float ay, float az)
{
	float g = 9.80665;
	float temp;
	float Tiltangle = 0;
	temp = ((sqrt(2)/2)*g/10);
	if (az > temp)
	{
		Tiltangle = (1-ay*ay) - (1-ax*ax);
		if (Tiltangle < 0) {
			Tiltangle = - Tiltangle;
		}

		Tiltangle = acos(sqrt(Tiltangle));
		Tiltangle = Tiltangle/PI*180;
		Tiltangle = 90 - Tiltangle;

	}
	else
	{
		Tiltangle = asin(az);
		Tiltangle = Tiltangle/PI*180;
		Tiltangle = 90-Tiltangle;
	}
	return Tiltangle;
}

//���ؽǶȲ�
//flag = 1���»�ȡ�ǶȻ�׼ֵ��flag = 0��ʼ����
//��Y����Ϊת����
static float calculateTilt_run_A(float ax, float ay, float az)
{
	//��ʼ��׼�Ƕ�ֵ
	static float First_Tiltangle = 0;
	//��һʱ�̽Ƕ�ֵ
	float Tiltangle = 0;
	//�����ʼλ�� 1:>=0, 0:<0
	static uint8_t flag_x;
	//ת������,˳ʱ��ת��1����ʱ��ת��0

	if (tilt_init_flag == 1)
	{
		tilt_init_flag = 0;
		First_Tiltangle = calculateTilt_A(ax,ay,az);
		if (ax < 0)
		{
			First_Tiltangle = 360 - First_Tiltangle;
		}
		flag_x = (ax >= 0);
	}
	Tiltangle = calculateTilt_A(ax,ay,az);
	if (ax < 0)
	{
		Tiltangle = 360 - Tiltangle;
	}
	if (First_Tiltangle < Tiltangle)
	{
		if (flag_x ^ (ax >= 0))
		{
			return (360 -(Tiltangle - First_Tiltangle));
		}
		else
			return (Tiltangle - First_Tiltangle);
	}
	else
	{
		if (flag_x ^ (ax >= 0))
		{
			return (360 -(First_Tiltangle - Tiltangle));
		}
		else
			return (First_Tiltangle - Tiltangle);
	}

}
#endif

static float calculateTilt_B(float ax, float ay, float az)
{
	/*
	*���ڼ���ǰ��ת���Ƕȳ�ֵ���⣬��¼ǰһʱ��x,y,���ٶ������������为+90��
	*/
	float temp;
	float g = 9.80665;
	float Tiltangle = 0;
	temp = ((sqrt(2)/2)*g/10);
	if (fabs(ay) < temp)
	{
		Tiltangle = asin(fabs(ay));
		Tiltangle = Tiltangle/PI*180;
	}
	else
	{
		Tiltangle = acos(fabs(ay));
		Tiltangle = Tiltangle/PI*180;
		Tiltangle = 90-Tiltangle;
	}
	if((az < 0)&&(ay < 0))
	{
		Tiltangle = 90 + Tiltangle;
	}
	else if ((az > 0)&&(ay < 0))
	{
		Tiltangle = 270 - Tiltangle;
	}
	else if ((az > 0) && (ay > 0))
	{
		Tiltangle = 270 + Tiltangle;
	}
    else if ((az < 0) && (ay > 0))
    {
        Tiltangle = 90 - Tiltangle;
    }

	return Tiltangle;
}

#if 0
//���ؽǶȲ�
//flag = 1���»�ȡ�ǶȻ�׼ֵ��flag = 0��ʼ����
//��Y����Ϊת����
static float calculateTilt_run_B(float ax, float ay, float az)
{
	//��ʼ��׼�Ƕ�ֵ
	static float First_Tiltangle = 0;
	float Tiltangle = 0;
    float Tiltangle_return = 0;
	//�����ʼλ�� 1:>=0, 0:<0

	if (tilt_init_flag == 1)
	{
		tilt_init_flag = 0;
		First_Tiltangle = calculateTilt_B(ax,ay,az);
		if (ax < 0)
		{
			First_Tiltangle = 360 - First_Tiltangle;
		}
	}
	Tiltangle = calculateTilt_B(ax,ay,az);
	TITL_LOG("First_Tiltangle %f,Tiltangle %f\n",First_Tiltangle,Tiltangle);
    Tiltangle_return = First_Tiltangle - Tiltangle;
	if (Tiltangle_return < -180.0)
	{
		return (360 + Tiltangle_return);
	}
    else if (Tiltangle_return > 180)
    {
        return (360 - Tiltangle_return);
    }
    else if (Tiltangle_return < 0)
    {
        return -Tiltangle_return;
    }
    else
        return Tiltangle_return;

}
#endif


float calculateTilt(void)
{
    UTCTimeStruct time;
    AxesRaw_t Axes_Raw_Data = {0};
    float ax,ay,az;
    uint8_t response;
	float tilt = 0.0;
	response = LIS3DH_GetAccAxesRaw(&Axes_Raw_Data);
	if (response == 1) {
		ax = Axes_Raw_Data.AXIS_X/16384.0;
		ay = Axes_Raw_Data.AXIS_Y/16384.0;
		az = Axes_Raw_Data.AXIS_Z/16384.0;
		g_cur_Tilt = calculateTilt_B(ax,ay,az);
				ConvertUTCTime(&time,TimeSeconds);
		TITL_LOG("X=%6f Y=%6f Z=%6f \r\n",
			ax,ay,az);
		TITL_LOG("Tilt = %6f \r\n", g_cur_Tilt);
		TITL_LOG("y:%d m:%d d:%d h:%d m:%d s:%d\r\n",\
					  time.year,time.month,time.day,time.hour,time.minutes,time.seconds);
		tilt = g_cur_Tilt;
	}
	return tilt;
}

void save_Tilt(float Tilt)
{
    UTCTimeStruct time;
    queue_items_t item;
	ConvertUTCTime(&time,TimeSeconds);
	item.year = time.year - 2000;
	item.mon = time.month;
	item.day = time.day;
	item.hour = time.hour;
	item.min = time.minutes;
	item.second = time.seconds;
	item.angle = (uint16_t)(g_cur_Tilt*100);
	queue_push(&item);
	TITL_LOG("data push y:%d m:%d d:%d h:%d m:%d s:%d\r\n",\
				  time.year,time.month,time.day,time.hour,time.minutes,time.seconds);
}
