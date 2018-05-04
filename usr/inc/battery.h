#ifndef __BATTERY_H__
#define __BATTERY_H__

#define BQ24040_PG_PIN      11
#define BQ24040_CHG_PIN     12

#define BATTER_VALUE_OFF    250
#define BATTER_VALUE_LOW    255
#define BATTER_VALUE_20     260
#define BATTER_VALUE_40     265
#define BATTER_VALUE_60     270
#define BATTER_VALUE_80     280
#define BATTER_VALUE_100    295

typedef enum BATTERY_CHARGE_STATUS
{
    BATTERY_NOT_CHARGE,
    BATTERY_CHARGING,
    BATTERY_CHARGE_COMPLETE,
    BATTERY_VALUE_LOW,
}Battery_Charge_Status_e;

extern uint16_t battery_value;

void battery_init(void);
void battery_manager(void);
void battery_get_status(void);
uint16_t battery_get_value(void);
Battery_Charge_Status_e battery_get_charege_status(void);


#endif
