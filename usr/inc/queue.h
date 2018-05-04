#ifndef __QUEUE_H__
#define __QUEUE_H__


//���нṹ��
typedef struct {
	uint16_t entries;
	uint16_t tx_point;
	uint16_t rx_point;
	uint16_t Reserved;
}queue_t;

typedef struct {
	uint8_t time[6]; //����ʱ��
	uint8_t mac_add[11]; //�豸��MAC��ַ
	uint8_t device_bonded; //�豸�󶨱�־
	uint8_t moto_strong;
	uint8_t Reserved[5];
}system_params_t;

typedef struct {
	uint8_t year;
	uint8_t mon;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t second;
	uint16_t angle;
}queue_items_t;

/**************************��������******************************/
#define DEFAULT_ALARM_ANGLE				75  /**< Ĭ�ϸ���Ƕ� ��λ: ��>**/
#define DEFAULT_ALARM_TIME              120 /**< Ĭ�ϸ���ʱ�� ��λ: s >**/
/**************************�洢����******************************/
//flash block ��С
#define FLASH_BLOCK               1024
//Flash block ����
#define FLASH_BLOCK_NUM           6
//flash�ռ��С
#define FLASH_SIZE                (FLASH_BLOCK * FLASH_BLOCK_NUM)
//������Ϣռ�ÿռ�(���ڴ洢��ǰ����״̬)
#define QUEUE_STATUS_SIZE         sizeof(queue_t)
//������Ϣ��ʼ��ַ
#define QUEUE_STATUS_ADDRESS      0
//ϵͳ��Ϣռ�ÿռ�����(���ڴ洢ϵͳ��Ϣ����������������Ϣ��)
#define SYSTEM_PARAMS_SIZE         sizeof(system_params_t)
//ϵͳ��Ϣ��ʼ��ַ
#define SYSTEM_PARAMS_ADDRESS     (QUEUE_STATUS_ADDRESS + QUEUE_STATUS_SIZE)
//������ʼ��ַ
#define QUEUE_BEGING_ADDRESS      (QUEUE_STATUS_SIZE+SYSTEM_PARAMS_SIZE)
//����Ԫ�ش�С
#define QUEUE_ITEM_SIZE           sizeof(queue_items_t)
//ÿһ��block�а�����Ԫ�ظ���
#define QUEUE_BLOCK_ITEMS_COUNT (FLASH_BLOCK / QUEUE_ITEM_SIZE)
//����������
#define QUEUE_MAX_ENTRIES  	      ((FLASH_SIZE - QUEUE_BEGING_ADDRESS)/QUEUE_ITEM_SIZE)
//��ǰ�������
#define QUEUE_ENTRIES_NUM         QUEUE_MAX_ENTRIES

typedef enum {
	QUEUE_STATUS_UPDATING,
	QUEUE_STATUS_UPDATE_READY,
	QUEUE_STATUS_UPDATE_FAILED,
	QUEUE_STATUS_LOADING,
	QUEUE_STATUS_LOAD_SUCCESS,
	QUEUE_STATUS_LOAD_ERROR,
}queue_status_t;

extern queue_t queue_entries;
extern system_params_t system_params;

void queue_init(void);
uint8_t queue_pop(queue_items_t *item);
void queue_push(queue_items_t *item);
uint16_t queue_get_entries(void);
void system_params_save(system_params_t * params);
void queue_sync(void);
uint8_t queue_is_full(void);
void queue_message_init(void);
uint8_t queue_message_pop(uint8_t *message);
void queue_message_push(uint8_t * message);

#endif
