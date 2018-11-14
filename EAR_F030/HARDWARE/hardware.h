/**
********************************************************************************
* ��ࣺ
* ģ�����ƣ�hardware.c
* ģ�鹦�ܣ�
* �������ڣ�
* �� �� �ߣ�
* ��    ����
********************************************************************************
**/
#ifndef __HARDWARE_
#define __HARDWARE_

/***********************************
* ͷ�ļ�
***********************************/
#include "datatype.h"
#include "stm32f0xx.h"
/**********************************
*�궨��
***********************************/
//�������
#define EXP_DETECT_PIN    GPIO_Pin_9
#define EXP_DETECT_PORT   GPIOA
//LED ��ɫ    
#define GREEN_LED_PIN    GPIO_Pin_9
#define GREEN_LED_PORT   GPIOB
//LED ��ɫ    
#define RED_LED_PIN    GPIO_Pin_8
#define RED_LED_PORT   GPIOB
////LED ��ɫ    
//#define BLUE_LED_PIN    GPIO_Pin_1
//#define BLUE_LED_PORT   GPIOB
//����
#define KEY_DETECT_PIN    GPIO_Pin_0
#define KEY_DETECT_PORT   GPIOA
//PWR_SAVE
#define KEY_PWR_SAVE_PIN  GPIO_Pin_5
#define KEY_PWR_SAVE_PORT   GPIOA


#define ADC1_DR_Address                0x40012440

/***********************************
* ȫ�ֱ���
***********************************/

/***********************************
* ��������
***********************************/
typedef enum{
	LED_CLOSE,
	LED_GREEN,
	LED_RED,
	LED_BLUE
}LED_COLOR;

//ʱ��ṹ��
typedef struct 
{
		//������������
	uint16_t w_year;
	uint8_t  w_month;
	uint8_t  w_date;
	
	uint8_t hour;
	uint8_t min;
	uint8_t sec;			 
}_calendar_obj;
/***********************************
* �ⲿ����
***********************************/
void init_hardware(void);
//void init_hardware_byWakeUpOrNot(BOOL bWakeUp);
void init_rtc(void);
void init_tim(void);
void convert_rtc(_calendar_obj* calendar, uint32_t rtc);
void set_rtc(uint32_t rtc);
void set_led(LED_COLOR color);
uint32_t get_rtc(void);
BOOL get_exp_status(void);
BOOL get_key_status(void);
BOOL save_one_page_to_flash(uint32_t Address, uint8_t* buf, uint16_t len);
void read_one_page_from_flash(uint32_t Address, uint8_t* buf, uint16_t len);
BOOL save_half_word_to_flash(uint32_t Address, uint16_t data);
void read_half_word_from_flash(uint32_t Address, uint16_t* pdata);
BOOL save_half_word_buf_to_eeprom(uint32_t Address, uint16_t* buf, uint16_t len);
void read_half_word_buf_from_eeprom(uint32_t Address, uint16_t* buf, uint16_t len);
uint8_t get_bat_vol_per(void);


void Init_LED(void);

void Init_PWRSAVE(void);

//���ð���key_wakeup
void Key_WakeUp_Init(void);

////��ʼ��flash�еĲ���
//void Init_parameter_in_Flash(void);

//ADC
void ADC1_Init(void);
//uint16_t Adc_Switch(uint32_t ADC_Channel);

//I2C
void ADS115_Init(void);


void Calibrate_pressure_sensor(int16_t* p_zeroPoint);
#endif
