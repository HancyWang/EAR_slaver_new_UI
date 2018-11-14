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

/***********************************
* ͷ�ļ�
***********************************/

#include "hardware.h"
#include "stm32f0xx.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_pwr.h"
//#include "stm32f0xx_rtc.h"
#include "stm32f0xx_dma.h"

#include "delay.h"
#include "os_cfg.h"
#include "datatype.h"

#include "time.h"
#include "i2c.h"
#include "comm_task.h"
#include "protocol_module.h"
#include "key_led_task.h"
#include "app.h"
#include "rtc.h"
/**********************************
*�궨��
***********************************/
#define  SAMPLING_CNT 8
/***********************************
* ȫ�ֱ���
***********************************/
unsigned short inner_adc_result[SAMPLING_CNT];
uint16_t RegularConvData_Tab[1];
// uint16_t RegularConvData_Tab[2];  //��ص�ѹ�Ͱ���ģʽ //�°汾�а���ģʽʹ��tact switch������ADC4��

int16_t zero_point_of_pressure_sensor;

extern uint8_t parameter_buf[PARAMETER_BUF_LEN];


extern const uint8_t default_parameter_buf[PARAMETER_BUF_LEN];
extern BOOL b_Is_PCB_PowerOn;
extern uint8_t prev_mode;

extern uint8_t mode;
/***********************************
* �ֲ�����
***********************************/
//static //ƽ����·����ڱ�
//const uint8_t mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};
/***********************************
* �ֲ�����
***********************************/


////�ⲿ����
//extern void I2C_SendByte(INT8U dat);
//extern void I2C_Stop(void); 
/**************************************************************
* ��ʼ��OS�δ�ʱ��
//ʹ��TIM3
**************************************************************/
/*
void init_tim(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  TIM_TimeBaseStructure.TIM_Period = SystemCoreClock/OS_TICKS_PER_SEC;           // �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) 
  TIM_TimeBaseStructure.TIM_Prescaler = 0;	//ʱ��Ԥ��Ƶ�� 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;			//���ϼ���ģʽ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_ClearFlag(TIM3, TIM_FLAG_Update);			        // �������жϱ�־ 
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM3, ENABLE);
}*/

/**************************************************************
* ��ʼ��OS�δ�ʱ��
//ʹ��TIM16
**************************************************************/
void init_tim(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16,ENABLE);//����ʱ��TIM16

  NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  TIM_TimeBaseStructure.TIM_Period = SystemCoreClock/OS_TICKS_PER_SEC-1;           // �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) 
  TIM_TimeBaseStructure.TIM_Prescaler = 0;	//ʱ��Ԥ��Ƶ�� 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;			//���ϼ���ģʽ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;

  TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);

  TIM_ClearFlag(TIM16, TIM_FLAG_Update);			        // �������жϱ�־ 
  TIM_ITConfig(TIM16,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM16, ENABLE);
}

/**************************************************************
* ��ʼLED�ܽ�
**************************************************************/
//void Init_LED(void)
//{
//	GPIO_InitTypeDef  GPIO_InitStructure;
//	//������� �����̵�1
//	GPIO_InitStructure.GPIO_Pin = GREEN_LED_PIN_1;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GREEN_LED_PORT_1, &GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin = YELLO_LED_CTRL_PIN;  //�رջƵ�
//  GPIO_Init(YELLO_LED_PORT, &GPIO_InitStructure);
//}

void Init_PWRSAVE(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	//��ԴPWR_SAVE
	GPIO_InitStructure.GPIO_Pin = KEY_PWR_SAVE_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(KEY_PWR_SAVE_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(KEY_PWR_SAVE_PORT,KEY_PWR_SAVE_PIN);
}

////У��pressure sensor
//void Calibrate_pressure_sensor(int16_t* p_zeroPoint)
//{
//	//����Ǹ�ѹ��ô�죿Ӧ�ö����int16?ֵ�᲻���ɸ�����
//	int16_t arr[10]={0};
//	int16_t sum=0;
//	
//	delay_ms(50); //��ADC�ȶ�
//	for(uint8_t i=0;i<10;i++)
//	{
//		arr[i]=ADS115_readByte(0x90);
//		sum+=arr[i];
//		delay_us(5);
//	}
//	*p_zeroPoint=sum/10;
//}

//PA0,KEY_WAKE_UP ���жϻ���
void Init_KEY_WAKEUP()  
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = KEY_DETECT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(KEY_DETECT_PORT, &GPIO_InitStructure);
}

void show_mode_LED()
{
	switch(mode)
	{
		case 1:
			set_led(LED_CLOSE,TRUE);
			set_led(LED_GREEN_1,TRUE);
			break;
		case 2:
			set_led(LED_CLOSE,TRUE);
			set_led(LED_GREEN_1,TRUE);
			set_led(LED_GREEN_2,TRUE);
			break;
		case 3:
			set_led(LED_CLOSE,TRUE);
			set_led(LED_GREEN_1,TRUE);
			set_led(LED_GREEN_2,TRUE);
			set_led(LED_GREEN_3,TRUE);
			break;
		default:
			break;
	}	
}

//��ʼ��LED��
void Init_LED()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//�Ƶƿ��ƽ� PB11  
	GPIO_InitStructure.GPIO_Pin = YELLO_LED_CTRL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(YELLO_LED_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(YELLO_LED_PORT,YELLO_LED_CTRL_PIN);   //�͵�ƽ�ضϹ���
	
	//LED�� PB8 PB9 PB10
	GPIO_InitStructure.GPIO_Pin = GREEN_LED_PIN_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GREEN_LED_PORT_1, &GPIO_InitStructure);
	//GPIO_ResetBits(GREEN_LED_PORT_1, GREEN_LED_PIN_1);
	
	GPIO_InitStructure.GPIO_Pin = GREEN_LED_PIN_2;
	GPIO_Init(GREEN_LED_PORT_2, &GPIO_InitStructure);
//	GPIO_ResetBits(GREEN_LED_PORT_2, GREEN_LED_PIN_2);
	
	GPIO_InitStructure.GPIO_Pin = GREEN_LED_PIN_3;
	GPIO_Init(GREEN_LED_PORT_3, &GPIO_InitStructure);
//	GPIO_ResetBits(GREEN_LED_PORT_3, GREEN_LED_PIN_3);
	
//	show_mode_LED();
//	set_led(LED_CLOSE,TRUE);
}

//��ʼ��PWR_SAVE,PA5
void Init_PWR_SAVE()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	//��ԴPWR_SAVE PA5
	GPIO_InitStructure.GPIO_Pin = KEY_PWR_SAVE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(KEY_PWR_SAVE_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(KEY_PWR_SAVE_PORT,KEY_PWR_SAVE_PIN);
}

//��ʼ��KEY_MODE,PA4
void Init_KEY_MODE()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = KEY_MODE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(KEY_MODE_PORT, &GPIO_InitStructure);
}

/**************************************************************
* ��ʼ��Ӳ���ܽ�
**************************************************************/
void init_hardware()
{
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

//	//��ʼ��key_wakeup  
//	Init_KEY_WAKEUP();  //PA0
	
	//��ʼ��LED��
	Init_LED();         //PB8,PB9,PB10,PB11
	
	//��ʼ��PWR_SAVE
	Init_PWR_SAVE();  	//PA5
	
	//��ʼ��ADC_BAT,PA1 (PA4������)
	Init_ADC1();   			//PA1
	
	//��ʼ��KEY_MODE,PA4
	Init_KEY_MODE();   //PA4
	
//	prev_mode=GetModeSelected(); //��ʼ����ʱ��õ�������ģʽ

//��ʼ��flash�е�datetime��¼ҳ
	Init_RecordPage();
}

/**************************************************************
* �弶Ӳ����Դ����
**************************************************************/
////�����z�y�Š�B
//BOOL get_exp_status(void)
//{
//	return GPIO_ReadInputDataBit(EXP_DETECT_PORT, EXP_DETECT_PIN);
//}

//�����z�y�Š�B
//TRUE������
//FALSE:����
BOOL get_key_status(void)
{
	return GPIO_ReadInputDataBit(KEY_DETECT_PORT, KEY_DETECT_PIN);
}

//����LEDָʾ��
void set_led(LED_COLOR color,BOOL ON_OFF)
{
	switch(color)
	{
		case LED_CLOSE:   //�ر����е�LED��
			if(ON_OFF)
			{
				GPIO_SetBits(GREEN_LED_PORT_1, GREEN_LED_PIN_1);  //�ر�3���̵�
				GPIO_SetBits(GREEN_LED_PORT_2, GREEN_LED_PIN_2);
				GPIO_SetBits(GREEN_LED_PORT_3, GREEN_LED_PIN_3);
				GPIO_ResetBits(YELLO_LED_PORT, YELLO_LED_CTRL_PIN); //�ر�3���Ƶ�
			}
			break;
		case LED_GREEN_1: //�����̵�1
			if(ON_OFF)
			{
				GPIO_ResetBits(GREEN_LED_PORT_1, GREEN_LED_PIN_1);     //�����̵�1
			}
			else
			{
				GPIO_SetBits(GREEN_LED_PORT_1, GREEN_LED_PIN_1);
			}
			break;
		case LED_GREEN_2://�����̵�2
			if(ON_OFF)
			{
				GPIO_ResetBits(GREEN_LED_PORT_2, GREEN_LED_PIN_2);     //�����̵�2
			}
			else
			{
				GPIO_SetBits(GREEN_LED_PORT_2, GREEN_LED_PIN_2);
			}
			break;	
		case LED_GREEN_3://�����̵�3
			if(ON_OFF)
			{
				GPIO_ResetBits(GREEN_LED_PORT_3, GREEN_LED_PIN_3);     //�����̵�3
			}
			else
			{
				GPIO_SetBits(GREEN_LED_PORT_3, GREEN_LED_PIN_3);
			}
			break;		
		case LED_YELLOW://�����Ƶ�
		if(ON_OFF)
			{
				GPIO_SetBits(YELLO_LED_PORT, YELLO_LED_CTRL_PIN);     //�ߵ�ƽ����
			}
			else
			{
				GPIO_ResetBits(YELLO_LED_PORT, YELLO_LED_CTRL_PIN);
			}
			break;	
		default:
			break;
	}
}
/**************************************************************
* ��ص�ѹ���
**************************************************************/
//uint8_t get_bat_vol_per(void)
//{
//	return 0;

//}


/**************************************************************
* RTC����
**************************************************************/
//RTC RCC
#if 0
void init_rtc_rcc(void)
{
	//����ʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
}

//rTC����
void config_rtc(void)
{
	RCC_LSICmd(ENABLE);//
	
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);//�ȴ�����
	
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
	
	RCC_RTCCLKCmd(ENABLE);
	RTC_WaitForSynchro();
}
#endif
#if 0
////RTC��ʼ��
//void init_rtc(void)
//{
//	RTC_InitTypeDef RTC_InitStructure;
//	RTC_DateTypeDef RTC_DateStrcuture;
//	RTC_TimeTypeDef RTC_TimeStrcuture;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
//	
//	PWR_BackupAccessCmd(ENABLE);
//	
//	uint32_t dr = RTC_ReadBackupRegister(RTC_BKP_DR1);
//	
//	if(dr != 0x5051)//RTC_BKP_DR1
//	{
//		config_rtc();
//		
//		RTC_InitStructure.RTC_AsynchPrediv = 0x63;
//		RTC_InitStructure.RTC_SynchPrediv = 0x18f;
//		RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
//		
//		if(RTC_Init(&RTC_InitStructure) == ERROR)
//		{
//			//while(1);//��ʼ��ʧ�ܣ����޸�
//			//������λ
//			
//		}
//		
//		RTC_DateStrcuture.RTC_Year = 16;
//		RTC_DateStrcuture.RTC_Month = 12;
//		RTC_DateStrcuture.RTC_Date = 23;
//		RTC_TimeStrcuture.RTC_Hours = 12;
//		RTC_TimeStrcuture.RTC_Minutes = 34;
//		RTC_TimeStrcuture.RTC_Seconds = 56;
//		RTC_SetDate(RTC_Format_BIN, &RTC_DateStrcuture);//RTC_Format_BIN
//		RTC_SetTime(RTC_Format_BIN, &RTC_TimeStrcuture);
//		
//		RTC_WriteBackupRegister(RTC_BKP_DR1, 0x5051);
//	}
//	else
//	{
//		RCC_LSICmd(ENABLE);
//		RTC_WaitForSynchro();
//	}
//}

////�ж��Ƿ������꺯��
////�·�   1  2  3  4  5  6  7  8  9  10 11 12
////����   31 29 31 30 31 30 31 31 30 31 30 31
////������ 31 28 31 30 31 30 31 31 30 31 30 31
////year:���
////����ֵ:������ǲ�������.1,��.0,����
//uint8_t Is_Leap_Year(uint16_t year)
//{			  
//	if(year%4==0) //�����ܱ�4����
//	{ 
//		if(year%100==0) 
//		{ 
//			if(year%400==0)return 1;//�����00��β,��Ҫ�ܱ�400���� 	   
//			else return 0;   
//		}else return 1;   
//	}else return 0;	
//}	

////������ת��
//void convert_rtc(_calendar_obj* calendar, uint32_t rtc)
//{
//	uint32_t temp = 0;
//	uint16_t temp1 = 0;
//	static uint16_t daycnt=0;
//	
//	temp = rtc/86400;   //�õ�����(��������Ӧ��)
//	if(daycnt!=temp)//����һ����
//	{	  
//		daycnt=temp;
//		temp1 = 1970;	//��1970�꿪ʼ
//		while(temp>=365)
//		{				 
//			if(Is_Leap_Year(temp1))//������
//			{
//				if(temp>=366)temp-=366;//�����������
//				else break;  
//			}
//			else temp-=365;	  //ƽ�� 
//			temp1++;  
//		}   
//		calendar->w_year=temp1;//�õ����
//		temp1=0;
//		while(temp>=28)//������һ����
//		{
//			if(Is_Leap_Year(calendar->w_year)&&temp1==1)//�����ǲ�������/2�·�
//			{
//				if(temp>=29)temp-=29;//�����������
//				else break; 
//			}
//			else 
//			{
//				if(temp>=mon_table[temp1])temp-=mon_table[temp1];//ƽ��
//				else break;
//			}
//			temp1++;  
//		}
//		calendar->w_month=temp1+1;	//�õ��·�
//		calendar->w_date=temp+1;  	//�õ����� 
//	}
//	temp=rtc%86400;     		//�õ�������   	   
//	calendar->hour=temp/3600;     	//Сʱ
//	calendar->min=(temp%3600)/60; 	//����	
//	calendar->sec=(temp%3600)%60; 	//����
//}

////����RTC
//void set_rtc(uint32_t rtc)
//{
//	RTC_DateTypeDef RTC_DateStrcuture;
//	RTC_TimeTypeDef RTC_TimeStrcuture;

//	_calendar_obj calendar;
//	
//	convert_rtc(&calendar, rtc);
//	
//	RTC_DateStrcuture.RTC_Year = calendar.w_year;
//	RTC_DateStrcuture.RTC_Month = calendar.w_month;
//	RTC_DateStrcuture.RTC_Date = calendar.w_date;
//	RTC_TimeStrcuture.RTC_Hours = calendar.hour;
//	RTC_TimeStrcuture.RTC_Minutes = calendar.min;
//	RTC_TimeStrcuture.RTC_Seconds = calendar.sec;
//	
//	RTC_SetDate(RTC_Format_BIN, &RTC_DateStrcuture);//RTC_Format_BIN
//	RTC_SetTime(RTC_Format_BIN, &RTC_TimeStrcuture);
//}

//uint32_t get_rtc(void)
//{
//#ifndef RTC_TR_RESERVED_MASK
//#define RTC_TR_RESERVED_MASK    ((uint32_t)0x007F7F7F)	
//#endif
//	
//	return (uint32_t)(RTC->TR & RTC_TR_RESERVED_MASK); 
//}
#endif
/**************************************************************
* FLASH�x������
**************************************************************/
//����һ퓔���
BOOL save_one_page_to_flash(uint32_t Address, uint8_t* buf, uint16_t len)
{
	return TRUE;
}
//�xһ퓔���
void read_one_page_from_flash(uint32_t Address, uint8_t* buf, uint16_t len)
{
}
//����һ�����֣����ֹ�
BOOL save_half_word_to_flash(uint32_t Address, uint16_t data)
{
	return TRUE;
}
//�xһ�����֣����ֹ�
void read_half_word_from_flash(uint32_t Address, uint16_t* pdata)
{
}
/**************************************************************
* /EEPROM�x������
**************************************************************/
//����һ�����֔��M
BOOL save_half_word_buf_to_eeprom(uint32_t Address, uint16_t* buf, uint16_t len)
{
	return TRUE;
}
//�xһ�����֔��M
void read_half_word_buf_from_eeprom(uint32_t Address, uint16_t* buf, uint16_t len)
{
}

void Key_WakeUp_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Init_ADC1(void)
{
	GPIO_InitTypeDef    GPIO_InitStructure;
	DMA_InitTypeDef     DMA_InitStructure;
	ADC_InitTypeDef     ADC_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;  //PA4����Ҫ��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);	
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);		
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
	
	ADC_DeInit(ADC1);//ADC???'??�?��??		

  DMA_DeInit(DMA1_Channel1);	/* DMA1 Channel1 Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;//�a��??�
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;//?�'?�??�
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//�a����?a�y?Y'?�?�?�'?'
//  DMA_InitStructure.DMA_BufferSize = 2;//
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�a��??�??'??�2?�?
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//?�'?�??�??'??�2?�?
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//�y?Y?�?�?a16??
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//�y?Y?�?�?a16??
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA_Priority��?�DMA����x�?��?t�??�??
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMA����x??�D��???a?�'?�??�'?'?�?
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
		  

	DMA_Cmd(DMA1_Channel1, ENABLE);/* DMA1 Channel1 enable */			
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular); /* Enable ADC_DMA */	
  ADC_DMACmd(ADC1, ENABLE);  

	ADC_StructInit(&ADC_InitStructure);//3?�??�ADC?�11
	
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12?????�
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //1??�??�?װ??1���?��?D???�?
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; 
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�y?Y?????a��????
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward; //ADC�?�?�??�
  ADC_Init(ADC1, &ADC_InitStructure); 
	 
  ADC_ChannelConfig(ADC1, ADC_Channel_1 , ADC_SampleTime_239_5Cycles); /* Convert the ADC1 Channel 11 with 239.5 Cycles as sampling time */  
//  ADC_ChannelConfig(ADC1, ADC_Channel_4 , ADC_SampleTime_239_5Cycles); /* Convert the ADC1 Channel 11 with 239.5 Cycles as sampling time */  
	
	ADC_GetCalibrationFactor(ADC1); /* ADC Calibration */  
  ADC_Cmd(ADC1, ENABLE);  /* Enable ADCperipheral[PerIdx] */	  
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN)); /* Wait the ADCEN falg */
  ADC_StartOfConversion(ADC1); /* ADC1 regular Software Start Conv */ 
}


//uint16_t Adc_Switch(uint32_t ADC_Channel)
//{
//	
//	//while(ADC_GetFlagStatus(ADC1,ADC_FLAG_ADEN)==RESET); 
//	//����ADC���õ�ͨ���Ͳ�������
//	ADC_ChannelConfig(ADC1,ADC_Channel,ADC_SampleTime_239_5Cycles);  
//	
//	//Delay_ms(10);
//	//��������ADCת��
//  ADC_StartOfConversion(ADC1);  

//	//�ȴ�ADCת�����
//	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC))
//	{
//		//do nothing
//	}
//  return ADC_GetConversionValue(ADC1) ; 
//}
