#ifndef _HONEYWELL_SAMPLING_DATA
#define _HONEYWELL_SAMPLING_DATA

#include "datatype.h"

#define HONEYWELL_RATE			11185   //斜率,根据公式算出来的
#define PRESSURE_SAFETY_THRESHOLD 20   //20mmHg，最大过压值
#define PRESSURE_SENSOR_VALUE(x) (((HONEYWELL_RATE)*(x))+(HONEYWELL_ZERO_POINT))


#define HONEYWELL_SAMPLING_DATA_PERIOD 10

typedef enum 
{
	HONEYWELL_START,
	HONEYWELL_NONE,
//	HONEYWELL_WAIT_5ms,
	HONEYWELL_READ_DATA,
	HONEYWELL_SAMPLE_DATA_FINISH
}HONEYWELL_STATE;

void honeywell_sampling_data(void);
UINT32 trans_xmmHg_2_adc_value(UINT8 xmmHg);
#endif //_HONEYWELL_SAMPLING_DATA


