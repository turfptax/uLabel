#include "nrf.h"
#include "adc_read.h"
#include "fft_opt.h"
#include "fast_math.h"

uint16_t adc_buf[24];
uint16_t res_buf[6];
volatile uint8_t has_new_data = 0;

ADC_PIN_CONFIG_REG conf_Hx;
ADC_PIN_CONFIG_REG conf_bat;

void adc_init()
{
	NRF_SAADC->ENABLE = 1;
	NRF_SAADC->CH[0].PSELP = adc_ain0;
	NRF_SAADC->CH[0].PSELN = adc_ain_nc;
	NRF_SAADC->CH[1].PSELP = adc_ain1;
	NRF_SAADC->CH[1].PSELN = adc_ain_nc;
	NRF_SAADC->CH[2].PSELP = adc_ain2;
	NRF_SAADC->CH[2].PSELN = adc_ain_nc;
	NRF_SAADC->CH[3].PSELP = adc_ain4; //H3 sensor on AIN4, not AIN3
	NRF_SAADC->CH[3].PSELN = adc_ain_nc;
	NRF_SAADC->CH[4].PSELP = adc_ain3; //H4 sensor on AIN3, not AIN4
	NRF_SAADC->CH[4].PSELN = adc_ain_nc;
	NRF_SAADC->CH[5].PSELP = adc_ain5; //battery on AIN5
	NRF_SAADC->CH[5].PSELN = adc_ain_nc;

	conf_Hx.fields.RESP = 0;
	conf_Hx.fields.RESN = 1; //pulldown
	conf_Hx.fields.GAIN = adc_gain_1_6;
	conf_Hx.fields.MODE_DIFF = 0;
	conf_Hx.fields.REF_VDD4 = 0;
	conf_Hx.fields.TACQ = adc_taq_40us;
	conf_Hx.fields.BURST = 1;
	NRF_SAADC->CH[0].CONFIG = conf_Hx.value;	
	NRF_SAADC->CH[1].CONFIG = conf_Hx.value;	
	NRF_SAADC->CH[2].CONFIG = conf_Hx.value;	
	NRF_SAADC->CH[3].CONFIG = conf_Hx.value;	
	NRF_SAADC->CH[4].CONFIG = conf_Hx.value;	
	NRF_SAADC->CH[5].CONFIG = conf_Hx.value;	
	NRF_SAADC->RESOLUTION = adc_res_14;
//	NRF_SAADC->OVERSAMPLE = adc_oversample_256x; //(5us + 2us) x 256 oversample -> ~558 Hz sample rate
//	NRF_SAADC->OVERSAMPLE = adc_oversample_128x; //(5us + 2us) x 128 oversample -> ~1116 Hz sample rate
	NRF_SAADC->OVERSAMPLE = adc_oversample_off; //can't use oversampling with scanning, 42us per channel, 6 channels -> ~3968Hz sample rate
	//sample rate is effectively 3968/4=992Hz due to 4x data points captured per interrupt
	NRF_SAADC->SAMPLERATE = 0; //control from task
	ADC_INT_REG intcfg;
	intcfg.value = 0;
	intcfg.fields.END = 1;
//	intcfg.fields.DONE = 1;
	intcfg.fields.RESULTDONE = 1;
	NRF_SAADC->INTEN = intcfg.value;
	NRF_SAADC->RESULT.PTR = adc_buf;
	NRF_SAADC->RESULT.MAXCNT = 24; //6 channels * 4 times
	NVIC_EnableIRQ(SAADC_IRQn);
	NRF_SAADC->EVENTS_STARTED = 0;
	NRF_SAADC->TASKS_START = 1;
	while(!NRF_SAADC->EVENTS_STARTED) ;
	NRF_SAADC->TASKS_SAMPLE = 1;
}

volatile int batt_skip_cnt = 999; //so we will read battery right after start

void SAADC_IRQHandler()
{
	if(NRF_SAADC->EVENTS_RESULTDONE && !NRF_SAADC->EVENTS_END)
	{
		NRF_SAADC->EVENTS_RESULTDONE = 0;
		NRF_SAADC->TASKS_SAMPLE = 1;
	}
//	NRF_SAADC->EVENTS_END = 0;
//	NRF_SAADC->EVENTS_DONE = 0;
//	NRF_SAADC->EVENTS_RESULTDONE = 0;
	if(NRF_SAADC->EVENTS_END)
	{
		NRF_SAADC->EVENTS_END = 0;
		NRF_SAADC->EVENTS_RESULTDONE = 0;
		for(int x = 0; x < 6; x++)
		{
			res_buf[x] = adc_buf[x] + adc_buf[6+x] + adc_buf[2*6+x] + adc_buf[3*6+x];
		}
		has_new_data = 1;
		NRF_SAADC->RESULT.PTR = adc_buf;
		NRF_SAADC->RESULT.MAXCNT = 24;

		NRF_SAADC->EVENTS_STARTED = 0;
		NRF_SAADC->TASKS_START = 1;
		while(!NRF_SAADC->EVENTS_STARTED) ;
		NRF_SAADC->TASKS_SAMPLE = 1;			
	}
}

uint8_t adc_has_data()
{
	return has_new_data;	
}

int adc_get_data(uint16_t *raw_data)
{
	if(!has_new_data) return 0;
	for(int x = 0; x < 6; x++)
		raw_data[x] = res_buf[x];
	
	has_new_data = 0;
	return 8;
}
uint32_t adc_read_battery()
{
	int bat_mv = res_buf[5] * 0.25 * 0.65918f;// / 16.384/4 * 0.6 * 6 * 3
	return bat_mv;
}

