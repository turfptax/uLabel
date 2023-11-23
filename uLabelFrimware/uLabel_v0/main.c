/**
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "nrf.h"
#include "leds.h"
#include "urf_radio.h"
#include "urf_star_protocol.h"
//#include "urf_ble_peripheral.h"
#include "ble_const.h"
#include "urf_timer.h"
#include "adc_read.h"
//#include "fft_opt.h"
#include "quat_math.h"
#include "lsm6ds3.h"
//#include "qmc7983.h" 
#include "persistent_storage.h"

uint8_t pin_button = 19;
uint8_t pin_syson = 4; 

sDevice_state dev_state;
uint32_t dev_state_changed_time = 0;
uint8_t dev_state_changed = 0;

uint8_t gyro_calibration_success = 0;
uint32_t gyro_calibration_end_time = 0;
uint8_t magnetic_calibration_started = 0;
uint32_t magnetic_calibration_start_time = 0;


float battery_mv = 3500; //by default non zero to prevent power off before actual measurement
int battery_low_threshold = 3200;

uint8_t cr_battery_mode = 0;
uint8_t led_out_enabled = 1;

uint8_t battery_level = 0;
uint8_t packet_id = 0;
uint8_t data_packet[64];

uint8_t unsent_cnt = 0;

uint8_t sensors_data_id = 0;
uint16_t hall_sensors_buf[5];

uint8_t param_send_id = 0;

enum param_sends
{
	param_batt = 0,
	param_end
};

void prepare_data_packet()
{
	uint8_t idx = 0;
	packet_id++;
	if(packet_id > 128) packet_id = 0;
	data_packet[idx++] = packet_id;
	data_packet[idx++] = 0; //fill in the end
	uint32_t unit_id = NRF_FICR->DEVICEID[1];
	data_packet[idx++] = (unit_id>>24)&0xFF;
	data_packet[idx++] = (unit_id>>16)&0xFF;
	data_packet[idx++] = (unit_id>>8)&0xFF;
	data_packet[idx++] = (unit_id)&0xFF;

	param_send_id++;
	if(param_send_id == param_end) param_send_id = 0;

	param_send_id = param_batt; //force for now
	
	data_packet[idx++] = param_send_id;
	if(param_send_id == param_batt)
	{
		int bat_mv = adc_read_battery();
		if(bat_mv < 2000)
			battery_level = 0;
		else
			battery_level = (bat_mv - 2000)/10;
		if(bat_mv > 4500) battery_level = 255;
		data_packet[idx++] = battery_level;
		data_packet[idx++] = 101; //version_id
		data_packet[idx++] = 0;
	}
	
	data_packet[idx++] = sensors_data_id;
	for(int n = 0; n < 5; n++)
	{
		data_packet[idx++] = hall_sensors_buf[n]>>8;
		data_packet[idx++] = hall_sensors_buf[n]&0xFF;
	}
	int16_t qww, qwx, qwy, qwz;
	int16_t raX, raY, raZ;
	int16_t rwX, rwY, rwZ;
	lsm_get_quat_packed(&qww, &qwx, &qwy, &qwz);
	lsm_get_acc(&raX, &raY, &raZ);
	lsm_get_W(&rwX, &rwY, &rwZ);
	
	data_packet[idx++] = qww>>8;
	data_packet[idx++] = qww&0xFF;
	data_packet[idx++] = qwx>>8;
	data_packet[idx++] = qwx&0xFF;
	data_packet[idx++] = qwy>>8;
	data_packet[idx++] = qwy&0xFF;
	data_packet[idx++] = qwz>>8;
	data_packet[idx++] = qwz&0xFF;

	data_packet[idx++] = raX>>8;
	data_packet[idx++] = raX&0xFF;
	data_packet[idx++] = raY>>8;
	data_packet[idx++] = raY&0xFF;
	data_packet[idx++] = raZ>>8;
	data_packet[idx++] = raZ&0xFF;
	
	int16_t roll = lsm_get_roll();
	int16_t yaw = lsm_get_yaw();
	int16_t pitch = lsm_get_pitch();

	data_packet[idx++] = yaw>>8;
	data_packet[idx++] = yaw&0xFF;
	data_packet[idx++] = pitch>>8;
	data_packet[idx++] = pitch&0xFF;
	data_packet[idx++] = roll>>8;
	data_packet[idx++] = roll&0xFF;

	data_packet[1] = idx;
}


uint16_t sensors_raw[6];

int push_adc_data()
{
	if(adc_get_data(sensors_raw))
	{
		for(int n = 0; n < 5; n++)
			hall_sensors_buf[n] = sensors_raw[n];
		return 1;
	}
	return 0;
}


void fast_clock_start()
{
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}
}
void slow_clock_start()
{
	NRF_CLOCK->LFCLKSRC = 0;
	NRF_CLOCK->TASKS_LFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) {}
}
void fast_clock_stop()
{
	slow_clock_start();
//	NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
//	NRF_CLOCK->TASKS_LFCLKSTART = 1;
//	while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) {}
	NRF_CLOCK->TASKS_HFCLKSTOP = 1;
}


void mode_lowbatt()
{
//	set_led(0);
//	nrf_delay_ms(1);
//	mcp3912_turnoff();
	time_stop();
	fast_clock_stop();
	NRF_SPI0->ENABLE = 0;
//	leds_pulse(0, 0, 0, 0, 1);
	delay_ms(2);
	NRF_GPIO->OUTCLR = 1<<pin_syson;
//	nrf_delay_ms(1);
//	NRF_POWER->TASKS_LOWPWR = 1;
//	NRF_POWER->DCDCEN = 1;
//	NRF_POWER->SYSTEMOFF = 1;
}
void stop_rtc()
{
	NRF_RTC1->TASKS_STOP = 1;
}
void start_rtc()
{
	stop_rtc();
	NRF_RTC1->TASKS_CLEAR = 1;
	NRF_RTC1->CC[0] = 220;
	NRF_RTC1->CC[1] = 0xFFFF;
	NRF_RTC1->CC[2] = 0xFFFF;
	NRF_RTC1->CC[3] = 0xFFFF;
	NRF_RTC1->PRESCALER = 0x3CFFFF;
	NRF_RTC1->INTENSET = (1<<16); //CC0 event
	NVIC_EnableIRQ(RTC1_IRQn);
	NRF_RTC1->TASKS_START = 1;
}
void RTC1_IRQHandler(void)
{
	/* Update compare counter */
	if (NRF_RTC1->EVENTS_COMPARE[0] != 0)
	{
		NRF_RTC1->EVENTS_COMPARE[0] = 0;
		NRF_RTC1->TASKS_CLEAR = 1;  // Clear Counter		    
//		if(U32_delay_ms) U32_delay_ms--; // used in V_hw_delay_ms()
		__SEV(); // to avoid race condition
		stop_rtc();
	}
}

void mode_idle()
{
//	leds_pause();
	time_pause();
	NRF_POWER->TASKS_LOWPWR = 1;
}
void mode_resume_idle()
{
	NRF_POWER->TASKS_CONSTLAT = 1;
	time_resume();
//	leds_resume();
}

void low_power_cycle()
{
	NRF_RADIO->POWER = 0;
//	mode_idle();
	__WFI();
//	mode_resume_idle();
	NRF_RADIO->POWER = 1;
}


enum
{
	radio_mode_fast32 = 0,
	radio_mode_ble,
	radio_mode_fast64
};

uint8_t radio_mode = radio_mode_fast64;

void switch_to_fr64()
{
	rf_disable();
	rf_override_irq(NULL);
	schedule_event_stop();
	NRF_RADIO->POWER = 0;
	rf_dettach_rx_irq();
	rf_dettach_tx_irq();
	for(int x = 0; x < 3; x++)
	{
		leds_pulse(0, 255, 0, 0, 150); 
		NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
		delay_ms(300);
	}
	
	star_init(21, 1000, 2000, 0);
}

uint32_t prev_short_time = 0;

void process_radio_switch()
{
	/*
	if(radio_mode == radio_mode_fast64)
	{
		radio_mode = radio_mode_fast32;
		switch_to_fr32();
		dev_state.fields.radio_mode = radio_mode;
		dev_state_changed = 1;
		dev_state_changed_time = millis();
		return;
	}
	if(radio_mode == radio_mode_fast32)
	{
		radio_mode = radio_mode_ble;
		switch_to_ble();
		dev_state.fields.radio_mode = radio_mode;
		dev_state_changed = 1;
		dev_state_changed_time = millis();
		return;
	}
	if(radio_mode == radio_mode_ble)
	{
		radio_mode = radio_mode_fast64;
		switch_to_fr64();
		dev_state.fields.radio_mode = radio_mode;
		dev_state_changed = 1;
		dev_state_changed_time = millis();
		return;
	}*/
//	emg_raw_data_send = !emg_raw_data_send;
}

void process_btn_long()
{
	NRF_GPIO->OUTCLR = 1<<pin_syson;
}

void process_btn_read()
{
	static uint8_t btn_pressed = 0;
	static uint8_t short_press_pending = 0;
	static uint8_t conseq_short_presses = 0;
	static uint32_t short_press_issued_time = 0;
	static uint32_t btn_on = 0;
	static uint32_t btn_off = 0;
	uint32_t ms = millis();
	if(NRF_GPIO->IN & 1<<pin_button)
	{
		if(!btn_pressed)
		{
			btn_pressed = 1;
			btn_on = ms;
		}
		if(ms - btn_on > 25 && ms - btn_on < 1000)
			leds_pulse(0, 0, 255, -1, 30);
		if(ms - btn_on > 1000 && ms - btn_on < 5000)
			leds_pulse(255, 0, 0, -1, 30);
		if(ms - btn_on > 5000)
			leds_pulse(255, 0, 255, -1, 30);
	}
	else
	{
		if(btn_pressed)
		{
			btn_pressed = 0;
			btn_off = ms;
			uint32_t btn_time = btn_off - btn_on;
			if(btn_time > 25) //ignore too short presses - noise
			{
				if(btn_time < 1000)
				{
					if(ms - short_press_issued_time < 400) conseq_short_presses++;
					else conseq_short_presses = 0;
					short_press_pending = 1;
					short_press_issued_time = ms;
				}
				else if(btn_time < 5000)
					process_btn_long();
			}
		}
	}

	if(short_press_pending && ms - short_press_issued_time > 500)
	{
		if(conseq_short_presses >= 10)
		{
			uint8_t gx, gy, gz;
			lsm_get_zero_offsets_packed(&gx, &gy, &gz);
			dev_state.fields.zero_wx_packed = gx;
			dev_state.fields.zero_wy_packed = gy;
			dev_state.fields.zero_wz_packed = gz;
			dev_state_changed = 1;
			dev_state_changed_time = millis();
			for(int x = 0; x < 20; x++)
			{
				leds_pulse(255, 255, 0, 0, 150); 
				NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
				delay_ms(300);
			}
		}
		else process_radio_switch();
		short_press_pending = 0;
	}
}

int main(void)
{
	NRF_GPIO->DIRSET = 1<<pin_syson;
	NRF_GPIO->OUTSET = 1<<pin_syson;
	NRF_POWER->DCDCEN = 1;
	
	NRF_WDT->CRV = 1*32768; //1 second timeout
	NRF_WDT->TASKS_START = 1;
	
	NRF_UICR->NFCPINS = 0;

	NRF_GPIO->PIN_CNF[pin_button] = 0;

	fast_clock_start();
	time_start();
	NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
	leds_init(8, 7, 6, 26);
	NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
	lsm_init(14, 13, 15, 16, 17);

	for(int x = 0; x < 3; x++)
	{
		leds_pulse((x==0)*255, (x==1)*255, (x==2)*255, 0, 250); 
		NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
		delay_ms(300);
	}
	
	NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
//	star_init(21, 1000, 2000, 0);

	dev_state = read_current_state();
	radio_mode = radio_mode_fast64;
	if(dev_state.fields.zero_wx_packed != 0xFF)
	{
		lsm_set_zero_offsets_packed(dev_state.fields.zero_wx_packed, dev_state.fields.zero_wy_packed, dev_state.fields.zero_wz_packed);
	}
		
	switch_to_fr64();

	adc_init();

	uint32_t last_sent_ms = 0;

	NRF_RNG->TASKS_START = 1;
	NRF_RNG->SHORTS = 0;
	NRF_RNG->CONFIG = 1;
		
	uint32_t unit_id = NRF_FICR->DEVICEID[1];
	
	int cur_rf32_dt = 10;
	
	star_set_id(unit_id);
	
	uint8_t zerow_test_passed = 0;
	
	NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
	delay_ms(50); //make sure battery was checked
	battery_mv = adc_read_battery();
	if(battery_mv < 3400)
	{
		cr_battery_mode = 1;
		for(int x = 0; x < 3; x++)
		{
			leds_pulse(255, 0, 0, 0, 300); 
			NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
			delay_ms(500);
		}
	}
	while(1)
	{
		NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
		if(radio_mode == radio_mode_fast64) star_loop_step();
//		low_power_cycle();
		uint32_t ms = millis();
		
		if(!zerow_test_passed)
		{
			if(ms > 20000)
			{
				if(lsm_get_zero_test_result()) //test passed
				{
					uint8_t gx, gy, gz;
					lsm_get_zero_offsets_packed(&gx, &gy, &gz);
					int need_upd = 0;
					int dx, dy, dz;
					dx = dev_state.fields.zero_wx_packed - gx;
					dy = dev_state.fields.zero_wy_packed - gy;
					dz = dev_state.fields.zero_wz_packed - gz;
					dx *= dx; dy *= dy; dz *= dz;
					if(dx > 1 || dy || dz > 1) need_upd = 1;
					gyro_calibration_success = 1;
					if(need_upd)
					{
						dev_state.fields.zero_wx_packed = gx;
						dev_state.fields.zero_wy_packed = gy;
						dev_state.fields.zero_wz_packed = gz;
						lsm_set_zero_offsets_packed(dev_state.fields.zero_wx_packed, dev_state.fields.zero_wy_packed, dev_state.fields.zero_wz_packed);
						update_current_state(dev_state);	
					}
					if(gyro_calibration_success)
					{
						for(int x = 0; x < 3; x++)
						{
							leds_pulse(255, 255, 0, 0, 150); 
							NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
							delay_ms(300);
						}
						gyro_calibration_end_time = millis();
					}
				}
				zerow_test_passed = 1;
			}
		}
		if(dev_state_changed)
		{
			if(ms - dev_state_changed_time > 10000)
			{
				update_current_state(dev_state);
				dev_state_changed = 0;
			}
		}
		
		process_btn_read();
		
		if(adc_has_data())
		{
			lsm_read();
		}
		
		if(push_adc_data())
		{
//			adc_cnt++;
			if(unsent_cnt < 30) unsent_cnt++;
			battery_mv = adc_read_battery();
			static int low_bat_cnt = 0;
			if(battery_mv < battery_low_threshold && !cr_battery_mode) //don't turn off due to low battery when running on CR2032
			{
				low_bat_cnt++;
				if(low_bat_cnt > 100)
				{
					mode_lowbatt();
				}
			}
			else
				low_bat_cnt = 0;
		}		
		if(radio_mode == radio_mode_fast64)
		{
			if(unsent_cnt > 0)
			{
				prepare_data_packet();
				star_queue_send(data_packet, data_packet[1]);
//				rf_send_and_listen(data_packet, data_packet[1]);
				last_sent_ms = ms;
				unsent_cnt = 0;
			}
		}
//		if((ms%1000) == 499) leds_pulse(0, 255*(!had_adc), 255*had_adc, 10), had_adc = 0;
	}
}

