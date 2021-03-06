/*
 * rang_dong.c
 *
 * Created: 10/25/2021 4:41:02 PM
 *  Author: PC5
 */ 



#include "rang_dong.h"
#include "rtc.h"

#define BUTTON_PRESS_TIME	200
//bool mode = 1;

extern unsigned char RD_rtc_call_back_flag;
bool phao1_status = 0;
bool phao2_status = 0;

 rd_data rtc_save_data;  // khai bao bien khong khoi tao
 rd_mode Mode;			  // khai bao bien khong khoi tao
unsigned int button_press_cnt = 0;
unsigned int button_press_flag = 0,button_press_flag_old = 0,button_press_flag_new = 0;

unsigned char training_in_cnt = 0;
extern unsigned char flash_training_flag;
extern uint16_t  training_time;
uint16_t rtc_join_train_cnt = 0;
unsigned char check_in_train = 0;
 unsigned char cnt_for_clear = 0;
bool button_debounce (){
	
	if(!rd_io_read(RD_PORT_B,BUTTON_PIN)){
		button_press_cnt++;
	}
	else button_press_cnt   = 0;
	if(button_press_cnt>=BUTTON_PRESS_TIME) return 1;
	else return 0;
}

void rd_iput_init (){
	rd_io_set_input(RD_PORT_B,BUTTON_PIN);
	rd_io_set_input(RD_PORT_A,PHAO1_PIN);
	rd_io_set_input(RD_PORT_A,PHAO1_PIN);
}
void rd_control_pum (){
	
	Mode.man_new = Mode.mode_control;
	if(Mode.man_old != Mode.man_new){
		Mode.man_old = Mode.man_new;
		if(Mode.man_new == MANUAL){
			printf(" MODE manual = %d ...\n",rtc_save_data.NoInitVar);
			RTC_0_init();
			RTC.CNT  = 5;
		}
	}
	
	if(Mode.mode_control == AUTO){
		if(phao2_status){
			if(phao1_status) rd_io_write(RD_PORT_B,PUM,0);
			else if(!phao1_status) rd_io_write(RD_PORT_B,PUM,1);
		}
		else rd_io_write(RD_PORT_B,PUM,0);
	}
	else if(Mode.mode_control == MANUAL){
		if(RD_rtc_call_back_flag){
			rtc_save_data.NoInitVar = 10000;
			rd_io_write(RD_PORT_B,PUM,0);
			Mode.mode_control = AUTO;
		}
		else{
			if(phao2_status) rd_io_write(RD_PORT_B,PUM,1);
			else rd_io_write(RD_PORT_B,PUM,0);
		}
	}
}

void rd_blink_led(){
	for(unsigned char j =0;j<5;j++){
		for(unsigned int i = 0;i<2000;i++){
			rd_io_write(RD_PORT_A,LED_AUTO,LED_ON);
			rd_io_write(RD_PORT_A,LED_MANUAL,LED_ON);
			rd_io_write(RD_LED_PORT,LED_DAY_PHAO1,LED_ON);
			rd_io_write(RD_LED_PORT,LED_CAN_PHAO1,LED_ON);
			rd_io_write(RD_PORT_B,LED_DAY_PHAO2,LED_ON);
			rd_io_write(RD_PORT_B,LED_CAN_PHAO2,LED_ON);
		}
		for(unsigned int i = 0;i<2000;i++){
			rd_io_write(RD_PORT_A,LED_AUTO,LED_OFF);
			rd_io_write(RD_PORT_A,LED_MANUAL,LED_OFF);
			rd_io_write(RD_LED_PORT,LED_DAY_PHAO1,LED_OFF);
			rd_io_write(RD_LED_PORT,LED_CAN_PHAO1,LED_OFF);
			rd_io_write(RD_PORT_B,LED_DAY_PHAO2,LED_OFF);
			rd_io_write(RD_PORT_B,LED_CAN_PHAO2,LED_OFF);
		}
	}
}

void rd_control_led (){	
	if(!rd_io_read(RD_PORT_B,BUTTON_PIN)){
		check_in_train++;
		training_in_cnt = 0;
		cnt_for_clear = 0;
		while(!rd_io_read(RD_PORT_B,BUTTON_PIN)){
			if (rtc_join_train_cnt != RTC.CNT){
				training_in_cnt++;
				rtc_join_train_cnt = RTC.CNT;
			}

			if(!rd_io_read(RD_PORT_B,BUTTON_PIN)){
				if (training_in_cnt >= 5){
					
					if(check_in_train >= 5){
						
						if(flash_training_flag == 1) flash_training_flag = 0xff;
						training_time = 0;
						FLASH_0_write_eeprom_byte(FLASH_ADDR,flash_training_flag);
						rd_blink_led();
						check_in_train = 0;
					}
					else{
						check_in_train = 0;
					}
				}
			}
			else{
				training_in_cnt = 0;
			}
			__builtin_avr_wdr();
		}
		RD_rtc_call_back_flag = 0;
		if(Mode.mode_control) Mode.mode_control = 0;
		else Mode.mode_control = 1;
	}
	else{
		training_in_cnt = 0;
	}
	if(flash_training_flag == 1){
		if(Mode.mode_control){
			rd_io_write(RD_PORT_A,LED_AUTO,LED_ON);
			rd_io_write(RD_PORT_A,LED_MANUAL,LED_OFF);
		}
		else {
			rd_io_write(RD_PORT_A,LED_AUTO,LED_OFF);
			rd_io_write(RD_PORT_A,LED_MANUAL,LED_ON);
		}
		
		if(rd_io_read(RD_PORT_A,PHAO1_PIN))	{
			phao1_status = 1;
			rd_io_write(RD_LED_PORT,LED_DAY_PHAO1,LED_ON);
			rd_io_write(RD_LED_PORT,LED_CAN_PHAO1,LED_OFF);
		}
		else{
			
			phao1_status = 0;
			rd_io_write(RD_LED_PORT,LED_DAY_PHAO1,LED_OFF);
			rd_io_write(RD_LED_PORT,LED_CAN_PHAO1,LED_ON);
		}
		
		if(rd_io_read(RD_PORT_A,PHAO2_PIN)){
			
			phao2_status = 1;
			rd_io_write(RD_PORT_B,LED_DAY_PHAO2,LED_ON);
			rd_io_write(RD_PORT_B,LED_CAN_PHAO2,LED_OFF);
			
		}
		else{
			phao2_status = 0;
			rd_io_write(RD_PORT_B,LED_DAY_PHAO2,LED_OFF);
			rd_io_write(RD_PORT_B,LED_CAN_PHAO2,LED_ON);
		}
	}
		
}




void rd_pum_init(){
	rd_io_set_output(RD_PORT_B,PUM);
	rd_io_write(RD_PORT_B,PUM,0);
}
void rd_led_init(){
	
	rd_io_set_output(RD_PORT_A,LED_DAY_PHAO1);
	rd_io_set_output(RD_PORT_A,LED_CAN_PHAO1);
	rd_io_set_output(RD_PORT_B,LED_DAY_PHAO2);
	rd_io_set_output(RD_PORT_B,LED_CAN_PHAO2);
	rd_io_set_output(RD_PORT_A,LED_AUTO);
	rd_io_set_output(RD_PORT_A,LED_MANUAL);
}

void rd_io_set_output(rd_port_t port,unsigned char pin){
	if(port == RD_PORT_A) VPORTA.DIR |= (1<<pin);
	else if(port == RD_PORT_B) VPORTB.DIR |= (1<<pin);
	else if(port == RD_PORT_C) VPORTC.DIR |= (1<<pin);
}

void rd_io_write(rd_port_t port,unsigned char pin,unsigned char level){
	if(port == RD_PORT_A){
		level&1? (VPORTA.OUT |= (1<<pin)):(VPORTA.OUT &= ~(1<<pin));
	}
	else if(port == RD_PORT_B) {
		level&1? (VPORTB.OUT |= (1<<pin)):(VPORTB.OUT &= ~(1<<pin));
	}
	else if(port == RD_PORT_C){
		level&1? (VPORTC.OUT |= (1<<pin)):(VPORTC.OUT &= ~(1<<pin));
	}
	
}


void rd_io_set_input(rd_port_t port,unsigned char pin){
	if(port == RD_PORT_A) VPORTA.DIR &= ~(1<<pin);
	else if(port == RD_PORT_B) VPORTB.DIR &= ~(1<<pin);
	else if(port == RD_PORT_C) VPORTC.DIR &= ~(1<<pin);
}


bool rd_io_read(rd_port_t port,unsigned char pin){
	if(port == RD_PORT_A){
		if(VPORTA.IN & (1<<pin)) return 1;
		else return 0;
	}
	else if(port == RD_PORT_B) {
		if(VPORTB.IN & (1<<pin)) return 1;
		else return 0;
	}
	else if(port == RD_PORT_C){
		if(VPORTC.IN & (1<<pin)) return 1;
		else return 0;
	}
	return -1;
}


int8_t WDT_0_init(WDT_PERIOD_t mode)
{

	ccp_write_io((void *)&(WDT.CTRLA),
	mode /* 8 cycles (8ms) */
	| WDT_WINDOW_OFF_gc /* Window mode off */);

	return 0;
}
void rd_clear_check_ou_in(unsigned char *data1,unsigned char *data2){
	static uint16_t check = 0;

	if(check != RTC.CNT){
		check = RTC.CNT;
		cnt_for_clear++;
		if(cnt_for_clear%2==0){
			*data1 = 0;
			*data2 = 0;	
			cnt_for_clear = 0;
		}
		
		
	}
}