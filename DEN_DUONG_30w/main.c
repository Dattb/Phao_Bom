#include <atmel_start.h>
#include "rang_dong.h"
#include "nvmctrl_basic.h"
extern unsigned char RD_rtc_call_back_flag;
bool uart_send_flag_old = 0 ,uart_send_flag_new = 0 ;
unsigned long pum_cnt = 0;
bool boot_flag = 0;
unsigned char check_out_train = 0;
extern unsigned char check_in_train;
/*extern __attribute__ ((section (".noinit")))*/ rd_data rtc_save_data;
/*extern __attribute__ ((section (".noinit")))*/ rd_mode Mode;  // khai bao bien khong khoi tao

//#define RESET_TRAINING		// luu y khi san xuat code nay se khong duoc chay, chi chay de test	
//#define  RD_DEBUG_UART

unsigned char flash_training_flag = 0;
unsigned char training_out_cnt = 0;
uint16_t  rtc_cnt = 0;
uint16_t  training_time = 0;
unsigned char counter_toggle = 0;
unsigned char check_training = 0;
uint16_t rtc_out_train_cnt = 0;
extern unsigned char cnt_for_clear;
int main(void)
{
	/* Initializes MCU, drivers and middleware */
 	atmel_start_init();
	rd_led_init();
	rd_pum_init();
	
	#ifdef RD_DEBUG_UART
	USART_0_initialization();
	#endif
	rd_iput_init();	
	if(rtc_save_data.header[0] != 0x06 && rtc_save_data.header[1] != 0x08 ){
		rtc_save_data.header[0] = 0x06;
		rtc_save_data.header[1] = 0x08;
		rtc_save_data.NoInitVar = 0;
	}
	
	if(Mode.header[0] != 0x06 && Mode.header[1] != 0x08 ){
		Mode.header[0] = 0x06;
		Mode.header[1] = 0x08;
		Mode.mode_control = 1;
		Mode.man_new = 1;
		Mode.man_old = 0;
	}
	
	#ifdef RD_DEBUG_UART
 	printf(" system booting NoInitVar = %d ...\n",rtc_save_data.NoInitVar);
 	printf(" system booting mode = %d ...\n",Mode.mode_control);
	#endif
	
	unsigned long main_loop = 0;
	WDT_0_init(WDT_PERIOD_8KCLK_gc);
	#ifdef RESET_TRAINING
		FLASH_0_write_eeprom_byte(FLASH_ADDR,0xff);
	#endif
	flash_training_flag = FLASH_0_read_eeprom_byte(FLASH_ADDR);
	if(flash_training_flag == 0xff) check_training = 1;
	if(check_training)RTC_0_init();
	while (1) {
		if(flash_training_flag == 1){
			main_loop++;
			if(main_loop >= BOOT_TIME){
				main_loop = BOOT_TIME;
				boot_flag = 1;
			}
			if(boot_flag){
				rd_control_led ();
				rd_control_pum();
			}
			
			pum_cnt++;
			if(pum_cnt>3000){
				pum_cnt = 0;
				rtc_save_data.NoInitVar = (unsigned int)(RTC.CNT);
				#ifdef RD_DEBUG_UART
				printf(" system booting NoInitVar = %d ...\n",rtc_save_data.NoInitVar);
				#endif
			}
		}
		else if(flash_training_flag == 0xff) {

			if (rtc_cnt != RTC.CNT){

				rtc_cnt = RTC.CNT;
				training_time++;
				counter_toggle++;
				if(counter_toggle >= TOGGLE_TIME && counter_toggle < (TOGGLE_TIME*2)){
					rd_io_write(RD_PORT_A,LED_AUTO,LED_OFF);
					rd_io_write(RD_PORT_A,LED_MANUAL,LED_OFF);
					rd_io_write(RD_LED_PORT,LED_DAY_PHAO1,LED_OFF);
					rd_io_write(RD_LED_PORT,LED_CAN_PHAO1,LED_OFF);
					rd_io_write(RD_PORT_B,LED_DAY_PHAO2,LED_OFF);
					rd_io_write(RD_PORT_B,LED_CAN_PHAO2,LED_OFF);
					rd_io_write(RD_PORT_B,PUM,0);
				}
				else if(counter_toggle >= (TOGGLE_TIME*2)){
					counter_toggle = 0;
					rd_io_write(RD_PORT_B,PUM,1);
					rd_io_write(RD_PORT_A,LED_AUTO,LED_ON);
					rd_io_write(RD_PORT_A,LED_MANUAL,LED_ON);
					rd_io_write(RD_LED_PORT,LED_DAY_PHAO1,LED_ON);
					rd_io_write(RD_LED_PORT,LED_CAN_PHAO1,LED_ON);
					rd_io_write(RD_PORT_B,LED_DAY_PHAO2,LED_ON);
					rd_io_write(RD_PORT_B,LED_CAN_PHAO2,LED_ON);
				}
			}
			
			if(training_time >= TRAINING_TIME){
				flash_training_flag = 1;
				training_time = 0;
				FLASH_0_write_eeprom_byte(FLASH_ADDR,flash_training_flag);
				rd_blink_led();
			}
			else{
				if(!rd_io_read(RD_PORT_B,BUTTON_PIN)){
					check_out_train++;
					training_out_cnt = 0;
					cnt_for_clear = 0;
					while(!rd_io_read(RD_PORT_B,BUTTON_PIN)){
						
						if (rtc_out_train_cnt != RTC.CNT){
							training_out_cnt++;
							rtc_out_train_cnt = RTC.CNT;
						}
						
						if (training_out_cnt >= 5){
							if(check_out_train >= 5){
								check_out_train = 0;
								flash_training_flag = 1;
								training_time = 0;
								FLASH_0_write_eeprom_byte(FLASH_ADDR,flash_training_flag);
								rd_blink_led();
							}
							else check_out_train = 0;
						}
							__builtin_avr_wdr();
					}
				}
				else{
					training_out_cnt = 0;
				}
			}
		}
		rd_clear_check_ou_in(&check_in_train,&check_out_train);
	__builtin_avr_wdr();
	}
}



