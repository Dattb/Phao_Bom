#include <atmel_start.h>
#include "rang_dong.h"

extern unsigned char RD_rtc_call_back_flag;
bool uart_send_flag_old = 0 ,uart_send_flag_new = 0 ;
unsigned long pum_cnt = 0;
bool boot_flag = 0;
/*extern __attribute__ ((section (".noinit")))*/ rd_data rtc_save_data;
/*extern __attribute__ ((section (".noinit")))*/ rd_mode Mode;  // khai bao bien khong khoi tao



//#define  RD_DEBUG_UART

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

	while (1) {
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
		__builtin_avr_wdr();
	}
}



