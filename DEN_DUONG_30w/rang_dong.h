/*
 * rang_dong.h
 *
 * Created: 10/25/2021 4:41:19 PM
 *  Author: PC5
 */ 


#ifndef RANG_DONG_H_
#define RANG_DONG_H_


#include <atmel_start.h>



typedef struct {
	unsigned char header[2];
	unsigned int NoInitVar;
}rd_data;


typedef struct {
	unsigned char header[2];
	bool mode_control;
	bool man_new;
	bool man_old;
}rd_mode;

typedef enum {
	RD_PORT_A,
	RD_PORT_B,
	RD_PORT_C
}rd_port_t;

#define  BOOT_TIME 50000

#define AUTO	1
#define MANUAL	0


#define LED_ON	0
#define LED_OFF	!LED_ON
#define RD_LED_PORT RD_PORT_A

#define  BUTTON_PIN		1
#define  PHAO1_PIN		1
#define  PHAO2_PIN		2

#define LED_DAY_PHAO1			6
#define LED_CAN_PHAO1			7
#define LED_DAY_PHAO2			3
#define LED_CAN_PHAO2			2
#define LED_AUTO				4
#define LED_MANUAL				5

#define PUM						0



int8_t WDT_0_init(WDT_PERIOD_t mode);
void rd_iput_init ();
void rd_control_pum ();
bool button_debounce ();
void rd_pum_init();
void rd_control_led ();
void rd_io_set_input(rd_port_t port,unsigned char pin);
bool rd_io_read(rd_port_t port,unsigned char pin);
void rd_led_init();
void rd_io_set_output(rd_port_t port,unsigned char pin);
void rd_io_write(rd_port_t port,unsigned char pin,unsigned char level);

#endif /* RANG_DONG_H_ */