#ifndef _CONFIG_
#define _CONFIG_

#include "stm32f4xx_conf.h"




//QUAD

#define timerMicros()				TIM5->CNT

#define FRQ_MPU						500.0f			// sample frequency in Hz
#define DT_CYCLE_MPU              			(float)(1.0f/500.0f)
#define DT_UKF             				(float)(1.0f/200.0f)
#define RAD2DEG 					(float)(180.0/M_PI)
#define DEG2RAD						(float)(M_PI/180.0)
/*** PRIORITET INTERRUPTS ***/
/*
 * 	ПРИОРИТЕТЫ ПРЕРЫВАНИЙ, ИСПОЛЬЗУЮЩИЕ ФУНКЦИИ FRERRTOS НЕЛЬЗЯ ДЕЛАТЬ ВЫШЕ(МЕНЬШЕ НОМЕР),
 * 	ЧЕМ configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
 */
#define PRIORITET_I2C 			2
#define PRIORITET_SPI_INS 		3
#define PRIORITET_ADC 			6
#define PRIORITET_UART_BLUETOOTH 	7
#define PRIORITET_UART_FRSKY	 	7
#define PRIORITET_UART_GPS 		4
#define PRIORITET_PPM 			3
#define PRIORITET_PWM 			3
/**PRIORITET TASKS**/
/*
 * ЧЕМ ВЫШЕ ПРИОРИТЕТ, ТЕМ ВЫШЕ НОМЕР(НАОБОРОТ В ОТЛИЧИИ ОТ ПРЕРЫВАНИЙ)
 *
 *
 */
#define PRIORITET_TASK_OPERATION_ADC				4
#define PRIORITET_TASK_REQUEST_ADC				2
#define PRIORITET_TASK_PROCESSING_MAG_BARO			4
#define PRIORITET_TASK_REQUEST_MAG_BARO				3
#define PRIORITET_TASK_ERR_MAG_BARO				3

#define PRIORITET_TASK_COLLECT_SD_PID				5
#define PRIORITET_TASK_COLLECT_SD_MAG_BARO			2
#define PRIORITET_TASK_COLLECT_SD_ADC				2
#define PRIORITET_TASK_COLLECT_SD_PPM				2
#define PRIORITET_TASK_COLLECT_SD_PWM				2
#define PRIORITET_TASK_COLLECT_SD_FLAGS				2
#define PRIORITET_TASK_COLLECT_SD_GPS				2
#define PRIORITET_TASK_COLLECT_SD_UKF				3
#define PRIORITET_TASK_TRANSIVER_SD					1


#define PRIORITET_TASK_PARCEL_TO_COMP				4
#define PRIORITET_TASK_GPS					5
#define PRIORITET_TASK_INS					6
#define PRIORITET_TASK_UKF					3
#define PRIORITET_TASK_ARMING					2
#define PRIORITET_TASK_BUZZER					2
#define PRIORITET_TASK_LEDS					2

////////////////////

/***BUZZER***/
#define BUZZER_FLAG_VALUE_TRANSMITTED 			1
#define BUZZER_DLIT_SIGNAL_VALUE_TRANSMITTED 		5
#define BUZZER_DLIT_PAUSE_VALUE_TRANSMITTED 		2
#define BUZZER_NUMBER_POVTOR_VALUE_TRANSMITTED 		2

#define BUZZER_FLAG_VOLTAGE_LOW 			2
#define BUZZER_DLIT_SIGNAL_VOLTAGE_LOW  		40
#define BUZZER_DLIT_PAUSE_VOLTAGE_LOW 			20
#define BUZZER_NUMBER_POVTOR_VOLTAGE_LOW 		2

#define BUZZER_FLAG_FAIL_SENSOR				3
#define BUZZER_DLIT_SIGNAL_FAIL_SENSOR			2
#define BUZZER_DLIT_PAUSE_FAIL_SENSOR	 		2
#define BUZZER_NUMBER_POVTOR_FAIL_SENSOR		-1

#define BUZZER_FLAG_ARMING_DISARMING			4
#define BUZZER_DLIT_SIGNAL_ARMING_DISARMING 		10
#define BUZZER_DLIT_PAUSE_ARMING_DISARMING 		2
#define BUZZER_NUMBER_POVTOR_ARMING_DISARMING		2

#define BUZZER_FLAG_FAIL_SD				5
#define BUZZER_DLIT_SIGNAL_FAIL_SD	 		10
#define BUZZER_DLIT_PAUSE_FAIL_SD	 		10
#define BUZZER_NUMBER_POVTOR_FAIL_SD			-1

#define BUZZER_FLAG_LIFT_COMMAND 			6
#define BUZZER_DLIT_SIGNAL_LIFT_COMMAND 		3
#define BUZZER_DLIT_PAUSE_LIFT_COMMAND 			10
#define BUZZER_NUMBER_POVTOR_LIFT_COMMAND  		1

#define BUZZER_FLAG_GLITCH 				7
#define BUZZER_DLIT_SIGNAL_GLITCH 	 		20
#define BUZZER_DLIT_PAUSE_GLITCH  			10
#define BUZZER_NUMBER_POVTOR_GLITCH 	  		4

#define BUZZER_FLAG_NOT_MODE 				8
#define BUZZER_DLIT_SIGNAL_NOT_MODE 	 		60
#define BUZZER_DLIT_PAUSE_NOT_MODE  			1
#define BUZZER_NUMBER_POVTOR_NOT_MODE 	  		1


#define __sqrtf(a)	sqrtf(a)
#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)

extern SemaphoreHandle_t xSD_FLAGS_collect_Semaphore;
extern volatile uint8_t buzzer_flag;
//состояние коптера для конечного автомата
enum supervisorStates {
    STATE_INITIALIZING	= 0x00,
    STATE_CALIBRATION	= 0x01,
    STATE_DISARMED	= 0x02,
    STATE_ARMED		= 0x04,
    STATE_FLYING	= 0x08,
    STATE_RADIO_LOSS1	= 0x10,
    GPS_GLITCH	= 0x20,
    STATE_LOW_BATTERY	= 0x40,
    ERR_SENSOR	= 0x80
};
enum supervisorStates supervisorState;

//режим коптера для конечного автомата
enum Modes {
	MODE_STAB	= 0x00,
	MODE_ESC_CALIB	= 0x01,
	MODE_ALT_HOLD	= 0x02,
	MODE_HEADFREE	= 0x04,
	MODE_AUTO	= 0x03,
	MODE_POS_HOLD	= 0x10,
};
enum Modes Mode;

//состояние навигации
enum StateNavUKFs{
    UKF_NORM = 0x00,
    UKF_BAD_GPS = 0x01,
    UKF_GLITCH_GPS = 0x02,
    UKF_NAN = 0x04
};
enum StateNavUKFs StateNavUKF;


//кол-во свободных байтов(слов) стека
size_t ovf_full_stack;
volatile uint32_t main_stack;
//счетчик мс
volatile uint32_t counter_sys;
extern float axf, ayf, azf, gxf, gyf, gzf;//правильные данные с датчиков в g и рад/сек
extern float mxf, myf, mzf;//данные с компаса
//ошибки сенсоров
enum Err_Sensor {
    OK,
    Err_Init,
    Err_value,
    Err_synq
};


#endif
