#ifndef _GPS_H_
#define _GPS_H_

#include "Config.h"

//***************************************************************************************************//
#define UART_GPS						USART3
#define UART_GPS_IRQ					USART3_IRQn


#define UART_GPS_RCC_Periph_Pin()			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE)
#define UART_GPS_GPIO						GPIOD
#define UART_GPS_Pin_TX						GPIO_Pin_8
#define UART_GPS_Pin_Soucre_TX				GPIO_PinSource8
#define UART_GPS_Pin_RX						GPIO_Pin_9
#define UART_GPS_Pin_Soucre_RX				GPIO_PinSource9
#define UART_GPS_Pin_AF						GPIO_AF_USART3

#define UART_GPS_RCC_Periph()	 			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE)
//***************************************************************************************************//
#define NAV_EQUATORIAL_RADIUS	(6378.137 * 1000.0)			    // meters
#define NAV_FLATTENING		(1.0 / 298.257223563)			    // WGS-84
#define NAV_E_2			(NAV_FLATTENING * (2.0 - NAV_FLATTENING))

#define GPS_LATENCY             75000       // us (comment out to use uBlox timepulse)


//Экспорт
//Для персчета позиции и скорости, в случае положения ЖПС не в центре
#define GPS_FRAME_POS_X		(float)(-10.5f*0.01f)
#define GPS_FRAME_POS_Y		(float)(11.0f*0.01f)
#define GPS_FRAME_POS_Z		(float)(13.5f*0.01f)

//вкл компенсацию
#define USE_GPS_FRAME_POS_CMPNST
#define USE_GPS_FRAME_VEL_CMPNST

extern SemaphoreHandle_t xGPS_UKF_Mutex;

//Счетчики цикла
extern uint32_t cnt_cycle_gps, cnt_ticks_gps;
//****************************************************//
//Микросекунды получения данных, флаги появления данных
extern uint32_t GPS_Micros_Update;
extern uint8_t flag_start_home, flag_get_pvt;
//****************************************************//
//****************************************************//
//Вспомогательные данные
extern uint16_t GPS_year;
extern uint8_t GPS_month, GPS_day, GPS_hours, GPS_minutes, GPS_seconds;
extern uint8_t GPS_satellits;
extern uint8_t GPS_valid_invalid;
extern uint8_t GPS_fix_type;
//****************************************************//
//Данные не для расчетов, только для телемтрии
extern float GPS_LA, GPS_LO;
extern int32_t GPS_lon, GPS_lat;
extern uint32_t GPS_hAcc, GPS_vAcc, GPS_sAcc;
extern uint16_t GPS_vDop, GPS_nDop, GPS_eDop, GPS_tDop;
//****************************************************//


//*******************Главные данные для расчетов*******************************//
//Позиция в м, относительно старта
extern float GPS_X, GPS_Y;
extern float GPS_X_home, GPS_Y_home;
extern float GPS_alt;
//Скорости, курс
extern float GPS_Y_speed, GPS_X_speed, GPS_Z_speed;
extern float GPS_vel, GPS_course;
//Показатели качества
extern float GPS_hAccf, GPS_vAccf, GPS_sAccf;
extern float GPS_pDopf, GPS_eDopf, GPS_vDopf, GPS_nDopf, GPS_tDopf;
//****************************************************//


extern SemaphoreHandle_t xSD_GPS_collect_Semaphore;

void prvGPS_Processing(void *pvParameters);
void USART_INIT_GPS(void);

#endif
