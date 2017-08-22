#ifndef _UART_TELEMETRY_H_
#define _UART_TELEMETRY_H_

#include "Config.h"


//***************************************************************************************************//
#define UART_BLUETOOTH							USART2

#define UART_BLUETOOTH_IRQ						USART2_IRQn

#define UART_BLUETOOTH_RCC_PERIPH_Pin()			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE)
#define UART_BLUETOOTH_Multi_Pin				GPIO_Pin_5|GPIO_Pin_6
#define UART_BLUETOOTH_GPIO						GPIOD
#define UART_BLUETOOTH_Pin_Source_TX			GPIO_PinSource5
#define UART_BLUETOOTH_Pin_Source_RX			GPIO_PinSource6
#define UART_BLUETOOTH_GPIO_AF					GPIO_AF_USART2

#define UART_BLUETOOTH_RCC_PERIPH()				RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE)
//***************************************************************************************************//
//***************************************************************************************************//
#define UART_FRSKY								UART4

#define UART_FRSKY_IRQ							UART4_IRQn

#define UART_FRSKY_RCC_PERIPH_Pin_TX()			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)
#define UART_FRSKY_RCC_PERIPH_Pin_RX()			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)
#define UART_FRSKY_Pin_TX						GPIO_Pin_10
#define UART_FRSKY_Pin_RX						GPIO_Pin_11
#define UART_FRSKY_GPIO_TX						GPIOC
#define UART_FRSKY_GPIO_RX						GPIOC
#define UART_FRSKY_Pin_Source_TX				GPIO_PinSource10
#define UART_FRSKY_Pin_Source_RX				GPIO_PinSource11
#define UART_FRSKY_GPIO_AF						GPIO_AF_UART4

#define UART_FRSKY_RCC_PERIPH()					RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE)
//***************************************************************************************************//



#define header_value   0x5e
#define tail_value     0x5e
#define escape_value   0x5d
#define decimal        0x8

#define GPSALT         0x1
#define TEMP1          0x2
#define RPM            0x3
#define FUEL           0x4
#define TEMP2          0x5
#define INDVOLT        0x6
#define VCC            0x7
//#define unused       0x8
#define GPSALTd        0x9
/* Mavlink data start */
#define HOME_DIR       0xA
#define HOME_DIST      0xB
#define CPU_LOAD       0xC
#define GPS_HDOP       0xD
#define WP_NUM         0xE
#define WP_BEARING     0xF
/* Mavlink data end */
#define ALTITUDE       0x10
#define GPSSPEED       0x11
#define LONGITUDE      0x12
#define LATITUDE       0x13
#define COURSE         0x14
#define DATE           0x15
#define YEAR           0x16
#define TIME           0x17
#define SECOND         0x18
//#define unused       0x19
//#define unused       0x1A
//#define unused       0x1B
//#define unused       0x1C
/* Mavlink data start */
#define BASEMODE       0x1D
#define WP_DIST        0x1E
#define HEALTH         0x1F
#define STATUS_MSG     0x20
/* Mavlink data end */
#define ALTIDEC        0x21
#define EASTWEST       0x22
#define NORTHSOUTH     0x23
#define ACCX           0x24
#define ACCY           0x25
#define ACCZ           0x26
#define VerticalSpeed  0x27
#define CURRENT        0x28
//#define unused       0x29
//#define unused       0x2A
//#define FR_VSPD        0x30 // 0x30 -> 0x27
#define VOLTAGE        0x3A
#define VOLTAGEDEC     0x3B

// MAVLink HeartBeat bits
#define MOTORS_ARMED 7  // 128
#define MSG_TIMER 30    // 30 sec
//***************************************************************************************************//

extern SemaphoreHandle_t xParcel_TX_Semaphore;

//***************************************************************************************************//

//extern volatile float throtle_ppm;
//
//extern float KP_ugla_xy, KI_ugla_xy, KD_ugla_xy;
//extern float KP_rate_xy, KI_rate_xy, KD_rate_xy;
//
//extern float KP_ugla_z, KI_ugla_z, KD_ugla_z;
//extern float KP_rate_z, KI_rate_z, KD_rate_z;
//
//extern float f_cut_diff_pid;
//extern float f_cut_diff_vel_alt_z, f_cut_err_vel_alt_z;
//
//extern float KP_Pos_Alt_Z;
//extern float KP_Vel_Alt_Z, KD_Vel_Alt_Z;
//extern float KP_Acc_Alt_Z, KI_Vel_Alt_Z, KD_Acc_Alt_Z;
//
////X Y
//extern float KP_Pos_XY;
//extern float KP_Vel_XY, KI_Vel_XY, KD_Vel_XY;
//extern float f_cut_err_vel_xy;
//extern float f_cut_diff_vel_xy;


//extern float q0_mag, q1_mag, q2_mag, q3_mag;
//extern float yaw_start;
//extern float axisx, axisy, axisz;
//extern float q0_z_ref, q3_z_ref;
//extern float x_ref, y_ref;
//extern float ref_alt_hold;
//extern float All_voltage, All_power, All_action, All_voltage_smooth, All_Current, All_percent_cap;

//extern float GPS_X, GPS_Y, pos_z, pos_x, pos_y;
//extern float GPS_alt;
//extern float GPS_vel, GPS_course;
//extern float GPS_LA, GPS_LO;
//extern float GPS_pDopf;
//extern uint8_t flag_start_home;
//extern uint8_t GPS_satellits;
//extern uint8_t GPS_fix_type;

//extern uint8_t compass_not_health, baro_not_health;




void USART_INIT_BLuetooth(void);
void USART_INIT_Frsky(void);


void prvParcel_to_Comp(void *pvParameters);



#endif
