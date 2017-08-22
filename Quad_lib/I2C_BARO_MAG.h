
#ifndef _I2C_BARO_MAG_H_
#define _I2C_BARO_MAG_H_

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "Config.h"

#define HMC5883_DEFAULT_ADDRESS     0x3C
#define MS5611_DEFAULT_ADDRESS      0xEE

//***************************************************************************************************//
#define I2C_BARO_MAG					I2C2
#define I2C_BARO_MAG_IRQ_EV				I2C2_EV_IRQn
#define I2C_BARO_MAG_IRQ_ER				I2C2_ER_IRQn

#define I2C_RCC_Periph_Pin()			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)
#define I2C_GPIO						GPIOB
#define I2C_Pin_SCL						GPIO_Pin_10
#define I2C_Pin_Soucre_SCL				GPIO_PinSource10
#define I2C_Pin_SDA						GPIO_Pin_11
#define I2C_Pin_Soucre_SDA				GPIO_PinSource11
#define I2C_Pin_AF						GPIO_AF_I2C2

#define I2C_RCC_Periph()	 			RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE)
//***************************************************************************************************//

extern SemaphoreHandle_t xSD_FLAGS_collect_Semaphore;
extern SemaphoreHandle_t xBAROMAG_UKF_Mutex;
//экспорт
extern float mxf, myf, mzf;
extern float Temperature, Altitude, Pressure;

extern uint8_t flag_start_home_baro;
extern uint16_t err_num_i2c_def, err_num_i2c_line;
extern uint8_t compass_not_health, baro_not_health;

extern uint32_t cnt_ticks_baro, cnt_cycle_baro;
extern uint32_t cnt_cycle_mag, cnt_ticks_mag;

extern enum Err_Sensor Err_Mag, Err_Baro;

extern int16_t mx, my, mz;

/////
extern SemaphoreHandle_t xSD_MAG_BARO_collect_Semaphore;

void Baro_Mag_Init_Full(void);





#endif
