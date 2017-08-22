/* библиотека для получения данных с MPU6500 - акселерометр, гироскоп */

#ifndef _MPU6500_H_
#define _MPU6500_H_

#include "stm32f4xx.h"
#include "Config.h"




//***************************************************************************************************//
#define	SPI_MPU_IRQ	 					SPI1_IRQn
#define	SPI_MPU		 					SPI1
#define SPI_MPU_IRQHandler				SPI1_IRQHandler

#define	SPI_MPU_Port_Group				GPIOA
#define	SPI_MPU_RCC_PERIPH_PIN()		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
#define	SPI_MPU_MULTI_Pin				GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7
#define	SPI_MPU_Pin_Source_MISO			GPIO_PinSource6
#define	SPI_MPU_Pin_Source_SCK			GPIO_PinSource5
#define	SPI_MPU_Pin_Source_MOSI			GPIO_PinSource7
#define	SPI_MPU_GPIO_AF					GPIO_AF_SPI1

#define	SPI_MPU_Pin_CS					GPIO_Pin_4
#define	SPI_MPU_RCC_PERIPH_CS()			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)
#define	SPI_MPU_Port_Group_CS			GPIOA

#define	SPI_MPU_RCC_PERIPH()			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE)


#define	EXTI_MPU_IRQ	 				EXTI0_IRQn
#define EXTI_IRQHandler					EXTI0_IRQHandler
#define EXTI_Line_MPU					EXTI_Line0
#define	EXTI_SPI_MPU_RCC_PERIPH_Pin()	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)
#define	EXTI_SPI_MPU_Pin				GPIO_Pin_0
#define	EXTI_SPI_MPU_Port_Group			GPIOB
#define	EXTI_SPI_MPU_RCC_PERIPH()		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE)
#define	EXTI_SPI_MPU_Pin_Source_Group	EXTI_PortSourceGPIOB
#define	EXTI_SPI_MPU_Pin_Source			EXTI_PinSource0
//***************************************************************************************************//



#define CS_LOW 							GPIO_ResetBits(SPI_MPU_Port_Group_CS, SPI_MPU_Pin_CS)
#define CS_HIGH 						GPIO_SetBits(SPI_MPU_Port_Group_CS, SPI_MPU_Pin_CS)




#define DLPF_CFG_GYRO_250Hz 		0	//delay = 0.97 ms
#define DLPF_CFG_GYRO_184Hz 		1	//delay = 2.9 ms
#define DLPF_CFG_GYRO_92Hz 		2	//delay = 3.9 ms
#define DLPF_CFG_GYRO_41Hz 		3	//delay = 5.9 ms
#define DLPF_CFG_GYRO_20Hz 		4	//delay = 9.9 ms
#define DLPF_CFG_GYRO_10Hz 		5	//delay = 17.85 ms
#define DLPF_CFG_GYRO_5Hz 		6	//delay = 33.48 ms
#define DLPF_CFG_GYRO_3600Hz 		7	//delay = 0.17 ms

#define F_CHOICE_B_GYRO_FAST 		1	//8800Hz
#define F_CHOICE_B_GYRO_LOW 		2	//3600Hz
#define F_CHOICE_B_GYRO_DLPF 		0	//DLPF

#define GYRO_FS_SEL_250	 		0
#define GYRO_FS_SEL_500	 		8
#define GYRO_FS_SEL_1000	 	16
#define GYRO_FS_SEL_2000	 	24

#define ACCEL_FS_SEL_2	 		0
#define ACCEL_FS_SEL_4	 		8
#define ACCEL_FS_SEL_8		 	16
#define ACCEL_FS_SEL_16		 	24

#define ACCEL_F_CHOICE_B_USE_DLPF 	0	//DLPF
#define ACCEL_F_CHOICE_B_NOT_USE_DLPF 	8	//1130Hz

#define ACCEL_CFG_GYRO_460Hz 		0	//delay = 1.94 ms
#define ACCEL_CFG_GYRO_184Hz 		1	//delay = 5.8 ms
#define ACCEL_CFG_GYRO_92Hz 		2	//delay = 7.8 ms
#define ACCEL_CFG_GYRO_41Hz 		3	//delay = 11.8 ms
#define ACCEL_CFG_GYRO_20Hz 		4	//delay = 19.8 ms
#define ACCEL_CFG_GYRO_10Hz 		5	//delay = 35.7 ms
#define ACCEL_CFG_GYRO_5Hz 		6	//delay = 66.96 ms
//#define ACCEL_CFG_GYRO_460Hz 		7	//delay = 1.94 ms

#define RAW_RDY_EN	 		1

#define SMPLRT_DIV_REG	 		25
#define CONFIG_REG 			26
#define GYRO_CONFIG_REG 		27
#define ACCEL_CONFIG_REG 		28
#define ACCEL_CONFIG_REG2 		29

#define INT_ENABLE_REG	 		56

#define ACCEL_XOUT_H_REG 		59
#define ACCEL_XOUT_L_REG 		60
#define ACCEL_YOUT_H_REG 		61
#define ACCEL_YOUT_L_REG 		62
#define ACCEL_ZOUT_H_REG 		63
#define ACCEL_ZOUT_L_REG 		64

#define TEMP_OUT_H_REG 			65
#define TEMP_OUT_L_REG 			66

#define GYRO_XOUT_H_REG 		67
#define GYRO_XOUT_L_REG 		68
#define GYRO_YOUT_H_REG 		69
#define GYRO_YOUT_L_REG 		70
#define GYRO_ZOUT_H_REG 		71
#define GYRO_ZOUT_L_REG 		72


//Импорт
extern void prvIMU_INS(void *pvParameters);
extern void prvIMU_INS_UKF(void *pvParameters);
extern SemaphoreHandle_t xUKF_Semaphore;
extern SemaphoreHandle_t  xMPU_UKF_Mutex;
extern TaskHandle_t ukf_handle;
//Экспрот
 /*FreeRTOS*/
extern SemaphoreHandle_t xMPU_Semaphore;
extern TaskHandle_t ins_handle;
/*FreeRTOS*/
//буффер данных с датчика
extern uint8_t MPU_buff[14];

extern enum Err_Sensor Err_MPU;


//функция настройки и инициализации датчика
void MPU6500_Init_Full(void);

#endif /* _MPU6500_H_ */
