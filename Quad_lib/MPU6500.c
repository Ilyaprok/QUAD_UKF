
#include "stm32f4xx.h"

#include "stm32f4xx_syscfg.h"


#include "Config.h"
#include "MPU6500.h"
#include "UKF_lib.h"


//Вспомогательные счетчики для отладки
uint8_t spi_counter_byte;//кол-во переданных байт в пакете
uint32_t counter_work_int_exti, counter_work_int_spi;
//****************************************************//

//ошибка датчика
enum Err_Sensor Err_MPU;
//****************************************************//

//семафор для связи с модулем орбаботки
SemaphoreHandle_t xMPU_Semaphore = NULL;

TaskHandle_t ins_handle;
//****************************************************//

//конечный буфер данных
uint8_t MPU_buff[14];
//****************************************************//

void EXTI0_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line_MPU);
	//данные готовы, теперь надо их забрать, отправляем запрос
    CS_LOW;
    SPI_I2S_ITConfig(SPI_MPU, SPI_IT_RXNE, ENABLE);
    SPI_I2S_SendData(SPI_MPU, ACCEL_XOUT_H_REG|0x80);
    spi_counter_byte = 255;
    counter_work_int_exti++;
}
void SPI1_IRQHandler(void)
    {
	/*FreeRTOS*/
	//аргумент для семафора для принудительной передачи времени задаче обработки
	static portBASE_TYPE xHigherPriorityTaskWoken_MPU6500 = pdFALSE;
	/*FreeRTOS*/

	if (SPI_I2S_GetITStatus(SPI_MPU, SPI_IT_RXNE))
		    {
			if (spi_counter_byte == 13)
			    {
				SPI_I2S_ITConfig(SPI_MPU, SPI_IT_RXNE, DISABLE);
				MPU_buff[spi_counter_byte] = SPI_I2S_ReceiveData(SPI_MPU);
				spi_counter_byte = 255;
				CS_HIGH;

				/*FreeRTOS*/
				 // выставляем семафор задаче на обработку данных

					xSemaphoreGiveFromISR(xMPU_Semaphore, &xHigherPriorityTaskWoken_MPU6500);
			        //принудительное переключение контекста для уменьшения времени реакции на прерывание
			        if( xHigherPriorityTaskWoken_MPU6500 != pdFALSE )
			        {
			        	taskYIELD();
				        counter_work_int_spi++;
			        }
			        /*FreeRTOS*/

				return;
			    }
			if (spi_counter_byte != 255)
			    MPU_buff[spi_counter_byte] = SPI_I2S_ReceiveData(SPI_MPU);
			else
			    SPI_I2S_ReceiveData(SPI_MPU);
			spi_counter_byte++;
			SPI_I2S_SendData(SPI_MPU, 0x00);
		    }
    }

void SPI_Init_MPU(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;

	/* подадим такт на порт */
	SPI_MPU_RCC_PERIPH_CS();
	/* настройка выводов выбора кристалла CS ИНС */
	GPIO_InitStruct.GPIO_Pin 	= SPI_MPU_Pin_CS;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_Init(SPI_MPU_Port_Group_CS, &GPIO_InitStruct);
	CS_HIGH;
	SPI_MPU_RCC_PERIPH_PIN();
	/* настройка выводов SPI1 */
	GPIO_InitStruct.GPIO_Pin 	= SPI_MPU_MULTI_Pin;
	GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_Init(SPI_MPU_Port_Group, &GPIO_InitStruct);
	/* Соединим выводы портов с GPIO_AF_SPI1 */
	GPIO_PinAFConfig(SPI_MPU_Port_Group, SPI_MPU_Pin_Source_SCK, SPI_MPU_GPIO_AF);
	GPIO_PinAFConfig(SPI_MPU_Port_Group, SPI_MPU_Pin_Source_MISO, SPI_MPU_GPIO_AF);
	GPIO_PinAFConfig(SPI_MPU_Port_Group, SPI_MPU_Pin_Source_MOSI, SPI_MPU_GPIO_AF);
	/* подадим такт на SPI */
	SPI_MPU_RCC_PERIPH();
	/* настроим SPI Master */
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_Init(SPI_MPU, &SPI_InitStruct);

	SPI_Cmd(SPI_MPU,ENABLE);

}


void EXTI_Init_MPU6500(void)
    {
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStruct;

	/* подадим такт на порт */
	EXTI_SPI_MPU_RCC_PERIPH_Pin();
	/* настройка выводов выбора кристалла CS ИНС */
	GPIO_InitStruct.GPIO_Pin 	= EXTI_SPI_MPU_Pin;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_IN;
	GPIO_Init(EXTI_SPI_MPU_Port_Group, &GPIO_InitStruct);


	/*----------- настройка запроса МПУ ----------------*/
	/* тактируем SYSCFG  */
	EXTI_SPI_MPU_RCC_PERIPH();
	/* конфигурация подключения к портам */
	SYSCFG_EXTILineConfig(EXTI_SPI_MPU_Pin_Source_Group,EXTI_SPI_MPU_Pin_Source);
	/* запрос прерывания по готовности данных ДУС*/
	EXTI_ClearITPendingBit(EXTI_Line_MPU);
	EXTI_InitStructure.EXTI_Line = EXTI_Line_MPU;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_SetPriority(EXTI_MPU_IRQ, PRIORITET_SPI_INS);
	NVIC_EnableIRQ(EXTI_MPU_IRQ);


    }

uint8_t spi_transfer(uint8_t out)
{
/* ждем пока последние данные уйдут */
    while(!(SPI_MPU->SR & SPI_I2S_FLAG_TXE));

/* шлем новые данные */
    SPI_MPU->DR =  out;
/* ждем ответ */
    while(!(SPI_MPU->SR & SPI_I2S_FLAG_RXNE));
/* возвращаем ответ */
    return (SPI_MPU->DR);
}
uint8_t spi_recieve(void)
{
    return spi_transfer(0x00);
}
uint8_t SPI_read_register( uint8_t reg1 )
{
	uint8_t res;
	CS_LOW;
	spi_transfer(reg1|0x80);
	res = spi_recieve();
	CS_HIGH;
	return res;
}
void SPI_write_register( uint8_t reg1, uint8_t value1 )
{
	CS_LOW;
	spi_transfer(reg1);
	spi_transfer(value1);
	CS_HIGH;
}
void SPI_write_bits( uint8_t reg, uint8_t mask )
{
	uint8_t registr, value;
	uint16_t pause_init;
	for (pause_init = 0; pause_init < 800; pause_init++); pause_init = 0;
	CS_LOW;
	registr = SPI_read_register(reg);
	value = registr|mask;
	for (pause_init = 0; pause_init < 800; pause_init++); pause_init = 0;
	SPI_write_register(reg, value);
	CS_HIGH;
	for (pause_init = 0; pause_init < 800; pause_init++); pause_init = 0;
}
uint16_t mpuid=0;
void MPU_Init(void)
    {

	//конфигурируем датчик на диапозон, фильтры, и тд.
	uint16_t pause_init;
	mpuid = SPI_read_register(0x75);

	SPI_write_register(0x6B, 0b10111111&SPI_read_register(0x6B));

	SPI_write_bits(SMPLRT_DIV_REG, 1);
	if (SPI_read_register(SMPLRT_DIV_REG) != 1) Err_MPU = Err_Init;

	SPI_write_bits(CONFIG_REG, DLPF_CFG_GYRO_41Hz);
	if (SPI_read_register(CONFIG_REG) != DLPF_CFG_GYRO_41Hz) Err_MPU = Err_Init;

	SPI_write_bits(GYRO_CONFIG_REG, GYRO_FS_SEL_1000|F_CHOICE_B_GYRO_DLPF);
	if (SPI_read_register(GYRO_CONFIG_REG) != (GYRO_FS_SEL_1000|F_CHOICE_B_GYRO_DLPF)) Err_MPU = Err_Init;

	SPI_write_bits(ACCEL_CONFIG_REG, ACCEL_FS_SEL_8);
	if (SPI_read_register(ACCEL_CONFIG_REG) != ACCEL_FS_SEL_8) Err_MPU = Err_Init;

	SPI_write_bits(INT_ENABLE_REG, RAW_RDY_EN);
	if (SPI_read_register(INT_ENABLE_REG) != RAW_RDY_EN) Err_MPU = Err_Init;

	//Для MPU6500
	//SPI_write_bits(ACCEL_CONFIG_REG2, ACCEL_F_CHOICE_B_USE_DLPF|ACCEL_CFG_GYRO_20Hz);
	//if (SPI_read_register(ACCEL_CONFIG_REG2) != (ACCEL_F_CHOICE_B_USE_DLPF|ACCEL_CFG_GYRO_20Hz)) Err_MPU = Err_Init;

	if(Err_MPU)
	    {
		supervisorState |= ERR_SENSOR;
		buzzer_flag = BUZZER_FLAG_FAIL_SENSOR;
	    }

	for (pause_init = 0; pause_init < 800; pause_init++); pause_init = 0;
    }


void MPU6500_Init_Full(void)
    {
	/*FreeRTOS*/
	//создание задачи обработки данных с акселерометра и гироскопа
	xTaskCreate(prvIMU_INS,(signed char*)"IMU_INS", 600, NULL, PRIORITET_TASK_INS, ins_handle);
	//двоичный семафор для передачи данных от прерывания задаче обработки этих данных
	vSemaphoreCreateBinary(xMPU_Semaphore);

	////
	xTaskCreate(prvIMU_INS_UKF,(signed char*)"UKF", 400, NULL, PRIORITET_TASK_UKF, ukf_handle);
	//двоичный семафор для передачи данных от прерывания задаче обработки этих данных
	vSemaphoreCreateBinary(xUKF_Semaphore);
	/*FreeRTOS*/

	navUkfInit();
	altUkfInit();


	uint32_t pause_init;
	SPI_Init_MPU();
	for (pause_init = 0; pause_init < 33600; pause_init++); pause_init = 0;
	MPU_Init();
	NVIC_SetPriority(SPI_MPU_IRQ, PRIORITET_SPI_INS);
	NVIC_EnableIRQ(SPI_MPU_IRQ);
	EXTI_Init_MPU6500();

    }
