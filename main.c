#include "stm32f4xx.h"
#include "Config.h"
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"






#include "MPU6500.h"
#include "IMU_INS.h"
#include "I2C_BARO_MAG.h"
#include "USART_TELEMETRY.h"
#include "GPS.h"
#include "ADC.h"
#include "PWM_PPM.h"
#include "Buzzer.h"

#include "LOGS.h"
#include "ff.h"
#include "spi_sd.h"

#define LED1 GPIOE, GPIO_Pin_0
#define LED2 GPIOE, GPIO_Pin_1
#define LED3 GPIOE, GPIO_Pin_2
#define LED4 GPIOE, GPIO_Pin_3

unsigned long dwt_cnt;
UINT nWritten_def_han;
BYTE File_Name_def_han[] = "Error.log";
FIL file_def_han;

//char str_task[600]={0};
//TaskStatus_t pxTaskStatusArray[21];
//volatile UBaseType_t uxArraySize, x;
//uint32_t ulTotalTime, ulStatsAsPercentage;
//
//TaskStatus_t task1, task2, task3, task4, task5, task6, task7, task8, task9, task10, task11, task12, task13, task14, task15, task16, task17, task18, task19, task20, task21;
//uint8_t text[15];
//volatile uint32_t percenttasks[21];

void reset_all_motors(void)
{
	TIM_SetCompare1(PWM_M, 28550);
	TIM_SetCompare2(PWM_M, 28550);
	TIM_SetCompare3(PWM_M, 28550);
	TIM_SetCompare4(PWM_M, 28550);

	f_open(&file_def_han, &File_Name_def_han, FA_CREATE_ALWAYS | FA_WRITE);
	f_write(&file_def_han, "Enter to Default Handler",24, &nWritten_def_han);
	f_sync(&file_def_han);
}
void prvLed_GPS( void *pvParameters )
{
	while(1)
	    {
		if (GPS_hAccf < NAV_MIN_GPS_ACC)
		    {
			GPIO_SetBits(LED1);
			vTaskDelay(600);
			GPIO_ResetBits(LED1);
			vTaskDelay(200);
		    }
		else
		{
			GPIO_SetBits(LED1);
			vTaskDelay(100);
		}

	    }
}
void prvLed_Arming( void *pvParameters )
{
	while(1)
	    {
		if (supervisorState&STATE_ARMED)
		    {
			GPIO_SetBits(LED2);
			vTaskDelay(900);
			GPIO_ResetBits(LED2);
			vTaskDelay(100);
		    }
		else
		    {
			GPIO_SetBits(LED2);
			vTaskDelay(400);
		    }

	    }
}
void prvLeds( void *pvParameters )
    {
	while(1)
	    {
		if (buzzer_flag == 0)
		    {
			if(flag_start_state_init == 3)
			    {

			    	GPIO_ResetBits(LED3);
			    	GPIO_SetBits(LED4);
				vTaskDelay(100);
			    	GPIO_ResetBits(LED4);
			    	GPIO_SetBits(LED3);
				vTaskDelay(100);
			    }
			else
			    {
			    	GPIO_SetBits(LED3);
			    	GPIO_ResetBits(LED4);
				vTaskDelay(30);
			    	GPIO_SetBits(LED4);
			    	GPIO_ResetBits(LED3);
				vTaskDelay(30);
			    }
		    }
		else
		    {
			GPIO_ResetBits(LED3);
		    	vTaskDelay(80);
			GPIO_SetBits(GPIOD,  GPIO_Pin_14 |GPIO_Pin_15 );
			vTaskDelay(80);
			GPIO_ResetBits(LED3);
		    	vTaskDelay(80);
			GPIO_SetBits(GPIOD,  GPIO_Pin_14 |GPIO_Pin_15 );
			vTaskDelay(80);
		    }


		//vTaskGetRunTimeStats(str_task);

//		uxArraySize = uxTaskGetNumberOfTasks();
//		//pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );
//		uxArraySize = uxTaskGetSystemState(pxTaskStatusArray,uxArraySize,&ulTotalTime);
//		uint8_t i;
//		ulTotalTime /=100;
//		for (i = 0; i< uxArraySize;i++)
//		    {
//			memcpy(text,&pxTaskStatusArray[i].pcTaskName, 15);
//			percenttasks[i] = pxTaskStatusArray[i].ulRunTimeCounter/ulTotalTime;
//		    }
//
//		memcpy(&task1,&pxTaskStatusArray[0],sizeof( TaskStatus_t ));
//		memcpy(&task2,&pxTaskStatusArray[1],sizeof( TaskStatus_t ));
//		memcpy(&task3,&pxTaskStatusArray[2],sizeof( TaskStatus_t ));
//		memcpy(&task4,&pxTaskStatusArray[3],sizeof( TaskStatus_t ));
//		memcpy(&task5,&pxTaskStatusArray[4],sizeof( TaskStatus_t ));
//		memcpy(&task6,&pxTaskStatusArray[5],sizeof( TaskStatus_t ));
//		memcpy(&task7,&pxTaskStatusArray[6],sizeof( TaskStatus_t ));
//		memcpy(&task8,&pxTaskStatusArray[7],sizeof( TaskStatus_t ));
//		memcpy(&task9,&pxTaskStatusArray[8],sizeof( TaskStatus_t ));
//		memcpy(&task10,&pxTaskStatusArray[9],sizeof( TaskStatus_t ));
//		memcpy(&task11,&pxTaskStatusArray[10],sizeof( TaskStatus_t ));
//		memcpy(&task12,&pxTaskStatusArray[11],sizeof( TaskStatus_t ));
//		memcpy(&task13,&pxTaskStatusArray[12],sizeof( TaskStatus_t ));
//		memcpy(&task14,&pxTaskStatusArray[13],sizeof( TaskStatus_t ));
//		memcpy(&task15,&pxTaskStatusArray[14],sizeof( TaskStatus_t ));
//		memcpy(&task16,&pxTaskStatusArray[15],sizeof( TaskStatus_t ));
//		memcpy(&task17,&pxTaskStatusArray[16],sizeof( TaskStatus_t ));
//		memcpy(&task18,&pxTaskStatusArray[17],sizeof( TaskStatus_t ));
//		memcpy(&task19,&pxTaskStatusArray[18],sizeof( TaskStatus_t ));
//		memcpy(&task20,&pxTaskStatusArray[19],sizeof( TaskStatus_t ));
//		memcpy(&task21,&pxTaskStatusArray[20],sizeof( TaskStatus_t ));
	    }
    }
void prvLed( void *pvParameters )
{

	while(1)
	    {
		if (Err_Mag||Err_Baro||Err_MPU)
		    {
			GPIO_ResetBits(GPIOD, GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15);
			GPIO_ToggleBits(GPIOD,GPIO_Pin_12);
			vTaskDelay(100);
		    }
		else
		{
			GPIO_ResetBits(GPIOD,GPIO_Pin_12);
			GPIO_SetBits(GPIOD,GPIO_Pin_13);
			vTaskDelay(100);
			GPIO_ResetBits(GPIOD,GPIO_Pin_13);
			GPIO_SetBits(GPIOD,GPIO_Pin_14);
			vTaskDelay(100);
			GPIO_ResetBits(GPIOD,GPIO_Pin_14);
			GPIO_SetBits(GPIOD,GPIO_Pin_15);
			vTaskDelay(100);
			GPIO_ResetBits(GPIOD,GPIO_Pin_15);
			GPIO_SetBits(GPIOD,GPIO_Pin_12);
			vTaskDelay(100);
		}



	    }
}

void timerInit(void) {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;
//
//    // Enable the TIMER_TIM global Interrupt
//    NVIC_InitStructure.NVIC_IRQChannel = TIMER_IRQ_CH;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	// stop timer when core halted (debug)
	DBGMCU_APB1PeriphConfig(((uint32_t)0x00000008), ENABLE);

	uint32_t pclk1;
	RCC_ClocksTypeDef rcc_clocks;
	  RCC_GetClocksFreq(&rcc_clocks);
	  pclk1 = rcc_clocks.PCLK1_Frequency;

    /* Time base configuration for 1MHz (us)*/
    TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = ((pclk1 * 2) / 1000000) - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5	, &TIM_TimeBaseStructure);

    // reset
    TIM_SetCounter(TIM5	, 0);

//    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
//    NVIC_EnableIRQ(TIM5_IRQn);

//    timerCancelAlarm1();
//    timerCancelAlarm2();
//    timerCancelAlarm3();
//    timerCancelAlarm4();

//    // Output Compare for alarms
//    TIM_OCStructInit(&TIM_OCInitStructure);
//    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;
//    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//
//    TIM_OC1Init(TIMER_TIM, &TIM_OCInitStructure);
//    TIM_OC1PreloadConfig(TIMER_TIM, TIM_OCPreload_Disable);
//
//    TIM_OC2Init(TIMER_TIM, &TIM_OCInitStructure);
//    TIM_OC2PreloadConfig(TIMER_TIM, TIM_OCPreload_Disable);
//
//    TIM_OC3Init(TIMER_TIM, &TIM_OCInitStructure);
//    TIM_OC3PreloadConfig(TIMER_TIM, TIM_OCPreload_Disable);
//
//    TIM_OC4Init(TIMER_TIM, &TIM_OCInitStructure);
//    TIM_OC4PreloadConfig(TIMER_TIM, TIM_OCPreload_Disable);

    // go...
    TIM_Cmd(TIM5, ENABLE);
}


void GPIO_setup_LEDS(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3; // we want to configure all LED GPIO pins
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	GPIO_SetBits(GPIOE, GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_2 |GPIO_Pin_3 );
}
int main(void)
{
	uint32_t pause_init;

	SystemInit();
/******************************************/
	for (pause_init = 0; pause_init < 33600000; pause_init++); pause_init = 0;

	xUKF_PID_Mutex = xSemaphoreCreateMutex();
	xGPS_UKF_Mutex = xSemaphoreCreateMutex();
	xMPU_UKF_Mutex = xSemaphoreCreateMutex();
	xBAROMAG_UKF_Mutex = xSemaphoreCreateMutex();
	xADC_Mutex = xSemaphoreCreateMutex();
	StateNavUKF = 0;
/******************************************/
	USART_INIT_GPS();
/******************************************/
	USART_INIT_BLuetooth();
	USART_INIT_Frsky();
/******************************************/
	Baro_Mag_Init_Full();
/******************************************/
	Init_PWM_Reciever1(83);
/******************************************/
	Init_PWM_Motor(57100,40000);
/******************************************/
	Init_SYS_ADC();
/******************************************/
	Init_logs();
/******************************************/
	MPU6500_Init_Full();
/******************************************/
	xTaskCreate(prvArming,(signed char*)"Arming", 100, NULL, PRIORITET_TASK_ARMING, ( xTaskHandle * ) NULL);
/******************************************/
	Buzzer_Init();
/******************************************/
	GPIO_setup_LEDS();
	xTaskCreate(prvLed_GPS,(signed char*)"LED_GPS",70, NULL, PRIORITET_TASK_LEDS, ( xTaskHandle * ) NULL);
	xTaskCreate(prvLed_Arming,(signed char*)"LED_Arm",70, NULL, PRIORITET_TASK_LEDS, ( xTaskHandle * ) NULL);

	xTaskCreate(prvLeds,(signed char*)"LEDS",400, NULL, PRIORITET_TASK_LEDS, ( xTaskHandle * ) NULL);
/******************************************/
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Разрешаем TRACE
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Разрешаем счетчик тактов
	DWT->CYCCNT = 0; // Обнуляем счетчик
/******************************************/
	timerInit();

	/* Start the scheduler. */
	vTaskStartScheduler();

    while(1)
    {
    }
}
