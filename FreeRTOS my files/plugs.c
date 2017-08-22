#include "plugs.h"

#include "Config.h"
#include "stm32f4xx_conf.h"

#include "MPU6500.h"
#include "stm32f4xx_adc.h"
#include "ADC.h"
#include "IMU_INS.h"

#include "Buzzer.h"


//uint32_t k_delay;
uint8_t flag_ovf_os_stack = 0;
/*******************************************************************/
void vApplicationIdleHook( void )
{

}



/*******************************************************************/
void vApplicationMallocFailedHook( void )
{
    for( ;; );
}



/*******************************************************************/
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;
    flag_ovf_os_stack = 1;
    for( ;; );
}



/*******************************************************************/
void vApplicationTickHook( void )
{
	static uint8_t cnt_sd;
	static portBASE_TYPE xHigherPriorityTaskWoken_UKF = pdFALSE;
	cnt_sd++;
	counter_sys++;
	if (!((counter_sys+1)%5))
	    {
		xSemaphoreGiveFromISR(xUKF_Semaphore, &xHigherPriorityTaskWoken_UKF);
		//xSemaphoreGive(xUKF_Semaphore);
	    }
	if (cnt_sd >=10)
	    {
		counter_buzzer_sys++;
		cnt_sd = 0;
		//disk_timerproc();
		if (buzzer_flag !=0) Buzzer_Function();
		ADC_SoftwareStartConv(ADC_CU);
	    }
}



/*******************************************************************/
