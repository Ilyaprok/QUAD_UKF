#ifndef _ADC_H_
#define _ADC_H_


//***************************************************************************************************//

#define	ADC_CU		 					ADC1
#define	ADC_CU_MULTI_Pin1				GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3
//#define	ADC_CU_MULTI_Pin2				GPIO_Pin_4
#define	ADC_CU_RCC_PERIPH_DMA_GPIO()	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOA , ENABLE)
#define	ADC_CU_RCC_PERIPH_ADC()			RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE)
#define	ADC_CU_Port_Group1				GPIOC
//#define	ADC_CU_Port_Group2				GPIOA

#define	ADC_CU_DMA_CHANNEL				DMA_Channel_0
#define	ADC_CU_DMA_STREAM				DMA2_Stream4

#define	ADC_CU_DMA_IRQ	 				DMA2_Stream4_IRQn
//***************************************************************************************************//

extern SemaphoreHandle_t xSD_ADC_collect_Semaphore;
extern SemaphoreHandle_t xADC_Mutex;
//√лавные перменные
extern float All_voltage, All_power, All_action, All_voltage_smooth, All_Current, All_percent_cap, All_capacity_batt;


//—четчики цикла
extern uint32_t cnt_cycle_adc, cnt_ticks_adc;
//****************************************************//


void Init_SYS_ADC(void);
void prvOperation_with_ADC_results(void *pvParameters);
void prvADC_Request(void *pvParameters);


#endif
