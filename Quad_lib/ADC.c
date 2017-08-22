#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

#include "ADC.h"

#include "Config.h"

//������������ �������������� ���
#define K_ADC_VOLTAGE 					0.000712890625f//ADC to Volt

#define K_VOLTAGE_TO_ALL_CURRENT		2.0f*54.0f//VOLTAGE to ALL CURRENT


#define K_PRESCALER_VOLTAGE 			6.0204f//PRESCALER ALL VOLTAGE

#define K_ADC_ALL_CURRENT 				K_VOLTAGE_TO_ALL_CURRENT*K_ADC_VOLTAGE


#define K_ADC_ALL_VOLTAGE 				K_PRESCALER_VOLTAGE*K_ADC_VOLTAGE
//***************************************************************************************************//

#define CAPACITY_BATT	 				4500.0f
//***************************************************************************************************//
//������������ ����������������� �����������
#define DT_ADC							1.0f/100.0f
#define F_CUT_DIFF_ADC_CURRENT			4.61f
#define K_EXP_DIFF_ADC_CURRENT			(float)(1.0f - 2.0f*M_PI*F_CUT_DIFF_ADC_CURRENT*DT_ADC/(1.0f+2.0f*M_PI*F_CUT_DIFF_ADC_CURRENT*DT_ADC))

#define F_CUT_DIFF_ADC_VOLTAGE			20.61f
#define K_EXP_DIFF_ADC_VOLTAGE			(float)(1.0f - 2.0f*M_PI*F_CUT_DIFF_ADC_VOLTAGE*DT_ADC/(1.0f+2.0f*M_PI*F_CUT_DIFF_ADC_VOLTAGE*DT_ADC))

#define F_CUT_DIFF_ADC_VOLTAGE_SM		0.08f
#define K_EXP_DIFF_ADC_VOLTAGE_SM		(float)(1.0f - 2.0f*M_PI*F_CUT_DIFF_ADC_VOLTAGE_SM*DT_ADC/(1.0f+2.0f*M_PI*F_CUT_DIFF_ADC_VOLTAGE_SM*DT_ADC))
//***************************************************************************************************//

//��������������� ������������� ����������
uint16_t sys_adc_buf[6]; /* ����� ���������� ��� */
//****************************************************//
//��� ��
SemaphoreHandle_t xADC_Semaphore = NULL;
SemaphoreHandle_t xADC_Mutex;
TaskHandle_t adc_handle;
uint32_t ovf_adc_stack;
//****************************************************//
//�������� �����
uint32_t cnt_cycle_adc, cnt_ticks_adc;
//****************************************************//


//������� ���������
float All_voltage, All_power, All_action, All_voltage_smooth, All_Current, All_percent_cap, All_capacity_batt;
//****************************************************//

void Init_SYS_ADC(void)
{

	//�������� ��������� �������� ��� �������� ��������� ������
	vSemaphoreCreateBinary(xADC_Semaphore);
	//������ ��������� ������ ���
	xTaskCreate(prvOperation_with_ADC_results,(signed char*)"ADC_Processing", 150, NULL, PRIORITET_TASK_OPERATION_ADC, adc_handle);
	//����������� ���������� ��������� ��������� �������
	All_voltage_smooth = 16.8f;
	GPIO_InitTypeDef     		GPIO_InitStructure, GPIO_InitStructure2;
	ADC_InitTypeDef 			ADC_InitStruct;
	ADC_CommonInitTypeDef 		ADC_CommonInitStruct;
	DMA_InitTypeDef				DMA_InitStruct;

	/* ������� ���� ������ ��� � ��� */
	ADC_CU_RCC_PERIPH_DMA_GPIO();

	/* ��������� ������������ ��� */
	ADC_CU_RCC_PERIPH_ADC();

	 /* �������� ������ ������ ��� ���������� ������ ��� */
	 GPIO_InitStructure.GPIO_Pin = ADC_CU_MULTI_Pin1;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	 GPIO_Init(ADC_CU_Port_Group1, &GPIO_InitStructure);


	/* �������� ������ ��� */
	ADC_Cmd(ADC_CU, DISABLE);

	/* �������� �������� ��������� */
	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div8;
	/* �������� ��������� */
	ADC_CommonInit(&ADC_CommonInitStruct);

	/* ������ ��������� � ��������� */
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;	/* ���������� 12 ��� */
	ADC_InitStruct.ADC_ScanConvMode = ENABLE;					/* ������������ ������� */
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;			/* ����������� �������������� */
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfConversion = 4;							/* ����� ������� � ������ */
  /* �������� ��������� */
	ADC_Init(ADC_CU, &ADC_InitStruct);

	/* ����� ������������������ � ������� �������������� ������� � ������ */
	ADC_RegularChannelConfig(ADC_CU, ADC_Channel_12, 1, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC_CU, ADC_Channel_13, 2, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC_CU, ADC_Channel_10, 3, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC_CU, ADC_Channel_11, 4, ADC_SampleTime_480Cycles);



 	/* ��������� ��������� ��� DMA2 Stream4 channel 0 */
	DMA_InitStruct.DMA_Channel = ADC_CU_DMA_CHANNEL;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC_CU->DR;		/* ������ ����� ��������� ������  */
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&sys_adc_buf;		/* ������� ����� ��������� ������  */
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;			/* ����� ���������� � ��� */
	DMA_InitStruct.DMA_BufferSize = 4;																		/* ��������� ����� ������������ ������	*/
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 		/* ���������� ����� ��������� */
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;			/* ������� ������ ������������� */
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; /* ����������� - 16 ��� */
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;													/* �������� �����  */
	DMA_InitStruct.DMA_Priority = DMA_Priority_Low;												/* ��������� ������ ������  */
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	/* ��������� ��������� */
	DMA_Init(ADC_CU_DMA_STREAM, &DMA_InitStruct);

	/* �������� �� ��������������� ��� ������� */
	ADC_DMARequestAfterLastTransferCmd(ADC_CU,ENABLE);

	/* ��������� ������ ��� DMA2_Stream4 */
	DMA_Cmd(ADC_CU_DMA_STREAM, ENABLE);

	/* ��������� ���������� �� ��� ��� ������ ������ */
	DMA_ITConfig(ADC_CU_DMA_STREAM, DMA_IT_TC, ENABLE);
	NVIC_SetPriority(ADC_CU_DMA_IRQ, PRIORITET_ADC);
	NVIC_EnableIRQ(ADC_CU_DMA_IRQ);

	/* ��������� ������ ��� ��� ��� */
	ADC_DMACmd(ADC_CU,ENABLE);

	ADC_CU->CR2 = 0x301;

	/* ��������� ������ ��� */
	ADC_Cmd(ADC_CU, ENABLE);
}
void DMA2_Stream4_IRQHandler(void)
    {
	static portBASE_TYPE xHigherPriorityTaskWoken_ADC = pdFALSE;

	DMA_ClearFlag(ADC_CU_DMA_STREAM,DMA_FLAG_TCIF4);

	xSemaphoreGiveFromISR(xADC_Semaphore, &xHigherPriorityTaskWoken_ADC);
        //�������������� ������������ ��������� ��� ���������� ������� ������� �� ����������
        if( xHigherPriorityTaskWoken_ADC != pdFALSE )
        {
        	taskYIELD();
        }

	//ADC_SoftwareStartConv(ADC1);
    }
void prvOperation_with_ADC_results(void *pvParameters)
    {
	float volt, volt_s=16.8f, curr_all = 0.0f, action=0.0f, power=0.0f, capacity_batt = CAPACITY_BATT, percent_cap;
	float al_c_cal;
	static float all_current_sum_calib;
	static float all_current_offset;
	static uint8_t cnt_current_calib, cnt_delay_start_calib;


	while(1)
	    {
		xSemaphoreTake(xADC_Semaphore, portMAX_DELAY);
		cnt_cycle_adc = (uint32_t)DWT->CYCCNT - cnt_ticks_adc;
		cnt_ticks_adc = (uint32_t)DWT->CYCCNT;


		al_c_cal = -(float)(sys_adc_buf[0])*K_ADC_ALL_CURRENT;

		//�������� ��� ������������ ����������� ����������
		if (cnt_delay_start_calib <255)
		    {
			cnt_delay_start_calib++;
		    }
		else
		    {
			if (cnt_current_calib < 128)
			    {
				//������� ��������
				cnt_current_calib++;
				all_current_sum_calib+=al_c_cal;
			    }
			else
			    {
				if (cnt_current_calib == 128)
				    {
					all_current_offset = (float)(all_current_sum_calib/(float)cnt_current_calib);
					cnt_current_calib++;
				    }

				//������� � ������ ��������
				curr_all = ((float)K_EXP_DIFF_ADC_CURRENT*curr_all+(1.0f-(float)K_EXP_DIFF_ADC_CURRENT)*(al_c_cal-all_current_offset));
			    }
		    }

		volt =K_EXP_DIFF_ADC_VOLTAGE*volt+(1.0f-K_EXP_DIFF_ADC_VOLTAGE)*(K_ADC_ALL_VOLTAGE*(float)sys_adc_buf[1]);

		power = curr_all*volt;
		action = action+(curr_all*0.277777f)*DT_ADC;

		volt_s = K_EXP_DIFF_ADC_VOLTAGE_SM*volt_s+(1.0f-K_EXP_DIFF_ADC_VOLTAGE_SM)*volt;
		if (capacity_batt>action)
		percent_cap = 100.0f*(capacity_batt - action)/capacity_batt;


		if ((counter_sys > 15000)&&(volt < 11.5f)&&(volt > 8.0f))
		    {
			supervisorState |= STATE_LOW_BATTERY;
			buzzer_flag = BUZZER_FLAG_VOLTAGE_LOW;
		    }
		//��������
		xSemaphoreTake(xADC_Mutex, portMAX_DELAY);
		All_Current = curr_all;
		All_voltage = volt;
		All_action = action;
		All_power = power;
		All_voltage_smooth = volt_s;
		All_percent_cap = percent_cap;
		xSemaphoreGive(xADC_Mutex);
		//taskEXIT_CRITICAL();

		ovf_adc_stack = uxTaskGetStackHighWaterMark(adc_handle);

		xSemaphoreGive(xSD_ADC_collect_Semaphore);
		////////////////////
	    }
    }
