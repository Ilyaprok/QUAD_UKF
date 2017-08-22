
#include "stm32f4xx.h"


#include "Config.h"
#include "stm32f4xx_conf.h"

#include "Buzzer.h"


#define Buzzer_ON() GPIO_SetBits(GPIOE, GPIO_Pin_5|GPIO_Pin_4)
#define Buzzer_OFF() GPIO_ResetBits(GPIOE, GPIO_Pin_5|GPIO_Pin_4)

volatile uint8_t buzzer_flag;
volatile uint32_t counter_buzzer_sys;

void Buzzer_Function(void)
    {
	static uint32_t count;
	static uint8_t flag_one, povtor, flag_disable;
	static uint8_t buzzer_number_povtor;
	static uint16_t buzzer_dlit_signal, buzzer_dlit_pause;
	if (flag_disable == 0)
	    {
		taskENTER_CRITICAL();
		switch(buzzer_flag)
		    {
				//case 0: return;break;

		    		case BUZZER_FLAG_VALUE_TRANSMITTED:
		    		buzzer_dlit_pause = BUZZER_DLIT_PAUSE_VALUE_TRANSMITTED;
		    		buzzer_dlit_signal = BUZZER_DLIT_SIGNAL_VALUE_TRANSMITTED;
		    		buzzer_number_povtor = BUZZER_NUMBER_POVTOR_VALUE_TRANSMITTED;
		    		break;
		    		case BUZZER_FLAG_VOLTAGE_LOW:
		    		buzzer_dlit_pause = BUZZER_DLIT_PAUSE_VOLTAGE_LOW;
		    		buzzer_dlit_signal = BUZZER_DLIT_SIGNAL_VOLTAGE_LOW;
		    		buzzer_number_povtor = BUZZER_NUMBER_POVTOR_VOLTAGE_LOW;
		    		break;
		    		case BUZZER_FLAG_FAIL_SENSOR:
		    		buzzer_dlit_pause = BUZZER_DLIT_PAUSE_FAIL_SENSOR;
		    		buzzer_dlit_signal = BUZZER_DLIT_SIGNAL_FAIL_SENSOR;
		    		buzzer_number_povtor = BUZZER_NUMBER_POVTOR_FAIL_SENSOR;
		    		break;
		    		case BUZZER_FLAG_ARMING_DISARMING:
		    		buzzer_dlit_pause = BUZZER_DLIT_PAUSE_ARMING_DISARMING;
		    		buzzer_dlit_signal = BUZZER_DLIT_SIGNAL_ARMING_DISARMING;
		    		buzzer_number_povtor = BUZZER_NUMBER_POVTOR_ARMING_DISARMING;
		    		break;
		    		case BUZZER_FLAG_FAIL_SD:
		    		buzzer_dlit_pause = BUZZER_DLIT_PAUSE_FAIL_SD;
		    		buzzer_dlit_signal = BUZZER_DLIT_SIGNAL_FAIL_SD;
		    		buzzer_number_povtor = BUZZER_NUMBER_POVTOR_FAIL_SD;
		    		break;
		    		case BUZZER_FLAG_LIFT_COMMAND:
		    		buzzer_dlit_pause = BUZZER_DLIT_PAUSE_LIFT_COMMAND;
		    		buzzer_dlit_signal = BUZZER_DLIT_SIGNAL_LIFT_COMMAND;
		    		buzzer_number_povtor = BUZZER_NUMBER_POVTOR_LIFT_COMMAND;
		    		break;
		    		case BUZZER_FLAG_GLITCH:
		    		buzzer_dlit_pause = BUZZER_DLIT_PAUSE_GLITCH;
		    		buzzer_dlit_signal = BUZZER_DLIT_SIGNAL_GLITCH;
		    		buzzer_number_povtor = BUZZER_NUMBER_POVTOR_GLITCH;
		    		break;
		    		case BUZZER_FLAG_NOT_MODE:
		    		buzzer_dlit_pause = BUZZER_DLIT_PAUSE_NOT_MODE;
		    		buzzer_dlit_signal = BUZZER_DLIT_SIGNAL_NOT_MODE;
		    		buzzer_number_povtor = BUZZER_NUMBER_POVTOR_NOT_MODE;
		    		break;
		    		default: break;

		    	    }
		taskEXIT_CRITICAL();
	    }

	if (povtor >= buzzer_number_povtor)
	    {
		taskENTER_CRITICAL();
		flag_disable = 0;
		flag_one = 0;
		povtor = 0;
		buzzer_flag = 0;
		Buzzer_OFF();
		taskEXIT_CRITICAL();
		return;
	    }
	if (flag_one == 0)
	    {
		taskENTER_CRITICAL();
		flag_disable = 1;
		flag_one = 1;
		count = counter_buzzer_sys;
		Buzzer_ON();
		taskEXIT_CRITICAL();
	    }
	if (counter_buzzer_sys - count == buzzer_dlit_signal)
	    {
		Buzzer_OFF();
	    }
	if (counter_buzzer_sys - count == buzzer_dlit_signal + buzzer_dlit_pause)
	    {
		taskENTER_CRITICAL();
		flag_one = 0;
		povtor++;
		taskEXIT_CRITICAL();
	    }

    }
void Buzzer_Init(void)
    {
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOE, GPIO_Pin_5|GPIO_Pin_4);
    }

