
#include "I2C_BARO_MAG.h"
#include "PWM_PPM.h"
#include <math.h>
#include "Config.h"
#define _MAG_INTERNAL
//#define _MAG_EXTERNAL


//Вспомогательные промежуточные переменные
uint8_t i2c_inx, i2c_dev_adr, i2c_cnt;
uint8_t i2c_buf[15], i2c_buf_mag[7], i2c_buf_baro[7];
uint8_t Flag_bar_temper;
uint16_t C1, C2, C3, C4, C5, C6, manufactura, crc;
uint16_t err_num_i2c_def, err_num_i2c_line;
uint8_t flag_err_i2c;
//****************************************************//

// Хандл и Стек
TaskHandle_t mag_baro_handle;
uint32_t ovf_mag_baro_stack;

SemaphoreHandle_t xMAG_BARO_Semaphore = NULL;
SemaphoreHandle_t xBAROMAG_UKF_Mutex;
//****************************************************//

//Полезные данные
int16_t mx, my, mz;
float mxf, myf, mzf;
float M_X, M_Y, M_Z;
float Temperature, Altitude, Pressure;
//****************************************************//
//Флаги
uint8_t flag_start_home_baro;
uint8_t compass_not_health, baro_not_health;
enum Err_Sensor Err_Mag, Err_Baro;

//счетчик тактов для измерения цикла
uint32_t cnt_cycle_mag, cnt_ticks_mag;
uint32_t cnt_cycle_baro, cnt_ticks_baro;

//калибровочные коэффициенты
#ifdef _MAG_INTERNAL
//внутренний компас
const float mag_x_bios = 235.095428f, mag_y_bios = -308.824699f, mag_z_bios = 0.407281f;
const float m1 = 1.050349f, 	m2 = 0.013182f, 	m3 = 0.018234f;
const float 			m5 = 1.080057f, 	m6 = -0.000538f;
const float 						m9 = 1.186722f;
#endif
//внешний компас
//на проводе дома
//const float mag_x_bios = -12.597431f, mag_y_bios = 95.272002f, mag_z_bios = 3.337495f;
//const float m1 = 5.544633f, m2 = 0.044157f, m3 = -0.048782f;
//const float m4 = 0.044157f, m5 = 5.450562f, m6 = -0.052568f;
//const float m7 = -0.048782f, m8 = -0.052568f, m9 = 6.340859f;

#ifdef _MAG_EXTERNAL
//в деревне
//const float mag_x_bios = -33.845827f, mag_y_bios = 65.105180f, mag_z_bios = -8.810466f;
//const float m1 = 5.084115f, m2 = 0.051629f, m3 = -0.013368f;
//const float m4 = 0.051629f, m5 = 5.037634f, m6 = -0.048618f;
//const float m7 = -0.013368f, m8 = -0.048618f, m9 = 5.803905f;
//в деревне (ровная сфера)
//const float mag_x_bios = -34.740234f, mag_y_bios = 64.616275f, mag_z_bios = -5.849274f;
//const float m1 = 5.059997f, m2 = 0.051608f, m3 = -0.011655f;
//const float m4 = 0.051608f, m5 = 5.024446f, m6 = -0.058183f;
//const float m7 = -0.011655f, m8 = -0.058183f, m9 = 5.778027f;
//за храмом
//const float mag_x_bios = -14.138508f, mag_y_bios = 101.658827f, mag_z_bios = 9.686203f;
//const float m1 = 5.033572f, m2 = 0.029271f, m3 = -0.050326f;
//const float m4 = 0.029271f, m5 = 4.959995f, m6 = -0.037160f;
//const float m7 = -0.050326f, m8 = -0.037160f, m9 = 5.746589f;
//на стадионе
const float mag_x_bios = -30.722438f, mag_y_bios = 59.441092f, mag_z_bios = -15.127328f;
const float m1 = 5.155095f, m2 = 0.043244f, m3 = -0.031934f;
const float m4 = 0.043244f, m5 = 5.084305f, m6 = -0.048353f;
const float m7 = -0.031934f, m8 = -0.048353f, m9 = 5.930254f;
#endif

void I2C_Init_for_loop(void)
{
	GPIO_InitTypeDef GPIO_I2C_InitStruct;
	I2C_InitTypeDef  I2C_InitStruct;

	/* подадим такт на порт */
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	I2C_RCC_Periph_Pin();
	/* настройка выводов и2ц */
	GPIO_I2C_InitStruct.GPIO_Pin 		= I2C_Pin_SCL;
	GPIO_I2C_InitStruct.GPIO_Speed 		= GPIO_Speed_100MHz;
	GPIO_I2C_InitStruct.GPIO_Mode 		= GPIO_Mode_AF;
	GPIO_I2C_InitStruct.GPIO_OType 		= GPIO_OType_PP;//GPIO_OType_PP GPIO_OType_OD
	GPIO_I2C_InitStruct.GPIO_PuPd 		= GPIO_PuPd_NOPULL;//GPIO_PuPd_UP
	GPIO_Init(I2C_GPIO, &GPIO_I2C_InitStruct);
	GPIO_PinAFConfig(I2C_GPIO, I2C_Pin_Soucre_SCL, I2C_Pin_AF);

	GPIO_I2C_InitStruct.GPIO_Pin 		= I2C_Pin_SDA;
	GPIO_I2C_InitStruct.GPIO_Speed 		= GPIO_Speed_100MHz;
	GPIO_I2C_InitStruct.GPIO_Mode 		= GPIO_Mode_AF;
	GPIO_I2C_InitStruct.GPIO_OType 		= GPIO_OType_OD;//GPIO_OType_PP GPIO_OType_OD
	GPIO_I2C_InitStruct.GPIO_PuPd 		= GPIO_PuPd_NOPULL;//GPIO_PuPd_UP
	GPIO_Init(I2C_GPIO, &GPIO_I2C_InitStruct);
	/* Соединим выводы портов с GPIO_AF_I2C1 */
	GPIO_PinAFConfig(I2C_GPIO, I2C_Pin_Soucre_SDA, I2C_Pin_AF);

	/* тактируем i2c */
	I2C_RCC_Periph();
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);

	I2C_DeInit(I2C_BARO_MAG);
	I2C_InitStruct.I2C_Mode 		= I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle		= I2C_DutyCycle_2;//I2C_DutyCycle_16_9 I2C_DutyCycle_2
	I2C_InitStruct.I2C_OwnAddress1 		= 1;
	I2C_InitStruct.I2C_Ack			= I2C_Ack_Disable;
	I2C_InitStruct.I2C_AcknowledgedAddress	= I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed 		= 200000;  /* 200kHz */
	I2C_Init(I2C_BARO_MAG, &I2C_InitStruct);
	I2C_Cmd(I2C_BARO_MAG, ENABLE);

	/* настройка прерывания И2Ц ИНС */
	NVIC_EnableIRQ(I2C_BARO_MAG_IRQ_ER);
	NVIC_EnableIRQ(I2C_BARO_MAG_IRQ_EV);

	NVIC_SetPriority(I2C_BARO_MAG_IRQ_ER, PRIORITET_I2C);
	NVIC_SetPriority(I2C_BARO_MAG_IRQ_EV, PRIORITET_I2C);
	/* разрешаем прерывание от и2ц */
	I2C_ITConfig(I2C_BARO_MAG, I2C_IT_EVT|I2C_IT_BUF|I2C_IT_ERR, ENABLE);
}
void I2C_Request_Read(uint8_t adr,uint8_t iadr,uint16_t cnt)
{
	if (I2C_GetFlagStatus(I2C_BARO_MAG, I2C_FLAG_BUSY)) return;
	if (flag_err_i2c==1) return;
	//INS_I2C_ClrAllErr();
	I2C_ITConfig(I2C_BARO_MAG, I2C_IT_EVT|I2C_IT_BUF|I2C_IT_ERR, ENABLE);
	I2C_ClearFlag(I2C_BARO_MAG,I2C_FLAG_AF|I2C_FLAG_ARLO|I2C_FLAG_BERR);
	/*  предварительная настройка переменных */
	i2c_dev_adr = adr;
	i2c_buf[0] = iadr;
	i2c_cnt = cnt;
	i2c_inx = 0;
	I2C_AcknowledgeConfig(I2C_BARO_MAG, ENABLE);
	/* передаем состояние START */
	I2C_GenerateSTART(I2C_BARO_MAG, ENABLE);
}
void I2C_Request_Write(uint8_t adr, uint8_t iadr)
{
	if (I2C_GetFlagStatus(I2C_BARO_MAG, I2C_FLAG_BUSY)) return;
	if (flag_err_i2c==1) return;
	//INS_I2C_ClrAllErr();
	I2C_ITConfig(I2C_BARO_MAG, I2C_IT_EVT|I2C_IT_BUF|I2C_IT_ERR, ENABLE);
	I2C_ClearFlag(I2C_BARO_MAG,I2C_FLAG_AF|I2C_FLAG_ARLO|I2C_FLAG_BERR);
	/*  предварительная настройка переменных */
	i2c_dev_adr = adr;
	i2c_buf[0] = iadr;
	i2c_cnt = 0;
	i2c_inx = 0;
	//I2C_AcknowledgeConfig(I2C1, DISABLE);
	/* передаем состояние START */
	I2C_GenerateSTART(I2C_BARO_MAG, ENABLE);
}
void I2C2_EV_IRQHandler(void)
{

	/*FreeRTOS*/
	//аргумент для семафора для принудительной передачи времени задаче обработки
	static portBASE_TYPE xHigherPriorityTaskWoken_MAG_BARO = pdFALSE;
	static portBASE_TYPE xHigherPriorityTaskWoken_flag = pdFALSE;
	/*FreeRTOS*/

	 switch(I2C_GetLastEvent(I2C_BARO_MAG)&(~I2C_FLAG_BTF))
	 {
	 	case I2C_EVENT_MASTER_MODE_SELECT:	 /* EV5 */
			if ((i2c_inx == 0)/* передаём вынутренний адрес */
			 ||(i2c_dev_adr&0x01 == 0)) /* режим передачи */
				I2C_Send7bitAddress(I2C_BARO_MAG,i2c_dev_adr,I2C_Direction_Transmitter);
			else  /* запуск чтения после повторного старта */
				I2C_Send7bitAddress(I2C_BARO_MAG,i2c_dev_adr,I2C_Direction_Receiver);

			break;

		case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED: 	/* EV6 */
		case I2C_EVENT_MASTER_BYTE_TRANSMITTING:			/* EV8 */
		case I2C_EVENT_MASTER_BYTE_TRANSMITTED:				/* EV8_2 */
			if ((i2c_inx == 1)&&(i2c_dev_adr&0x01))
				/* повторный старт для чтения */
				I2C_GenerateSTART(I2C_BARO_MAG, ENABLE);
			else { /* либо продолжаем писать */
				if (i2c_inx < i2c_cnt+1){
					/* передаём следующий байт */
					I2C_SendData(I2C_BARO_MAG,i2c_buf[i2c_inx]);
				        i2c_inx++;}
				else/* передача буфера завершена */{
					I2C_GenerateSTOP(I2C_BARO_MAG, ENABLE);
					I2C_ITConfig(I2C_BARO_MAG, I2C_IT_EVT|I2C_IT_BUF|I2C_IT_ERR, DISABLE);
					//I2C_AcknowledgeConfig(I2C1, DISABLE);
				}


			}
			break;

		case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:  /* EV6 */
		case (uint32_t)(I2C_FLAG_BUSY|I2C_FLAG_MSL):   /* состояние после сброса флага ADDR */
			if (i2c_cnt == 1)  /* принимаем всего один байт */
				 I2C_AcknowledgeConfig(I2C_BARO_MAG, DISABLE);
				 //I2C_GenerateSTOP(I2C1, ENABLE);
			break;

		case I2C_EVENT_MASTER_BYTE_RECEIVED:		   /* EV7 */
			if (i2c_inx == i2c_cnt+1)
			    { /* завершили чтение */
				I2C_GenerateSTOP(I2C_BARO_MAG, ENABLE);
				I2C_ITConfig(I2C_BARO_MAG, I2C_IT_EVT|I2C_IT_BUF|I2C_IT_ERR, DISABLE);
				I2C_ReceiveData(I2C_BARO_MAG);
				switch(i2c_dev_adr)
				    {
				    case HMC5883_DEFAULT_ADDRESS|0x01:
				    i2c_buf_mag[1] = i2c_buf[1]; i2c_buf_mag[2] = i2c_buf[2]; i2c_buf_mag[3] = i2c_buf[3];
				    i2c_buf_mag[4] = i2c_buf[4]; i2c_buf_mag[5] = i2c_buf[5]; i2c_buf_mag[6] = i2c_buf[6];
				    break;
				    case MS5611_DEFAULT_ADDRESS|0x01:
				    i2c_buf_baro[1] = i2c_buf[1]; i2c_buf_baro[2] = i2c_buf[2]; i2c_buf_baro[3] = i2c_buf[3];
				    break;
				    }

				/*FreeRTOS*/
				 // выставляем семафор задаче на обработку данных
				xSemaphoreGiveFromISR(xMAG_BARO_Semaphore, &xHigherPriorityTaskWoken_MAG_BARO);
			        //принудительное переключение контекста для уменьшения времени реакции на прерывание
			        if( xHigherPriorityTaskWoken_MAG_BARO != pdFALSE )
			        {
			        	taskYIELD();
			        }
			        /*FreeRTOS*/
			    }
			else
			    {/* продолжаем чтение */
				if (i2c_inx == i2c_cnt) /* будем принимать последний байт */
					I2C_AcknowledgeConfig(I2C_BARO_MAG, DISABLE);
				i2c_buf[i2c_inx] = I2C_ReceiveData(I2C_BARO_MAG);
				i2c_inx++;
			    }
			break;

		default: /* не понятное состояние = ошибка интерфейса */
			err_num_i2c_def++;
			//xSemaphoreGiveFromISR(xSD_FLAGS_collect_Semaphore, &xHigherPriorityTaskWoken_flag);
			//Sbor_buf_flags();
			break;
	}

}
void I2C2_ER_IRQHandler(void)
{
	err_num_i2c_line++;
	flag_err_i2c = 1;
	I2C_ClearFlag(I2C_BARO_MAG,I2C_FLAG_AF|I2C_FLAG_ARLO|I2C_FLAG_BERR);
	I2C_SendData(I2C_BARO_MAG,I2C_GetLastEvent(I2C_BARO_MAG));
	I2C_GenerateSTOP(I2C_BARO_MAG, ENABLE);
	I2C_ITConfig(I2C_BARO_MAG, I2C_IT_EVT|I2C_IT_BUF|I2C_IT_ERR, DISABLE);

}

void Obrabotka_Mag(void)
    {
	if (i2c_dev_adr==(HMC5883_DEFAULT_ADDRESS|0x01))
	    {
		int16_t mx1, my1, mz1;
		static int16_t mx_last, my_last, mz_last;
		static uint8_t first=0, one = 0;
		int16_t mx_r, my_r, mz_r;
		float norm_m;
		float mx_bios, my_bios, mz_bios;
		float mx_cal, my_cal, mz_cal;
		float mxf_, myf_, mzf_, MX_, MY_, MZ_;


		cnt_cycle_mag = (uint32_t)DWT->CYCCNT - cnt_ticks_mag;
		cnt_ticks_mag = (uint32_t)DWT->CYCCNT;

		#ifdef _MAG_EXTERNAL
		mx1 = (int16_t)(((uint16_t)i2c_buf_mag[1]<<8)+i2c_buf_mag[2]);
		mz1 = -(int16_t)(((uint16_t)i2c_buf_mag[3]<<8)+i2c_buf_mag[4]);
		my1 = -(int16_t)(((uint16_t)i2c_buf_mag[5]<<8)+i2c_buf_mag[6]);
		#endif
		#ifdef _MAG_INTERNAL
		mx1 = (int16_t)(((uint16_t)i2c_buf_mag[1]<<8)+i2c_buf_mag[2]);
		mz1 = (int16_t)(((uint16_t)i2c_buf_mag[3]<<8)+i2c_buf_mag[4]);
		my1 = (int16_t)(((uint16_t)i2c_buf_mag[5]<<8)+i2c_buf_mag[6]);
		#endif

		if (first < 30)
		    {
			mx_last = mx1;
			my_last = my1;
			mz_last = mz1;
			first++;
		    }
		if (mx1>mx_last) mx_r = mx1 - mx_last; else mx_r = mx_last - mx1;
		if (my1>my_last) my_r = my1 - my_last; else my_r = my_last - my1;
		if (mz1>mz_last) mz_r = mz1 - mz_last; else mz_r = mz_last - mz1;

		if (((mx_r > 120)||(my_r > 120)||(mz_r > 120))&& one == 0)
		    {
			mx = mx_last;
			my = my_last;
			mz = mz_last;
			one = 1;
		    }
		else
		{
			mx = mx1;
			my = my1;
			mz = mz1;
			mx_last = mx1;
			my_last = my1;
			mz_last = mz1;
			one = 0;
		}

		mx_bios = (float)mx - mag_x_bios;
		my_bios = (float)my - mag_y_bios;
		mz_bios = (float)mz - mag_z_bios;

//		mx_cal = m1*mx_bios + m2*my_bios + m3*mz_bios;
//		my_cal = m4*mx_bios + m5*my_bios + m6*mz_bios;
//		mz_cal = m7*mx_bios + m8*my_bios + m9*mz_bios;
		mx_cal = m1*mx_bios + m2*my_bios + m3*mz_bios;
		my_cal = m2*mx_bios + m5*my_bios + m6*mz_bios;
		mz_cal = m3*mx_bios + m6*my_bios + m9*mz_bios;



		//защитить
		//taskENTER_CRITICAL();
		xSemaphoreTake(xBAROMAG_UKF_Mutex, portMAX_DELAY);
		mxf = mx_cal;
		myf = my_cal;
		mzf = mz_cal;
		xSemaphoreGive(xBAROMAG_UKF_Mutex);
		//taskEXIT_CRITICAL();
		//
		 if (compass_not_health == 0)
		 xSemaphoreGive(xSD_MAG_BARO_collect_Semaphore);

	    }
    }
void Obrabotka_Baro(void)
    {

	if ((i2c_dev_adr==(MS5611_DEFAULT_ADDRESS|0x01)))
	    {
		static uint32_t ADC_Temp, ADC_Press;
		static int32_t dt_baro, TEMP_baro, TEMP_baro2;
		static int32_t TEMP_baro_act, TEMP_baro_last, deltat;
		static int64_t OFF_baro, sens_baro, OFF_baro2, sens_baro2;
		static float press_last, alt_last = 0.0f, temp11;
		static uint16_t shet_alt;
		static double Altitude_s;
		static float Bios_alt;
		static float press_act, pressik, altik, deltaalt;
		static uint8_t first_temp = 0, first_alt = 0;

		if (Flag_bar_temper == 1)//temp
		    {
			ADC_Temp = (uint32_t)(i2c_buf_baro[1] << 16 | i2c_buf_baro[2] << 8 | i2c_buf_baro[3]);

			dt_baro = (int32_t)(ADC_Temp - (uint32_t)C5*256);
			TEMP_baro = (int32_t)(2000+((int64_t)dt_baro*C6)/(1<<23));
			TEMP_baro2 = 0;
			if (TEMP_baro < 2000)
			{
				TEMP_baro2 = (dt_baro * dt_baro) / (2 << 30);
			}


			TEMP_baro = TEMP_baro - TEMP_baro2;

			if (first_temp == 0)
			    {
				first_temp = 1;
				TEMP_baro_last = TEMP_baro;
			    }
			if (TEMP_baro >= TEMP_baro_last)
			deltat = TEMP_baro - TEMP_baro_last;
			else
			deltat = TEMP_baro_last - TEMP_baro;

			if ((TEMP_baro != NAN)&&(deltat < 500 ))
			    {
				TEMP_baro_act = TEMP_baro;
				TEMP_baro_last = TEMP_baro_act;
			    }
			else TEMP_baro_act = TEMP_baro_last;

			temp11 = (float)TEMP_baro_act/100.0f;





		    }

		if (Flag_bar_temper == 2)//press
		    {
			cnt_cycle_baro = (uint32_t)DWT->CYCCNT - cnt_ticks_baro;
			cnt_ticks_baro = (uint32_t)DWT->CYCCNT;
			ADC_Press = (uint32_t)(i2c_buf_baro[1] << 16 | i2c_buf_baro[2] << 8 | i2c_buf_baro[3]);
			OFF_baro = (int64_t)((int64_t)C2*65536+((int64_t)C4*(int64_t)dt_baro)/128);
			sens_baro = (int64_t)((int64_t)C1*32768+((int64_t)C3*(int64_t)dt_baro)/256);
			//Pressure = (int32_t)(((int64_t)(ADC_Press*sens_baro/2097152)-(int64_t)OFF_baro)/32768);

			OFF_baro2 = 0;
			sens_baro2 = 0;

			if (TEMP_baro < 2000)
			{
			    OFF_baro2 = 5 * ((TEMP_baro_act - 2000) * (TEMP_baro_act - 2000)) / 2;
			    sens_baro2 = 5 * ((TEMP_baro_act - 2000) * (TEMP_baro_act - 2000)) / 4;
			}
			if (TEMP_baro < -1500)
			{
			    OFF_baro2 = OFF_baro2 + 7 * ((TEMP_baro_act + 1500) * (TEMP_baro_act + 1500));
			    sens_baro2 = sens_baro2 + 11 * ((TEMP_baro_act + 1500) * (TEMP_baro_act + 1500)) / 2;
			}
			OFF_baro = OFF_baro - OFF_baro2;
			sens_baro = sens_baro - sens_baro2;

			pressik = (float)((int64_t)ADC_Press * sens_baro / 2097152 - OFF_baro)*(1.0f/32768.0f);
			if (!(isnanf(pressik)))
			    {
				press_act = pressik;
				press_last = press_act;
			    }
			else press_act = press_last;
			Pressure = press_act;
			if (flag_start_home_baro == 0)
			    {
				shet_alt = 0;
				Altitude_s = 0.0f;
				first_alt = 0;
				flag_start_home_baro = 1;
			    }
			if (shet_alt < 10) shet_alt++;
			else
			    {
				if ((shet_alt < 10+15)&&(shet_alt >= 10))
				    {
					Altitude = 0;
					Altitude_s +=(float)44330.0f * (float)((float)1.0f - (float)powf((float)press_act/101325.0f, 0.19029495f));
					shet_alt++;
				    }
				else
				    {
					if (shet_alt == 10+15)
					    {
						Bios_alt = (float)((double)Altitude_s/(double)(shet_alt-10));
						shet_alt++;
						//flag_start_home_baro = 1;

					    }
					altik = (float)44330.0f * (float)((float)1.0f - (float)powf((float)press_act/101325.0f, 0.19029495f));
					altik -= Bios_alt;

					if (first_alt == 0)
					    {
						first_alt = 1;
						alt_last = altik;
					    }
					if (altik >= alt_last)
					deltaalt = altik - alt_last;
					else
					deltaalt = alt_last - altik;

					if ((!(isnanf(altik)))&&(deltaalt < 5.0f))
					    {
						alt_last = altik;
						//защитить
						//taskENTER_CRITICAL();
						xSemaphoreTake(xBAROMAG_UKF_Mutex, portMAX_DELAY);
						Temperature = temp11;
						Altitude = altik;
						xSemaphoreGive(xBAROMAG_UKF_Mutex);
						//taskEXIT_CRITICAL();
						////////////////////
						 if ((compass_not_health == 1)&&(baro_not_health == 0))
						 xSemaphoreGive(xSD_MAG_BARO_collect_Semaphore);
					    }

				    }
			    }

		    }

	    }
    }
void prvMAG_BARO_Processing(void *pvParameters)
    {

	while(1)
	    {
		xSemaphoreTake(xMAG_BARO_Semaphore, portMAX_DELAY);
		Obrabotka_Mag();
		Obrabotka_Baro();
		ovf_mag_baro_stack = uxTaskGetStackHighWaterMark(mag_baro_handle);
		ovf_full_stack = xPortGetFreeHeapSize();
	    }
    }
void prvMAG_BARO_Request(void *pvParameters)
    {
	static float last_bar;
	static uint8_t schet1, schet2;
	static int16_t mxl, myl, mzl;

	while(1)
	    {
		if ((mx == mxl)&&(my == myl)&&(mz == mzl)&&(schet1<5)) schet1++;
		if ((mx != mxl)||(my != myl)||(mz != mzl)) schet1=0;
		if ((last_bar == Pressure)&&(schet2<5)) schet2++;
				if (last_bar != Pressure) schet2=0;

		if (schet1 == 5)
		    {
			schet1 = 6;
			Err_Mag = Err_value;
			supervisorState |= ERR_SENSOR;
			compass_not_health = 1;
			buzzer_flag = BUZZER_FLAG_FAIL_SENSOR;
			xSemaphoreGive(xSD_FLAGS_collect_Semaphore);

		    }
		if (schet2 == 5)
		    {
			schet2 = 6;
			Err_Baro = Err_value;
			supervisorState |= ERR_SENSOR;
			baro_not_health = 1;
			buzzer_flag = BUZZER_FLAG_FAIL_SENSOR;
			xSemaphoreGive(xSD_FLAGS_collect_Semaphore);
		    }
		mxl = mx;
		myl = my;
		mzl = mz;
		last_bar = Pressure;
		 if ((compass_not_health == 1)&&(baro_not_health == 1))
		 xSemaphoreGive(xSD_MAG_BARO_collect_Semaphore);
		vTaskDelay(2);
		I2C_Request_Read(HMC5883_DEFAULT_ADDRESS|0x01, 0x03, 6);
		vTaskDelay(6);
		I2C_Request_Write(MS5611_DEFAULT_ADDRESS, 0x58);
		Flag_bar_temper = 1;
		vTaskDelay(7);
		I2C_Request_Read(HMC5883_DEFAULT_ADDRESS|0x01, 0x03, 6);
		vTaskDelay(3);
		I2C_Request_Read(MS5611_DEFAULT_ADDRESS|0x01, 0x00, 3);
		vTaskDelay(3);
		I2C_Request_Write(MS5611_DEFAULT_ADDRESS, 0x48);
		Flag_bar_temper = 2;
		vTaskDelay(7);
		I2C_Request_Read(HMC5883_DEFAULT_ADDRESS|0x01, 0x03, 6);
		vTaskDelay(3);
		I2C_Request_Read(MS5611_DEFAULT_ADDRESS|0x01, 0x00, 3);
		vTaskDelay(8);
	    }
    }
void I2C_ClockToggling(void)
{
    const int delay = 10000;
    GPIO_InitTypeDef GPIO_InitStructure;


    /* Configure SCL GPIO as output */
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    I2C_RCC_Periph_Pin();
	GPIO_InitStructure.GPIO_Pin = I2C_Pin_SCL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(I2C_GPIO, &GPIO_InitStructure);
	/* Configure SDA GPIO as input */
	GPIO_InitStructure.GPIO_Pin = I2C_Pin_SDA;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(I2C_GPIO, &GPIO_InitStructure);

	uint8_t input_pin_state = 100;
	input_pin_state = GPIO_ReadInputDataBit(I2C_GPIO, I2C_Pin_SDA);
	while (input_pin_state == 0)
    {
        input_pin_state = GPIO_ReadInputDataBit(I2C_GPIO, I2C_Pin_SDA);
        GPIO_SetBits(I2C_GPIO, I2C_Pin_SCL);
        int j = 0;
        for (j = 0; j < delay; j++);
        GPIO_ResetBits(I2C_GPIO, I2C_Pin_SCL);
        for (j = 0; j < delay; j++);
    }
	GPIO_DeInit(I2C_GPIO);
	int j = 0;
	for (j = 0; j < delay; j++);
}
void I2C_Init_Baro_Mag_for_init(void)
{

	I2C_InitTypeDef I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable I2C and GPIO clocks */
	RCC_AHB1PeriphClockCmd(I2C_GPIO, ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	I2C_RCC_Periph();

	/* Configure I2C pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin = I2C_Pin_SCL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// GPIO_OType_PP
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_PinAFConfig(I2C_GPIO, I2C_Pin_Soucre_SCL, I2C_Pin_AF);
	GPIO_Init(I2C_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = I2C_Pin_SDA;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;// GPIO_OType_PP
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_PinAFConfig(I2C_GPIO, I2C_Pin_Soucre_SDA, I2C_Pin_AF);
	GPIO_Init(I2C_GPIO,&GPIO_InitStructure);

	/* I2C configuration */
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x01;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	/* Apply I2C configuration after enabling it */
	I2C_Init(I2C_BARO_MAG, &I2C_InitStructure);

	/* I2C Peripheral Enable */
	I2C_Cmd(I2C_BARO_MAG, ENABLE);

}

void I2C_ByteWrite(u8 slaveAddr, u8* pBuffer, u8 writeAddr)
{
    // ENTR_CRT_SECTION();

    /* Send START condition */
    I2C_GenerateSTART(I2C_BARO_MAG, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(I2C_BARO_MAG, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU6050 address for write */
    I2C_Send7bitAddress(I2C_BARO_MAG, slaveAddr, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(I2C_BARO_MAG, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Send the MPU6050's internal address to write to */
    I2C_SendData(I2C_BARO_MAG, writeAddr);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C_BARO_MAG, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send the byte to be written */
    I2C_SendData(I2C_BARO_MAG, *pBuffer);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C_BARO_MAG, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STOP condition */
    I2C_GenerateSTOP(I2C_BARO_MAG, ENABLE);

    // EXT_CRT_SECTION();
}
void I2C_BufferRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{
    // ENTR_CRT_SECTION();

    /* While the bus is busy */
    while (I2C_GetFlagStatus(I2C_BARO_MAG, I2C_FLAG_BUSY));

    /* Send START condition */
    I2C_GenerateSTART(I2C_BARO_MAG, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(I2C_BARO_MAG, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU6050 address for write */
    I2C_Send7bitAddress(I2C_BARO_MAG, slaveAddr, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(I2C_BARO_MAG, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Clear EV6 by setting again the PE bit */
    I2C_Cmd(I2C_BARO_MAG, ENABLE);

    /* Send the MPU6050's internal address to write to */
    I2C_SendData(I2C_BARO_MAG, readAddr);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C_BARO_MAG, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STRAT condition a second time */
    I2C_GenerateSTART(I2C_BARO_MAG, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(I2C_BARO_MAG, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU6050 address for read */
    I2C_Send7bitAddress(I2C_BARO_MAG, slaveAddr, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(I2C_BARO_MAG, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    /* While there is data to be read */
    while (NumByteToRead)
    {
        if (NumByteToRead == 1)
        {
            /* Disable Acknowledgment */
            I2C_AcknowledgeConfig(I2C_BARO_MAG, DISABLE);

            /* Send STOP Condition */
            I2C_GenerateSTOP(I2C_BARO_MAG, ENABLE);
        }

        /* Test on EV7 and clear it */
        if (I2C_CheckEvent(I2C_BARO_MAG, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            /* Read a byte from the MPU6050 */
            *pBuffer = I2C_ReceiveData(I2C_BARO_MAG);

            /* Point to the next location where the byte read will be saved */
            pBuffer++;

            /* Decrement the read bytes counter */
            NumByteToRead--;
        }
    }

    /* Enable Acknowledgment to be ready for another reception */
    I2C_AcknowledgeConfig(I2C_BARO_MAG, ENABLE);
    // EXT_CRT_SECTION();
}
void MS5611_Get_Coef_Calib(void)
    {
	u8 tmpBuffer[2];
	int i = 0;
	I2C_BufferRead(MS5611_DEFAULT_ADDRESS, tmpBuffer, 0xA0, 2);
	manufactura = (uint16_t)(tmpBuffer[i] << 8 | tmpBuffer[i + 1]);
	I2C_BufferRead(MS5611_DEFAULT_ADDRESS, tmpBuffer, 0xA2, 2);
	C1 = (uint16_t)(tmpBuffer[i] << 8 | tmpBuffer[i + 1]);
	I2C_BufferRead(MS5611_DEFAULT_ADDRESS, tmpBuffer, 0xA4, 2);
	C2 = (uint16_t)(tmpBuffer[i] << 8 | tmpBuffer[i + 1]);
	I2C_BufferRead(MS5611_DEFAULT_ADDRESS, tmpBuffer, 0xA6, 2);
	C3 = (uint16_t)(tmpBuffer[i] << 8 | tmpBuffer[i + 1]);
	I2C_BufferRead(MS5611_DEFAULT_ADDRESS, tmpBuffer, 0xA8, 2);
	C4 = (uint16_t)(tmpBuffer[i] << 8 | tmpBuffer[i + 1]);
	I2C_BufferRead(MS5611_DEFAULT_ADDRESS, tmpBuffer, 0xAA, 2);
	C5 = (uint16_t)(tmpBuffer[i] << 8 | tmpBuffer[i + 1]);
	I2C_BufferRead(MS5611_DEFAULT_ADDRESS, tmpBuffer, 0xAC, 2);
	C6 = (uint16_t)(tmpBuffer[i] << 8 | tmpBuffer[i + 1]);
	I2C_BufferRead(MS5611_DEFAULT_ADDRESS, tmpBuffer, 0xAE, 1);
	crc = (uint16_t)(tmpBuffer[i]);
	if (!(C1&&C2&&C3&&C4&&C5&&C6)) Err_Baro = Err_Init;
    }
void Baro_Mag_Init_Full(void)
{
	//создание двоичного семафора для передачи обработки данных
	vSemaphoreCreateBinary(xMAG_BARO_Semaphore);
	//vSemaphoreCreateBinary(xMAG_BARO_Err_Semaphore);
	//задача обработки данных с датчиков
	xTaskCreate(prvMAG_BARO_Processing,(signed char*)"MAG_BARO_Processing", 130, NULL, PRIORITET_TASK_PROCESSING_MAG_BARO, mag_baro_handle);
	//xTaskCreate(prvErrToggling,(signed char*)"MAG_BARO_err", 50, NULL, PRIORITET_TASK_ERR_MAG_BARO, ( xTaskHandle * ) NULL);
	u8 buf1 = 0;
	u8 buf2 = 0b00000000;
	u8 buf3 = 0b01011000;
	u8 tmp;
	uint16_t pause_init;
	I2C_ClockToggling();
	I2C_Init_Baro_Mag_for_init();
	I2C_ByteWrite(HMC5883_DEFAULT_ADDRESS, &buf1, 0x02);
	I2C_ByteWrite(HMC5883_DEFAULT_ADDRESS, &buf2, 0x01);
	I2C_ByteWrite(HMC5883_DEFAULT_ADDRESS, &buf3, 0x00);
	I2C_BufferRead(HMC5883_DEFAULT_ADDRESS, &tmp, 0x02, 1);
	if (tmp != buf1) Err_Mag = Err_Init;

	MS5611_Get_Coef_Calib();
	I2C_ClockToggling();
	for (pause_init = 0; pause_init < 15000; pause_init++); pause_init = 0;
	I2C_Init_for_loop();
	//создание задачи переодического вызова магнетометра барометра
	xTaskCreate(prvMAG_BARO_Request,(signed char*)"MAG_BARO_Request", 70, NULL, PRIORITET_TASK_REQUEST_MAG_BARO, ( xTaskHandle * ) NULL);

}


