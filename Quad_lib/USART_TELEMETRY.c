
#include "stm32f4xx.h"
#include "Config.h"

#include "USART_TELEMETRY.h"

#include "I2C_BARO_MAG.h"
#include "MPU6500.h"
#include "IMU_INS.h"
#include "GPS.h"
#include "ADC.h"
#include "PWM_PPM.h"
#include "UKF_lib.h"
#include "PID.h"

//Промежуточные, служебные перменные
SemaphoreHandle_t xParcel_TX_Semaphore = NULL;
uint8_t cnt_irq_tx_frsky;
//****************************************************//

//Буферы приема/передачи
//bluetooth
volatile unsigned char Send_buf_Bluetooth[255], Send_count_Bluetooth = 0, Recieve_buf_Bluetooth[10], Recieve_buf_Bluetooth_last[10], Recieve_count_Bluetooth;
unsigned char is_Bluetooth = 0;
//frsky
volatile unsigned char Send_buf_Frsky[255], Send_count_Frsky = 0, Recieve_buf_Frsky[10], Recieve_buf_Frsky_last[10], Recieve_count_Frsky;
unsigned char is_Frsky = 0;
//****************************************************//


//Для удобного конвертирования
typedef union
{
float f;
unsigned char a [sizeof (float)];
} bd_float;
bd_float uart_rx, uart_tx;
//****************************************************//

void USART_INIT_BLuetooth(void)
    {
	vSemaphoreCreateBinary(xParcel_TX_Semaphore);

	GPIO_InitTypeDef GPIO_InitTypeDef_FTDI;
	USART_InitTypeDef USART_InitTypeDef_FTDI;

	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	UART_BLUETOOTH_RCC_PERIPH_Pin();

	GPIO_InitTypeDef_FTDI.GPIO_Pin = UART_BLUETOOTH_Multi_Pin;
	GPIO_InitTypeDef_FTDI.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitTypeDef_FTDI.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitTypeDef_FTDI.GPIO_OType = GPIO_OType_PP;
	GPIO_InitTypeDef_FTDI.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(UART_BLUETOOTH_GPIO, &GPIO_InitTypeDef_FTDI);

	GPIO_PinAFConfig(UART_BLUETOOTH_GPIO, UART_BLUETOOTH_Pin_Source_TX, UART_BLUETOOTH_GPIO_AF);//TX
	GPIO_PinAFConfig(UART_BLUETOOTH_GPIO, UART_BLUETOOTH_Pin_Source_RX, UART_BLUETOOTH_GPIO_AF);//RX

	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	UART_BLUETOOTH_RCC_PERIPH();

	USART_InitTypeDef_FTDI.USART_BaudRate = 921600;
	USART_InitTypeDef_FTDI.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitTypeDef_FTDI.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitTypeDef_FTDI.USART_Parity = USART_Parity_No;
	USART_InitTypeDef_FTDI.USART_StopBits = USART_StopBits_1;
	USART_InitTypeDef_FTDI.USART_WordLength = USART_WordLength_8b;

	USART_Init(UART_BLUETOOTH, &USART_InitTypeDef_FTDI);
	USART_Cmd(UART_BLUETOOTH, ENABLE);

	NVIC_EnableIRQ(UART_BLUETOOTH_IRQ);

	NVIC_SetPriority(UART_BLUETOOTH_IRQ, PRIORITET_UART_BLUETOOTH);//отправка на комп порт

	USART_ITConfig(UART_BLUETOOTH, USART_IT_RXNE, ENABLE);

	xTaskCreate(prvParcel_to_Comp,(signed char*)"Parcel_TX", 100, NULL, PRIORITET_TASK_PARCEL_TO_COMP, ( xTaskHandle * ) NULL);
    }
void USART_INIT_Frsky(void)
    {
	//vSemaphoreCreateBinary(xParcel_TX_Semaphore);

	GPIO_InitTypeDef GPIO_InitTypeDef_Frsky;
	USART_InitTypeDef USART_InitTypeDef_Frsky;
	USART_ClockInitTypeDef USART_ClockInitTypeDef_Frsky;

	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	UART_FRSKY_RCC_PERIPH_Pin_TX();

	GPIO_InitTypeDef_Frsky.GPIO_Pin = UART_FRSKY_Pin_TX;
	GPIO_InitTypeDef_Frsky.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitTypeDef_Frsky.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(UART_FRSKY_GPIO_TX, &GPIO_InitTypeDef_Frsky);

	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	UART_FRSKY_RCC_PERIPH_Pin_RX();

	GPIO_InitTypeDef_Frsky.GPIO_Pin = UART_FRSKY_Pin_RX;

	GPIO_Init(UART_FRSKY_GPIO_RX, &GPIO_InitTypeDef_Frsky);

	GPIO_PinAFConfig(GPIOC, UART_FRSKY_Pin_Source_TX, UART_FRSKY_GPIO_AF);//TX
	GPIO_PinAFConfig(GPIOD, UART_FRSKY_Pin_Source_RX, UART_FRSKY_GPIO_AF);//RX

	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	UART_FRSKY_RCC_PERIPH();

	USART_InitTypeDef_Frsky.USART_BaudRate = 9600;
	USART_InitTypeDef_Frsky.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitTypeDef_Frsky.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitTypeDef_Frsky.USART_Parity = USART_Parity_No;
	USART_InitTypeDef_Frsky.USART_StopBits = USART_StopBits_1;
	USART_InitTypeDef_Frsky.USART_WordLength = USART_WordLength_8b;

	USART_Init(UART_FRSKY, &USART_InitTypeDef_Frsky);

	USART_Cmd(UART_FRSKY, ENABLE);

	NVIC_EnableIRQ(UART_FRSKY_IRQ);

	NVIC_SetPriority(UART_FRSKY_IRQ, PRIORITET_UART_FRSKY);//отправка на комп порт

	USART_ITConfig(UART_FRSKY, USART_IT_RXNE, ENABLE);

    }

void USART2_IRQHandler(void)
{
	static uint8_t i, k;
	static portBASE_TYPE xHigherPriorityTaskWoken_flag = pdFALSE;

	if(USART_GetITStatus(UART_BLUETOOTH, USART_IT_ORE_RX) == SET)
	    {
		USART_GetFlagStatus(UART_BLUETOOTH, USART_IT_ORE_RX);
		USART_ReceiveData(UART_BLUETOOTH);
		Recieve_count_Bluetooth=0;
	    }
	if(USART_GetITStatus(UART_BLUETOOTH, USART_IT_TXE) == SET)
	{
		USART_ClearITPendingBit(UART_BLUETOOTH, USART_IT_TXE);
			if (Send_count_Bluetooth != is_Bluetooth)
			{
				USART_SendData(UART_BLUETOOTH, Send_buf_Bluetooth[Send_count_Bluetooth]);
				Send_count_Bluetooth++;
				return;
			}
			else
			{
			  Send_count_Bluetooth = 0;
			  USART_SendData(UART_BLUETOOTH, '\r');
			  USART_ITConfig(UART_BLUETOOTH, USART_IT_TXE, DISABLE);
			  return;
			}

	}
	if (USART_GetITStatus(UART_BLUETOOTH, USART_IT_RXNE) == SET)
		{
			USART_ClearITPendingBit(UART_BLUETOOTH, USART_IT_RXNE);
			Recieve_buf_Bluetooth[Recieve_count_Bluetooth] = USART_ReceiveData(UART_BLUETOOTH);
			if (Recieve_buf_Bluetooth[Recieve_count_Bluetooth] == 0x0D)
			    {


				 for (i=0; i<Recieve_count_Bluetooth; i++)
				     {
					 if (Recieve_buf_Bluetooth_last[i] != Recieve_buf_Bluetooth[i])
					     {
						 for (k=0; k<Recieve_count_Bluetooth; k++)
						     {
							 Recieve_buf_Bluetooth_last[k] = Recieve_buf_Bluetooth[k];
						     }
						 Recieve_count_Bluetooth=0;
						 return;
					     }

				     }

				 //Sbor_buf_flags();
				 for (k=0; k<Recieve_count_Bluetooth; k++)
				     {
					 Recieve_buf_Bluetooth_last[k] = Recieve_buf_Bluetooth[k];
				     }
				 Recieve_count_Bluetooth=0;
				 if (Recieve_buf_Bluetooth[0] <= 100)
				     {
					 uart_rx.a[3]=Recieve_buf_Bluetooth[1];
					 uart_rx.a[2]=Recieve_buf_Bluetooth[2];
					 uart_rx.a[1]=Recieve_buf_Bluetooth[3];
					 uart_rx.a[0]=Recieve_buf_Bluetooth[4];
					 switch(Recieve_buf_Bluetooth[0])
					     {
						 case 1: KP_ugla_xy =uart_rx.f; break;
						 case 2: KI_ugla_xy =uart_rx.f; break;
						 case 3: KD_ugla_xy =uart_rx.f; break;
						 case 4: KP_rate_xy =uart_rx.f; break;
						 case 5: KI_rate_xy =uart_rx.f; break;
						 case 6: KD_rate_xy =uart_rx.f; break;
						 case 7: KP_rate_z =uart_rx.f; break;
						 case 8: KD_rate_z =uart_rx.f; break;
						 case 9: KI_rate_z =uart_rx.f; break;
						 case 10: KP_ugla_z =uart_rx.f; break;
						 case 11: KI_ugla_z =uart_rx.f; break;
						 case 12: f_cut_diff_pid =uart_rx.f; break;

						 case 14: KP_Pos_Alt_Z =uart_rx.f; break;
						 case 15: KP_Vel_Alt_Z =uart_rx.f; break;
						 case 16: KD_Vel_Alt_Z =uart_rx.f; break;
						 case 17: KP_Acc_Alt_Z =uart_rx.f; break;
						 case 18: KI_Vel_Alt_Z =uart_rx.f; break;
						 case 19: KD_Acc_Alt_Z =uart_rx.f; break;
						 case 20: f_cut_diff_vel_alt_z =uart_rx.f; break;
						 case 21: f_cut_err_vel_alt_z =uart_rx.f; break;

						 case 22: KP_Pos_XY =uart_rx.f; break;
						 case 23: KP_Vel_XY =uart_rx.f; break;
						 case 24: KI_Vel_XY =uart_rx.f; break;
						 case 25: KD_Vel_XY =uart_rx.f; break;
						 case 26: f_cut_diff_vel_xy =uart_rx.f; break;
						 case 27: f_cut_err_vel_xy =uart_rx.f; break;

						 default: break;

					     }
					 buzzer_flag = BUZZER_FLAG_VALUE_TRANSMITTED;
					 xSemaphoreGiveFromISR(xSD_FLAGS_collect_Semaphore, &xHigherPriorityTaskWoken_flag);
				     }
			    }

			else if (Recieve_count_Bluetooth<9) Recieve_count_Bluetooth++;
		}
}
void UART4_IRQHandler(void)
{

	if(USART_GetITStatus(UART_FRSKY, USART_IT_ORE_RX) == SET)
	    {
		USART_GetFlagStatus(UART_FRSKY, USART_IT_ORE_RX);
		USART_ReceiveData(UART_FRSKY);
	    }
	if(USART_GetITStatus(UART_FRSKY, USART_IT_TXE) == SET)
	{
		cnt_irq_tx_frsky++;
		USART_ClearITPendingBit(UART_FRSKY, USART_IT_TXE);
			if (Send_count_Frsky != is_Frsky)
			{
				USART_SendData(UART_FRSKY, Send_buf_Frsky[Send_count_Frsky]);
				Send_count_Frsky++;
				return;
			}
			else
			{
			  Send_count_Frsky = 0;
			  //USART_SendData(UART5, '\r');
			  USART_ITConfig(UART_FRSKY, USART_IT_TXE, DISABLE);
			  return;
			}
	}
	if (USART_GetITStatus(UART_FRSKY, USART_IT_RXNE) == SET)
		{
			USART_ClearITPendingBit(UART_FRSKY, USART_IT_RXNE);
		}
}


void send_str_irq_Bluetooth(void)
{
	Send_count_Bluetooth = 0;
	USART_SendData(UART_BLUETOOTH, Send_buf_Bluetooth[Send_count_Bluetooth]);//UART5
	Send_count_Bluetooth++;
	USART_ITConfig(UART_BLUETOOTH, USART_IT_TXE, ENABLE);//USART2
}
void send_str_irq_Frsky(void)
{
	Send_count_Frsky = 0;
	USART_SendData(UART_FRSKY, Send_buf_Frsky[Send_count_Frsky]);//UART5
	Send_count_Frsky++;
	USART_ITConfig(UART_FRSKY, USART_IT_TXE, ENABLE);//UART5
}
void Parcel_to_comp(float q0, float q1, float q2, float q3, float x, float y, float z, float volt, float curr)
    {
//	    if(flag_otpravki_quatr == 1)
//	    {
	    	sprintf(Send_buf_Bluetooth, "");
	    	is_Bluetooth = 0;
	    	uart_tx.f = q0;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[3]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[2]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[1]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[0]; is_Bluetooth++;
	    	uart_tx.f = q1;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[3]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[2]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[1]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[0]; is_Bluetooth++;
	    	uart_tx.f = q2;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[3]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[2]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[1]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[0]; is_Bluetooth++;
	    	uart_tx.f = q3;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[3]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[2]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[1]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[0]; is_Bluetooth++;
	    	uart_tx.f = x;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[3]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[2]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[1]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[0]; is_Bluetooth++;
	    	uart_tx.f = y;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[3]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[2]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[1]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[0]; is_Bluetooth++;
	    	uart_tx.f = z;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[3]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[2]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[1]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[0]; is_Bluetooth++;
	    	uart_tx.f = volt;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[3]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[2]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[1]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[0]; is_Bluetooth++;
	    	uart_tx.f = curr;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[3]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[2]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[1]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = uart_tx.a[0]; is_Bluetooth++;
	    	Send_buf_Bluetooth[is_Bluetooth] = 0; is_Bluetooth++;
	    	send_str_irq_Bluetooth();
//	    	flag_otpravki_quatr = 0;
//	    }
    }
void prvParcel_to_Comp(void *pvParameters)
    {
	float q_0, q_1, q_2, q_3, x, y, z, volt, curr;
	uint8_t cnt_frsky_5, cnt_frsky_1;
	while(1)
	    {
		xSemaphoreTake(xParcel_TX_Semaphore, portMAX_DELAY);

		//защитить
		q_0 = q0;
		q_1 = q1;
		q_2 = q2;
		q_3 = q3;

//		x = UKF_POSE;
//		y = UKF_POSN;
//		z = UKF_POSD;
		x = DataUKF.posx;
		y = DataUKF.posy;
		z = DataUKF.tru_pos_z;
		volt = All_voltage;

		curr = All_Current;
		///////////
		cnt_frsky_5++;
		cnt_frsky_1++;
		if(cnt_frsky_5>=20)
		    {
			cnt_frsky_5=0;
			FrSkysendFrSky5Hz();
		    }
		if(cnt_frsky_1>=90)
		    {
			cnt_frsky_1=0;
			FrSkysendFrSky1Hz();
		    }

		Parcel_to_comp(q_0, q_1, q_2, q_3, x, y, z, volt, curr);
	    }
    }
void FrSkysendFrSky5Hz(void)
{
	is_Frsky = 0;

	addBufferData(BASEMODE, Send_buf_Frsky, &is_Frsky);
	addBufferData(TEMP1, Send_buf_Frsky, &is_Frsky);
	addBufferData(TEMP2, Send_buf_Frsky, &is_Frsky);

	addBufferData(COURSE, Send_buf_Frsky, &is_Frsky);
	addBufferData(CURRENT, Send_buf_Frsky, &is_Frsky);
	addBufferData(VOLTAGE, Send_buf_Frsky, &is_Frsky);
	addBufferData(GPSALT, Send_buf_Frsky, &is_Frsky);
	addBufferData(ALTITUDE, Send_buf_Frsky, &is_Frsky);
	addBufferData(RPM, Send_buf_Frsky, &is_Frsky);


	Send_buf_Frsky[is_Frsky] = tail_value;is_Frsky++;
	send_str_irq_Frsky();

}
void FrSkysendFrSky1Hz(void)
{
	is_Frsky = 0;

	addBufferData(GPSSPEED, Send_buf_Frsky, &is_Frsky);
	addBufferData(FUEL, Send_buf_Frsky, &is_Frsky);
	addBufferData(LATITUDE, Send_buf_Frsky, &is_Frsky);
	addBufferData(LONGITUDE, Send_buf_Frsky, &is_Frsky);

	addBufferData(GPS_HDOP, Send_buf_Frsky, &is_Frsky);
	addBufferData(HEALTH, Send_buf_Frsky, &is_Frsky);
	addBufferData(STATUS_MSG, Send_buf_Frsky, &is_Frsky);

	Send_buf_Frsky[is_Frsky] = tail_value;is_Frsky++;
	send_str_irq_Frsky();

}

int8_t lsByte(int16_t value)
{
  return ((int8_t) ((value) & 0xff));
}
int8_t msByte(int16_t value)
{
  return ((int8_t) ((value) >> 8));
}
void addBufferData(const char id, uint8_t* buf, uint8_t *cnt_buff)
{
	uint8_t cnt = *cnt_buff;
	uint8_t tmo;
	switch(id)
	    {
		case GPSALT :
		{
		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=GPSALT;cnt++;

		    tmo=lsByte((int)GPS_alt);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    tmo=msByte((int)GPS_alt);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}

		    unsigned int temp = (unsigned int)((GPS_alt - (int)GPS_alt) * 10000.0f);
		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=GPSALT + decimal;cnt++;
		    tmo=lsByte(temp);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    tmo=msByte(temp);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    *cnt_buff = cnt;
		}
		break;
		case FUEL :
		{
		    // Battery remaining in %
		    int fuelLevel = (int16_t)All_percent_cap;
		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=FUEL;cnt++;
		    tmo=lsByte(fuelLevel);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    tmo=msByte(fuelLevel);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    *cnt_buff = cnt;
		}
		break;
		case BASEMODE :
		{
		    // APM base mode bitfield
		    //int base_mode = dataProvider->getBaseMode();
		    int base_mode;
		    if (supervisorState&STATE_ARMED) base_mode = 128; else base_mode = 0;
		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=BASEMODE;cnt++;
		    tmo=lsByte(base_mode);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    tmo=msByte(base_mode);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    *cnt_buff = cnt;
		}
		break;
		case TEMP1 :
		{
		    // APM mode
		    //int temp1 = dataProvider->getTemp1();
		    int temp1=(int)Mode;
		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=TEMP1;cnt++;
		    tmo=lsByte(temp1);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    tmo=msByte(temp1);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    *cnt_buff = cnt;
		}
		break;
		case TEMP2 :
		{
		    // GPS status, number of satelites in view
		    //int value = dataProvider->getTemp2();
//		    uint8_t reggp;
//		    if ((GPS_satellits>=3)&&(GPS_satellits<=5)) reggp = 2;
//		    if (GPS_satellits>5) reggp = 3;
//		    if (GPS_satellits<3) reggp = 1;
		    int value = GPS_satellits*10+GPS_fix_type;
		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=TEMP2;cnt++;
		    tmo=lsByte(value);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    tmo=msByte(value);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    *cnt_buff = cnt;
		}
		break;
		case ALTITUDE :
		{
			// Altitude in cm minus Home altitude in cm
			// Altitude in Taranis is offset by -10 m
		    float altitude = DataUKF.tru_pos_z;
		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=ALTITUDE;cnt++;
		    tmo=lsByte((int)altitude);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    tmo=msByte((int)altitude);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}

		    unsigned int temp;
		    if ( altitude > 0 ) {
			temp = (unsigned int)((altitude - (unsigned int)altitude) * 100.0f);
		    } else if ( (altitude <= 0) && (altitude > -1) ) {
			temp = 0; // FrSky bug for values from 0.0 to -0.99
		    } else {
			temp = (unsigned int)((-altitude + (int)altitude) * 100.0f);
		    }

		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=ALTIDEC;cnt++;
		    tmo=lsByte(temp);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    tmo=msByte(temp);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    *cnt_buff = cnt;

		}
		break;
		case GPSSPEED :
		{
		    // GPS Ground speed in knots
		    // Seems like there is an offset of 1.84 for some reason
		    //int gpsSpeed  = (int16_t)GPS_vel; // / 1.84f;
		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=GPSSPEED;cnt++;
		    tmo=lsByte((int)GPS_vel);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    tmo=msByte((int)GPS_vel);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    *cnt_buff = cnt;

		    unsigned int temp = (unsigned int)((GPS_vel - (int)GPS_vel) * 1000.0f);

		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=GPSSPEED + decimal;cnt++;
		    tmo=lsByte(temp);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    tmo=msByte(temp);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    *cnt_buff = cnt;
		}
		break;
		case LATITUDE :
		{
			//float gpsLatitude = gpsDdToDmsFormat(termToDecimal(4) / 10000000.0f);
		    float gpsLatitude = (float)GPS_LA;
		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=LATITUDE;cnt++;
		    tmo=lsByte((int)gpsLatitude);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    tmo=msByte((int)gpsLatitude);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}

		    unsigned int temp = (unsigned int)((gpsLatitude - (int)gpsLatitude) * 10000.0f);

		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=LATITUDE + decimal;cnt++;
		    tmo=lsByte(temp);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    tmo=msByte(temp);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}

		    char northSouth = (gpsLatitude < 0) ? 'S' : 'N';

		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=NORTHSOUTH;cnt++;
		    buf[cnt]=northSouth;cnt++;
		    buf[cnt]=0;cnt++;

		    *cnt_buff = cnt;
		}
		break;
		case LONGITUDE :
		{
			//float gpsLongitude = gpsDdToDmsFormat(termToDecimal(5) / 10000000.0f);
		    float gpsLongitude = (float)GPS_LO;
		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=LONGITUDE;cnt++;
		    tmo=lsByte((int)gpsLongitude);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    tmo=msByte((int)gpsLongitude);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}

		    unsigned int temp = (unsigned int)((gpsLongitude - (int)gpsLongitude) * 10000.0f);

		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=LONGITUDE + decimal;cnt++;
		    tmo=lsByte(temp);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    tmo=msByte(temp);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}

		    char eastWest = (gpsLongitude < 0) ? 'W' : 'E';

		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=EASTWEST;cnt++;
		    buf[cnt]=eastWest;cnt++;
		    buf[cnt]=0;cnt++;

		    *cnt_buff = cnt;
		}
		break;
		case COURSE :
		{
		    //float course = (par->termToDecimal(14) / 100.0f); // Course in 1/100 degree
		    float yaw_real;
		    yaw_real = -atan2f((q1*q2-q0*q3),(q0*q0+q2*q2-0.5f));
		    float course = (yaw_start-yaw_real)*RAD2DEG;//курс относительно старта!
		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=COURSE;cnt++;
		    tmo=lsByte((int)course);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    tmo=msByte((int)course);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}

		    unsigned int temp = (unsigned int)((course - (int)course) * 1000.0f);

		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=COURSE + decimal;cnt++;
		    tmo=lsByte(temp);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    tmo=msByte(temp);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}

		    *cnt_buff = cnt;
		}
		break;

		case CURRENT :
		{
			//float current = par->termToDecimal(1) / 1000.0f; // 10.0f -> 1A
			int current = (int)(All_Current*10.0f); //  12 - 1.2A;
			buf[cnt]=header_value;cnt++;
			buf[cnt]=CURRENT;cnt++;
			tmo=lsByte((int)current);
			if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
			if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
			if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
			tmo=msByte((int)current);
			if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
			if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
			if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}

			*cnt_buff = cnt;
		}
		break;

		case VOLTAGE :
		{
		    //float batteryVoltage = par->termToDecimal(0) * 0.5238f;
		    float batteryVoltage = All_voltage* 0.5238f;
		    //float batteryVoltage = All_voltage/1000.0f;
		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=VOLTAGE;cnt++;
		    tmo=lsByte((int)batteryVoltage);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    tmo=msByte((int)batteryVoltage);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}

		    unsigned int temp = (unsigned int)((batteryVoltage - (int)batteryVoltage) * 10.0f);

		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=VOLTAGEDEC;cnt++;
		    tmo=lsByte(temp);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
		    tmo=msByte(temp);
		    if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
		    if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
		    if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}

		    *cnt_buff = cnt;
		}
		break;
		case HEALTH :
		{
		    unsigned int sensors_health = 0;
		    if(Err_Mag)sensors_health|=3;
		    if(Err_MPU)sensors_health|=12;


		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=HEALTH;cnt++;
		    buf[cnt]=sensors_health;cnt++;
		    buf[cnt]=0;cnt++;

		    *cnt_buff = cnt;
		}
		break;
//		case STATUS_MSG :
//		{
//			unsigned int msg_stat = 0;
//			Send_buf_Frsky[is_Frsky] = header_value;
//			Send_buf_Frsky[is_Frsky + 1] = STATUS_MSG;
//			Send_buf_Frsky[is_Frsky + 2] = lsByte(msg_stat);
//			Send_buf_Frsky[is_Frsky + 3] = msByte(msg_stat);
//
//			return 4;
//		}
//		break;
//		case HOME_DIR :
//		{
//			unsigned int home_direction = (int16_t)atan2f(Y_GPS, X_GPS);
//			Send_buf_Frsky[is_Frsky] = header_value;
//			Send_buf_Frsky[is_Frsky + 1] = HOME_DIR;
//			Send_buf_Frsky[is_Frsky + 2] = lsByte(home_direction);
//			Send_buf_Frsky[is_Frsky + 3] = msByte(home_direction);
//
//			return 4;
//		}
//		break;
//		case HOME_DIST :
//		{
//			unsigned int home_distance = (int16_t)sqrtf(X_GPS*X_GPS+Y_GPS*Y_GPS);
//			Send_buf_Frsky[is_Frsky] = header_value;
//			Send_buf_Frsky[is_Frsky + 1] = HOME_DIST;
//			Send_buf_Frsky[is_Frsky + 2] = lsByte(home_distance);
//			Send_buf_Frsky[is_Frsky + 3] = msByte(home_distance);
//
//			return 4;
//		}
//		break;
//		case CPU_LOAD :
//		{
//			unsigned int cpu_load = 0;
//			Send_buf_Frsky[is_Frsky] = header_value;
//			Send_buf_Frsky[is_Frsky + 1] = CPU_LOAD;
//			Send_buf_Frsky[is_Frsky + 2] = lsByte(cpu_load);
//			Send_buf_Frsky[is_Frsky + 3] = msByte(cpu_load);
//
//			return 4;
//		}
//		break;
		case RPM :
		{
			//float current = par->termToDecimal(1) / 1000.0f; // 10.0f -> 1A
			int thr = (int)(throtle_ppm*100.0f); //  12 - 1.2A;
			buf[cnt]=header_value;cnt++;
			buf[cnt]=RPM;cnt++;
			tmo=lsByte((int)thr);
			if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
			if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
			if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}
			tmo=msByte((int)thr);
			if (tmo==header_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3E;cnt++;}
			if (tmo==escape_value) { buf[cnt]=0x5D;cnt++;buf[cnt]=0x3D;cnt++;}
			if((tmo!=header_value)&&(tmo!=escape_value))  { buf[cnt]=tmo;cnt++;}

			*cnt_buff = cnt;
		}
		break;
		case GPS_HDOP :
		{
		    unsigned int gpsHdop = GPS_hAccf > 99.9f ? 9990 :(int)(GPS_hAccf*100.0f);
		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=GPS_HDOP;cnt++;
		    buf[cnt]=lsByte(gpsHdop);cnt++;
		    buf[cnt]=msByte(gpsHdop);cnt++;
		    *cnt_buff = cnt;
		}
		break;
		case STATUS_MSG:
		{
		    unsigned int sts;
		    if (compass_not_health == 1) sts=4;
		    if (baro_not_health == 1) sts=3;
		    buf[cnt]=header_value;cnt++;
		    buf[cnt]=STATUS_MSG ;cnt++;
		    buf[cnt]=lsByte(sts);cnt++;
		    buf[cnt]=msByte(sts);cnt++;
		    *cnt_buff = cnt;
		}
		break;
		default : break;

  }

}

