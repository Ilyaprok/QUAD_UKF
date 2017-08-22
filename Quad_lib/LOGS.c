
#include "Config.h"
#include "stm32f4xx_conf.h"


#include "ff.h"
#include "spi_sd.h"

#include "LOGS.h"


#include "MPU6500.h"
#include "IMU_INS.h"
#include "I2C_BARO_MAG.h"

#include "GPS.h"
#include "ADC.h"
#include "PWM_PPM.h"

#include "PID.h"


#define CS_ENABLE()         		GPIO_SetBits(GPIOE, GPIO_Pin_15);
#define CS_DISABLE()    	  	GPIO_ResetBits(GPIOE, GPIO_Pin_15);

#define PERIOD_LOGS_MAKE_NEW_FILE1		100*1000
#define PERIOD_LOGS_MAKE_NEW_FILE2		400*1000
#define PERIOD_LOGS_MAKE_NEW_FILE3		1000*1000

uint16_t Num_of_files1,Num_of_files2,Num_of_files, Num_of_dir;

BYTE buf_pwm[SIZE_OF_BUFFER_SD_PWM];
UINT spi_sd_count_pwm = 0, spi_sd_count_pwm_fix = 0;
BYTE buf_ppm[SIZE_OF_BUFFER_SD_PPM];
UINT spi_sd_count_ppm = 0, spi_sd_count_ppm_fix = 0;
BYTE buf_ADC_res[SIZE_OF_BUFFER_SD_ADC];
UINT spi_sd_count_ADC = 0, spi_sd_count_ADC_fix = 0;
BYTE buf_baro_mag[SIZE_OF_BUFFER_SD_BARO_MAG];
UINT spi_sd_count_baro_mag = 0, spi_sd_count_baro_mag_fix = 0;
BYTE buf_gps[SIZE_OF_BUFFER_SD_GPS];
UINT spi_sd_count_gps = 0, spi_sd_count_gps_fix = 0;

BYTE buf_ukf[SIZE_OF_BUFFER_SD_UKF];
UINT spi_sd_count_ukf = 0, spi_sd_count_ukf_fix = 0;
BYTE buf_pid[SIZE_OF_BUFFER_SD_PID];
UINT spi_sd_count_pid = 0, spi_sd_count_pid_fix = 0;
BYTE buf_flags[SIZE_OF_BUFFER_SD_FLAGS];
UINT spi_sd_count_flags = 0, spi_sd_count_flags_fix = 0;



FRESULT err_mount, err_mkdir, err_chkdir, err_opdir, err_read, err_write, err_num, err_write_dan, err_sync;

FIL file_pwm, file_ppm, file_adc, file_baro_mag, file_gps, file_ukf, file_pid, file_flags;
UINT  nWritten_pwm, nWritten_ppm, nWritten_adc, nWritten_baro_mag, nWritten_gps, nWritten_ukf, nWritten_pid, nWritten_flags;

FATFS FATFS_Obj;
DIR dir;
FILINFO fileInfo, dirInfo;
BYTE Dir_Name[10] = "Полет";

BYTE File_Name_pwm[11] = "PWM";
BYTE File_Name_ppm[11] = "PPM";
BYTE File_Name_adc[11] = "АЦП";
BYTE File_Name_baro_mag[15] = "BARO&MAG";
BYTE File_Name_gps[11] = "GPS";
BYTE File_Name_ukf[11] = "UKF";
BYTE File_Name_pid[11] = "PID";
BYTE File_Name_flags[11] = "Флаги";


uint32_t counter_sys_logs_file_close1, counter_sys_logs_file_close2, counter_sys_logs_file_close3;

char buf_num_of_dir[10], buf_num_of_files[10];
uint32_t cnt_cycle_sd_write, cnt_ticks_sd_write;

typedef union
{
float f;
unsigned char a [sizeof (float)];
} bd_float2;
bd_float2 SD_ff;
/*************FREERTOS**********/

//SemaphoreHandle_t xSD_INS_transiver_Semaphore = NULL;



SemaphoreHandle_t xSD_PPM_collect_Semaphore=NULL;
SemaphoreHandle_t xSD_PWM_collect_Semaphore=NULL;
SemaphoreHandle_t xSD_ADC_collect_Semaphore=NULL;
SemaphoreHandle_t xSD_MAG_BARO_collect_Semaphore=NULL;
SemaphoreHandle_t xSD_GPS_collect_Semaphore=NULL;
SemaphoreHandle_t xSD_UKF_collect_Semaphore=NULL;
SemaphoreHandle_t xSD_PID_collect_Semaphore=NULL;
SemaphoreHandle_t xSD_FLAGS_collect_Semaphore=NULL;

TaskHandle_t  sd_trans_handle;
uint32_t  ovf_sd_trans_stack;

TaskHandle_t sd_sbor_pwm_handle;
uint32_t ovf_sd_sbor_pwm_stack;

TaskHandle_t sd_sbor_ppm_handle;
uint32_t ovf_sd_sbor_ppm_stack;

TaskHandle_t sd_sbor_adc_handle;
uint32_t ovf_sd_sbor_adc_stack;

TaskHandle_t sd_sbor_mag_baro_handle;
uint32_t ovf_sd_sbor_mag_baro_stack;

TaskHandle_t sd_sbor_gps_handle;
uint32_t ovf_sd_sbor_gps_stack;

TaskHandle_t sd_sbor_ukf_handle;
uint32_t ovf_sd_sbor_ukf_stack;

TaskHandle_t sd_sbor_pid_handle;
uint32_t ovf_sd_sbor_pid_stack;

TaskHandle_t sd_sbor_flags_handle;
uint32_t ovf_sd_sbor_flags_stack;

////////////////////////////////

void itoa_(int n, char s[])
 {
     int i, sign;

     if ((sign = n) < 0)  /* записываем знак */
	 n = -n;          /* делаем n положительным числом */
     i = 0;
     do {       /* генерируем цифры в обратном порядке */
	 s[i++] = n % 10 + '0';   /* берем следующую цифру */
     } while ((n /= 10) > 0);     /* удаляем */
     if (sign < 0)
	 s[i++] = '-';
     s[i] = '\0';
     reverse(s);
 }
void reverse(char s[])
{
    int i, j;
    char c;

    for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
	c = s[i];
	s[i] = s[j];
	s[j] = c;
    }
}
void float_to_buf(float ch, uint8_t* buf, uint16_t *cnt_buff, uint16_t size)
    {

	uint16_t cnt = *cnt_buff;
	buf[cnt] = *((uint8_t*)&ch+3);cnt++;
	if (cnt==size) cnt = 0;
	buf[cnt] = *((uint8_t*)&ch+2);cnt++;
	if (cnt==size) cnt = 0;
	buf[cnt] = *((uint8_t*)&ch+1);cnt++;
	if (cnt==size) cnt = 0;
	buf[cnt] = *((uint8_t*)&ch);cnt++;
	if (cnt==size) cnt = 0;
	*cnt_buff = cnt;
    }
void uint32_t_to_buf (uint32_t ch, BYTE* buf_sd, UINT *cnt_buff, uint16_t size)
    {
	UINT cnt = *cnt_buff;
	buf_sd[cnt] = (BYTE)(ch>>24);cnt++;
	if (cnt==size) cnt = 0;
	buf_sd[cnt] = (BYTE)(ch>>16);cnt++;
	if (cnt==size) cnt = 0;
	buf_sd[cnt] = (BYTE)(ch>>8);cnt++;
	if (cnt==size) cnt = 0;
	buf_sd[cnt] = (BYTE)(ch);cnt++;
	if (cnt==size) cnt = 0;
	*cnt_buff = cnt;
    }
void uint16_t_to_buf (uint16_t ch, BYTE* buf_sd, UINT *cnt_buff, uint16_t size)
    {
	UINT cnt = *cnt_buff;
	buf_sd[cnt] = (BYTE)(ch>>8);cnt++;
	if (cnt==size) cnt = 0;
	buf_sd[cnt] = (BYTE)(ch);cnt++;
	if (cnt==size) cnt = 0;
	*cnt_buff = cnt;
    }
void SPI_SD_Init(void)
{

	//PB13 - SCK
	//PB15 - MOSI
	//PB16 - MISO
	//PE15 - CS

	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;

	/* подадим такт на порт */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOE, ENABLE);
	/* настройка выводов выбора CS */
	GPIO_InitStruct.GPIO_Pin 	= GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	CS_ENABLE();
	/* настройка выводов SPI2 */
	GPIO_InitStruct.GPIO_Pin 	= GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd 	= GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	/* Соединим выводы портов с GPIO_AF_SPI2 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	/* подадим такт на SPI2 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	/* настроим SPI Master */
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_Init(SPI2, &SPI_InitStruct);

	SPI_NSSInternalSoftwareConfig(SPI2, SPI_NSSInternalSoft_Set);
	SPI_Cmd(SPI2,ENABLE);

	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	//SPI_I2S_ITConfig(SPI2, SPI_IT_TXE, ENABLE);
}
void Init_logs(void)
    {

	vSemaphoreCreateBinary(xSD_PPM_collect_Semaphore);
	vSemaphoreCreateBinary(xSD_PWM_collect_Semaphore);
	vSemaphoreCreateBinary(xSD_ADC_collect_Semaphore);
	vSemaphoreCreateBinary(xSD_MAG_BARO_collect_Semaphore);
	vSemaphoreCreateBinary(xSD_GPS_collect_Semaphore);
	vSemaphoreCreateBinary(xSD_UKF_collect_Semaphore);
	vSemaphoreCreateBinary(xSD_PID_collect_Semaphore);
	vSemaphoreCreateBinary(xSD_FLAGS_collect_Semaphore);

	xTaskCreate(prvSbor_buf_ppm,(signed char*)"Collect_SD_PPM", 100, NULL, PRIORITET_TASK_COLLECT_SD_PPM, sd_sbor_ppm_handle);
	xTaskCreate(prvSbor_buf_pwm,(signed char*)"Collect_SD_PWM", 100, NULL, PRIORITET_TASK_COLLECT_SD_PWM, sd_sbor_pwm_handle);
	xTaskCreate(prvSbor_buf_adc,(signed char*)"Collect_SD_ADC", 100, NULL, PRIORITET_TASK_COLLECT_SD_ADC, sd_sbor_adc_handle);
	xTaskCreate(prvSbor_buf_baro_mag,(signed char*)"Collect_SD_BARO_MAG", 100, NULL, PRIORITET_TASK_COLLECT_SD_MAG_BARO, sd_sbor_mag_baro_handle);
	xTaskCreate(prvSbor_buf_gps,(signed char*)"Collect_SD_GPS", 100, NULL, PRIORITET_TASK_COLLECT_SD_GPS, sd_sbor_gps_handle);
	xTaskCreate(prvSbor_buf_ukf,(signed char*)"Collect_SD_UKF", 100, NULL, PRIORITET_TASK_COLLECT_SD_UKF, sd_sbor_ukf_handle);
	xTaskCreate(prvSbor_buf_pid,(signed char*)"Collect_SD_PID", 100, NULL, PRIORITET_TASK_COLLECT_SD_PID, sd_sbor_pid_handle);
	xTaskCreate(prvSbor_buf_flags1,(signed char*)"Collect_SD_FLAGS", 150, NULL, PRIORITET_TASK_COLLECT_SD_FLAGS, sd_sbor_flags_handle);

	xTaskCreate(prvTransfer_buf,(signed char*)"Trans_SD_logs", 170, NULL, PRIORITET_TASK_TRANSIVER_SD, sd_trans_handle);

	SPI_SD_Init();
	disk_initialize(0);
	err_mount = f_mount(0, &FATFS_Obj);
	err_opdir = f_opendir(&dir, "");
	while (((err_num = f_readdir(&dir, &dirInfo)) == FR_OK) && dirInfo.fname[0])
	    {
		Num_of_dir++;
	    }
	Num_of_dir +=1;
	Num_of_files = 1;

	Num_of_files1 = 1;
	Num_of_files2 = 1;


	itoa_(Num_of_dir, buf_num_of_dir);
	strcat(Dir_Name, buf_num_of_dir);

	err_mkdir = f_mkdir (Dir_Name); //создаем директорию
	err_chkdir = f_chdir (Dir_Name);
	//err_opdir = f_opendir(&dir, Dir_Name);

	itoa_(Num_of_files, buf_num_of_files);
	strcat(File_Name_pwm, buf_num_of_files);
	strcat(File_Name_ppm, buf_num_of_files);
	strcat(File_Name_adc, buf_num_of_files);
	strcat(File_Name_baro_mag, buf_num_of_files);
	strcat(File_Name_gps, buf_num_of_files);
	strcat(File_Name_ukf, buf_num_of_files);
	strcat(File_Name_pid, buf_num_of_files);
	strcat(File_Name_flags, buf_num_of_files);

	strcat(File_Name_pwm, ".log");
	strcat(File_Name_ppm, ".log");
	strcat(File_Name_adc, ".log");
	strcat(File_Name_baro_mag, ".log");
	strcat(File_Name_gps, ".log");
	strcat(File_Name_ukf, ".log");
	strcat(File_Name_pid, ".log");
	strcat(File_Name_flags, ".log");

	if ((err_write = f_open(&file_pwm, &File_Name_pwm, FA_CREATE_ALWAYS | FA_WRITE)) == FR_OK)
	{
		init_log_file_pwm();
	}
	if ((err_write = f_open(&file_ppm, &File_Name_ppm, FA_CREATE_ALWAYS | FA_WRITE)) == FR_OK)
	{
		init_log_file_ppm();
	}
	if ((err_write = f_open(&file_adc, &File_Name_adc, FA_CREATE_ALWAYS | FA_WRITE)) == FR_OK)
	{
		init_log_file_adc();
	}
	if ((err_write = f_open(&file_baro_mag, &File_Name_baro_mag, FA_CREATE_ALWAYS | FA_WRITE)) == FR_OK)
	{
		init_log_file_baro_mag();
	}
	if ((err_write = f_open(&file_gps, &File_Name_gps, FA_CREATE_ALWAYS | FA_WRITE)) == FR_OK)
	{
		init_log_file_gps();
	}
	if ((err_write = f_open(&file_ukf, &File_Name_ukf, FA_CREATE_ALWAYS | FA_WRITE)) == FR_OK)
	{
		init_log_file_ukf();
	}
	if ((err_write = f_open(&file_pid, &File_Name_pid, FA_CREATE_ALWAYS | FA_WRITE)) == FR_OK)
	{
		init_log_file_pid();
	}
	if ((err_write = f_open(&file_flags, &File_Name_flags, FA_CREATE_ALWAYS | FA_WRITE)) == FR_OK)
	{
		init_log_file_flags();
	}

    }

void close_op_new_file(void)
    {
	if (counter_sys-counter_sys_logs_file_close1 >=PERIOD_LOGS_MAKE_NEW_FILE1)
	    {
		counter_sys_logs_file_close1 = counter_sys;
		f_close(&file_pid);
		Num_of_files1++;
		sprintf(File_Name_pid, "PID");

		sprintf(buf_num_of_files, "");
		itoa_(Num_of_files1, buf_num_of_files);
		strcat(File_Name_pid, buf_num_of_files);

		strcat(File_Name_pid, ".log");

		nWritten_pid = 0;

		if ((err_write = f_open(&file_pid, &File_Name_pid, FA_CREATE_ALWAYS|FA_WRITE)) == FR_OK)
		{
			init_log_file_pid();
		}
	    }
	if (counter_sys-counter_sys_logs_file_close2 >=PERIOD_LOGS_MAKE_NEW_FILE2)
	    {
		counter_sys_logs_file_close2 = counter_sys;
		f_close(&file_pwm);
		f_close(&file_ukf);

		Num_of_files2++;
		sprintf(File_Name_pwm, "PWM");
		sprintf(File_Name_ukf, "UKF");

		sprintf(buf_num_of_files, "");
		itoa_(Num_of_files2, buf_num_of_files);

		strcat(File_Name_pwm, buf_num_of_files);
		strcat(File_Name_ukf, buf_num_of_files);



		strcat(File_Name_pwm, ".log");
		strcat(File_Name_ukf, ".log");
		nWritten_pwm = 0;
		nWritten_ukf = 0;

		if ((err_write = f_open(&file_pwm, &File_Name_pwm, FA_CREATE_ALWAYS|FA_WRITE)) == FR_OK)
		{
			init_log_file_pwm();
		}
		if ((err_write = f_open(&file_ukf, &File_Name_ukf, FA_CREATE_ALWAYS|FA_WRITE)) == FR_OK)
		{
			init_log_file_ukf();
		}
	    }
	if (counter_sys-counter_sys_logs_file_close3 >=PERIOD_LOGS_MAKE_NEW_FILE3)
	    {
		counter_sys_logs_file_close3 = counter_sys;

		f_close(&file_ppm);
		f_close(&file_baro_mag);
		f_close(&file_gps);
		f_close(&file_flags);
		f_close(&file_adc);

		Num_of_files++;

		sprintf(File_Name_ppm, "PPM");
		sprintf(File_Name_baro_mag, "BARO&MAG");
		sprintf(File_Name_gps, "GPS");
		sprintf(File_Name_flags, "Флаги");
		sprintf(File_Name_adc, "АЦП");


		sprintf(buf_num_of_files, "");
		itoa_(Num_of_files, buf_num_of_files);

		strcat(File_Name_ppm, buf_num_of_files);
		strcat(File_Name_baro_mag, buf_num_of_files);
		strcat(File_Name_gps, buf_num_of_files);
		strcat(File_Name_flags, buf_num_of_files);
		strcat(File_Name_adc, buf_num_of_files);



		strcat(File_Name_ppm, ".log");
		strcat(File_Name_baro_mag, ".log");
		strcat(File_Name_gps, ".log");
		strcat(File_Name_flags, ".log");
		strcat(File_Name_adc, ".log");


		nWritten_ppm = 0;
		nWritten_baro_mag = 0;
		nWritten_gps = 0;
		nWritten_flags = 0;
		nWritten_adc = 0;


		if ((err_write = f_open(&file_ppm, &File_Name_ppm, FA_CREATE_ALWAYS|FA_WRITE)) == FR_OK)
		{
			init_log_file_ppm();
		}
		if ((err_write = f_open(&file_baro_mag, &File_Name_baro_mag, FA_CREATE_ALWAYS|FA_WRITE)) == FR_OK)
		{
			init_log_file_baro_mag();
		}
		if ((err_write = f_open(&file_gps, &File_Name_gps, FA_CREATE_ALWAYS|FA_WRITE )) == FR_OK)
		{
			init_log_file_gps();
		}
		if ((err_write = f_open(&file_flags, &File_Name_flags, FA_CREATE_ALWAYS|FA_WRITE )) == FR_OK)
		{
			init_log_file_flags();
		}
		if ((err_write = f_open(&file_adc, &File_Name_adc, FA_CREATE_ALWAYS|FA_WRITE )) == FR_OK)
		{
			init_log_file_adc();
		}

	    }
    }

void init_log_file_pwm(void)
    {
	uint8_t zagalovki_pwm[]="Логи PWM\r\n";
	f_write(&file_pwm, zagalovki_pwm,sizeof(zagalovki_pwm)-1, &nWritten_pwm);
	f_sync(&file_pwm);
	uint8_t opisanie[10];
	uint8_t cnt_simv = 0;
	opisanie[cnt_simv]=6;cnt_simv++;
	opisanie[cnt_simv]=TIME_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=CYCLE_TIME_PWM_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=M1_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=M2_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=M3_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=M4_LOG_ID;cnt_simv++;
//	opisanie[cnt_simv]=ALL_CURR_LOG_ID;cnt_simv++;
//	opisanie[cnt_simv]=ALL_VOLT_LOG_ID;cnt_simv++;
//	opisanie[cnt_simv]=ALL_POWR_LOG_ID;cnt_simv++;
//	opisanie[cnt_simv]=ALL_ACTN_LOG_ID;cnt_simv++;
//	opisanie[cnt_simv]=ALL_VOLT_SM_LOG_ID;cnt_simv++;
	f_write(&file_pwm, opisanie,cnt_simv, &nWritten_pwm);
	f_sync(&file_pwm);
    }
void init_log_file_adc(void)
    {
	uint8_t zagalovki_adc[]="Логи токов и напряжений\r\n";
	f_write(&file_adc, zagalovki_adc,sizeof(zagalovki_adc)-1, &nWritten_adc);
	f_sync(&file_adc);
	uint8_t opisanie[15];
	uint8_t cnt_simv = 0;
	opisanie[cnt_simv]=12;cnt_simv++;
	opisanie[cnt_simv]=TIME_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=CYCLE_TIME_ADC_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=ALL_CURR_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=ALL_VOLT_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=ALL_POWR_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=ALL_ACTN_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=ALL_VOLT_SM_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=ALL_PERC_CAP_LOG_ID;cnt_simv++;

	opisanie[cnt_simv]=CURRENT1_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=CURRENT2_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=CURRENT3_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=CURRENT4_LOG_ID;cnt_simv++;
	f_write(&file_adc, opisanie,cnt_simv, &nWritten_adc);
	f_sync(&file_adc);
    }
void init_log_file_ppm(void)
    {
	uint8_t zagalovki_ppm[]="Логи PPM\r\n";
	f_write(&file_ppm, zagalovki_ppm,sizeof(zagalovki_ppm)-1, &nWritten_ppm);
	f_sync(&file_ppm);
	uint8_t opisanie[12];
	uint8_t cnt_simv = 0;
	opisanie[cnt_simv]=10;cnt_simv++;
	opisanie[cnt_simv]=TIME_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=CYCLE_TIME_PPM_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=YAW_PPM_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=PITCH_PPM_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=THROTTLE_PPM_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=ROLL_PPM_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=MODE_PPM_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=MODE2_PPM_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=MODE3_PPM_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=OFF_POW_PPM_LOG_ID;cnt_simv++;
	f_write(&file_ppm, opisanie,cnt_simv, &nWritten_ppm);
	f_sync(&file_ppm);
    }
void init_log_file_baro_mag(void)
    {
	uint8_t zagalovki_baro_mag[]="Логи BARO & MAG\r\n";
	f_write(&file_baro_mag, zagalovki_baro_mag,sizeof(zagalovki_baro_mag)-1, &nWritten_baro_mag);
	f_sync(&file_baro_mag);
	uint8_t opisanie[15];
	uint8_t cnt_simv = 0;
	opisanie[cnt_simv]=11;cnt_simv++;
	opisanie[cnt_simv]=TIME_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=CYCLE_TIME_BARO_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=CYCLE_TIME_MAG_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=MX_RAW_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=MY_RAW_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=MZ_RAW_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=ALT_BARO_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=TEMP_BARO_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=TEMP_GYRO_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=UKF_POSZ_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GPS_ALT_LOG_ID;cnt_simv++;

	f_write(&file_baro_mag, opisanie,cnt_simv, &nWritten_baro_mag);
	f_sync(&file_baro_mag);
    }
void init_log_file_gps(void)
    {
	uint8_t zagalovki_gps[]="Логи GPS\r\n";
	f_write(&file_gps, zagalovki_gps,sizeof(zagalovki_gps)-1, &nWritten_gps);
	f_sync(&file_gps);
	uint8_t opisanie[25];
	uint8_t cnt_simv = 0;
	opisanie[cnt_simv]=21;cnt_simv++;
	opisanie[cnt_simv]=TIME_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=CYCLE_TIME_GPS_LOG_ID;cnt_simv++;

	opisanie[cnt_simv]=GPS_SATTELITS_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GPS_START_HOME_LOG_ID;cnt_simv++;

	opisanie[cnt_simv]=GPS_X_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GPS_Y_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GPS_ALT_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GPS_VEL_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GPS_COURSE_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GPS_X_SPEED_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GPS_Y_SPEED_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GPS_Z_SPEED_LOG_ID;cnt_simv++;

	opisanie[cnt_simv]=GPS_LAT_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GPS_LON_LOG_ID;cnt_simv++;

	opisanie[cnt_simv]=GPS_HACC_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GPS_VACC_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GPS_SACC_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GPS_VDOP_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GPS_TDOP_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GPS_EDOP_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GPS_NDOP_LOG_ID;cnt_simv++;

	f_write(&file_gps, opisanie,cnt_simv, &nWritten_gps);
	f_sync(&file_gps);

    }
void init_log_file_pid(void)
    {
	uint8_t zagalovki_pid[]="Логи PID и RAW MPU\r\n";
	f_write(&file_pid, zagalovki_pid,sizeof(zagalovki_pid)-1, &nWritten_pid);
	f_sync(&file_pid);
	uint8_t opisanie[30];
	uint8_t cnt_simv = 0;
	opisanie[cnt_simv]=22;cnt_simv++;
	opisanie[cnt_simv]=TIME_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=CYCLE_TIME_PID_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=AX_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=AY_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=AZ_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GX_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GY_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=GZ_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=PID_U_X_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=PID_U_Y_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=PID_U_Z_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=PID_U_T_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=PID_REF_VEL_GLOB_X_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=PID_REF_VEL_GLOB_Y_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=PID_REF_ANG_GLOB_X_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=PID_REF_ANG_GLOB_Y_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=PID_REF_ANG_X_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=PID_REF_ANG_Y_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=PID_VEL_Z_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=PID_I_RATEX_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=PID_I_RATEY_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=PID_I_RATEZ_LOG_ID;cnt_simv++;
	f_write(&file_pid, opisanie,cnt_simv, &nWritten_pid);
	f_sync(&file_pid);
    }
void init_log_file_ukf(void)
    {
	uint8_t zagalovki_ukf[]="Логи UKF\r\n";
	f_write(&file_ukf, zagalovki_ukf,sizeof(zagalovki_ukf)-1, &nWritten_ukf);
	f_sync(&file_ukf);
	uint8_t opisanie[20];
	uint8_t cnt_simv = 0;
	opisanie[cnt_simv]=17;cnt_simv++;
	opisanie[cnt_simv]=TIME_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=CYCLE_TIME_UKF_LOG_ID;cnt_simv++;

	opisanie[cnt_simv]=UKF_POSX_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=UKF_POSY_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=UKF_POSZ_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=UKF_VELX_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=UKF_VELY_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=UKF_VELZ_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=UKF_ABX_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=UKF_ABY_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=UKF_ABZ_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=UKF_GBX_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=UKF_GBY_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=UKF_GBZ_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=UKF_AXISX_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=UKF_AXISY_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=UKF_AXISZ_LOG_ID;cnt_simv++;

	f_write(&file_ukf, opisanie,cnt_simv, &nWritten_ukf);
	f_sync(&file_ukf);
    }
void init_log_file_flags(void)
    {
	uint8_t zagalovki_flags[]="Логи флагов, режимов, коэффициентов, ошибок и2с\r\n";
	f_write(&file_flags, zagalovki_flags,sizeof(zagalovki_flags)-1, &nWritten_flags);
	f_sync(&file_flags);
	uint8_t opisanie[30];
	uint8_t cnt_simv = 0;
	opisanie[cnt_simv]=16;cnt_simv++;
	opisanie[cnt_simv]=TIME_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=ARM_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=BUZZER_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=ALT_WITHOUT_GPS_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=ERR_I2C_DEF_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=ERR_I2C_LINE_LOG_ID;cnt_simv++;


	opisanie[cnt_simv]=KP_PID_ALT_POS_LOG_ID;cnt_simv++;

	opisanie[cnt_simv]=KP_PID_ALT_VEL_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=KD_PID_ALT_VEL_LOG_ID;cnt_simv++;

	opisanie[cnt_simv]=KP_PID_ALT_ACC_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=KI_PID_ALT_ACC_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=KD_PID_ALT_ACC_LOG_ID;cnt_simv++;

	opisanie[cnt_simv]=F_CUT_ERR_ALT_VEL_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=F_CUT_DIFF_ALT_VEL_LOG_ID;cnt_simv++;

	opisanie[cnt_simv]=KI_PID_VEL_Z_LOG_ID;cnt_simv++;
	opisanie[cnt_simv]=KD_PID_VEL_Z_LOG_ID;cnt_simv++;

	f_write(&file_flags, opisanie,cnt_simv, &nWritten_flags);
	f_sync(&file_flags);

    }

void prvSbor_buf_pwm(void *pvParameters)
    {
	while(1)
	    {
		xSemaphoreTake(xSD_PWM_collect_Semaphore, portMAX_DELAY);

		uint32_t_to_buf(counter_sys, buf_pwm, &spi_sd_count_pwm, SIZE_OF_BUFFER_SD_PWM);
		uint32_t_to_buf(cnt_cycle_pwm, buf_pwm, &spi_sd_count_pwm, SIZE_OF_BUFFER_SD_PWM);

		uint16_t_to_buf(M1_lev_front, buf_pwm, &spi_sd_count_pwm, SIZE_OF_BUFFER_SD_PWM);
		uint16_t_to_buf(M2_prav_back, buf_pwm, &spi_sd_count_pwm, SIZE_OF_BUFFER_SD_PWM);
		uint16_t_to_buf(M3_prav_front, buf_pwm, &spi_sd_count_pwm, SIZE_OF_BUFFER_SD_PWM);
		uint16_t_to_buf(M4_lev_back, buf_pwm, &spi_sd_count_pwm, SIZE_OF_BUFFER_SD_PWM);

		ovf_sd_sbor_pwm_stack = uxTaskGetStackHighWaterMark(sd_sbor_pwm_handle);

	    }
    }
void prvSbor_buf_ppm(void *pvParameters)
    {
	while(1)
	    {
	       xSemaphoreTake(xSD_PPM_collect_Semaphore, portMAX_DELAY);

		uint32_t_to_buf(counter_sys, buf_ppm, &spi_sd_count_ppm, SIZE_OF_BUFFER_SD_PPM);
		uint32_t_to_buf(cnt_cycle_ppm, buf_ppm, &spi_sd_count_ppm, SIZE_OF_BUFFER_SD_PPM);

		uint16_t_to_buf(ppm1_buf[1], buf_ppm, &spi_sd_count_ppm, SIZE_OF_BUFFER_SD_PPM);
		uint16_t_to_buf(ppm1_buf[2], buf_ppm, &spi_sd_count_ppm, SIZE_OF_BUFFER_SD_PPM);
		uint16_t_to_buf(ppm1_buf[3], buf_ppm, &spi_sd_count_ppm, SIZE_OF_BUFFER_SD_PPM);
		uint16_t_to_buf(ppm1_buf[4], buf_ppm, &spi_sd_count_ppm, SIZE_OF_BUFFER_SD_PPM);
		uint16_t_to_buf(ppm1_buf[5], buf_ppm, &spi_sd_count_ppm, SIZE_OF_BUFFER_SD_PPM);
		uint16_t_to_buf(ppm1_buf[6], buf_ppm, &spi_sd_count_ppm, SIZE_OF_BUFFER_SD_PPM);
		uint16_t_to_buf(ppm1_buf[7], buf_ppm, &spi_sd_count_ppm, SIZE_OF_BUFFER_SD_PPM);
		uint16_t_to_buf(ppm1_buf[8], buf_ppm, &spi_sd_count_ppm, SIZE_OF_BUFFER_SD_PPM);

		ovf_sd_sbor_ppm_stack = uxTaskGetStackHighWaterMark(sd_sbor_ppm_handle);

	    }
    }
void prvSbor_buf_adc(void *pvParameters)
    {
	while(1)
	    {
		xSemaphoreTake(xSD_ADC_collect_Semaphore, portMAX_DELAY);

		uint32_t_to_buf(counter_sys, buf_ADC_res, &spi_sd_count_ADC, SIZE_OF_BUFFER_SD_ADC);
		uint32_t_to_buf(cnt_cycle_adc, buf_ADC_res, &spi_sd_count_ADC, SIZE_OF_BUFFER_SD_ADC);

		float_to_buf(All_Current, buf_ADC_res, &spi_sd_count_ADC, SIZE_OF_BUFFER_SD_ADC);
		float_to_buf(All_voltage, buf_ADC_res, &spi_sd_count_ADC, SIZE_OF_BUFFER_SD_ADC);
		float_to_buf(All_power, buf_ADC_res, &spi_sd_count_ADC, SIZE_OF_BUFFER_SD_ADC);
		float_to_buf(All_action, buf_ADC_res, &spi_sd_count_ADC, SIZE_OF_BUFFER_SD_ADC);
		float_to_buf(All_voltage_smooth, buf_ADC_res, &spi_sd_count_ADC, SIZE_OF_BUFFER_SD_ADC);
		float_to_buf(All_percent_cap, buf_ADC_res, &spi_sd_count_ADC, SIZE_OF_BUFFER_SD_ADC);

		//свободные переменные
		float_to_buf(0.0f, buf_ADC_res, &spi_sd_count_ADC, SIZE_OF_BUFFER_SD_ADC);
		float_to_buf(0.0f, buf_ADC_res, &spi_sd_count_ADC, SIZE_OF_BUFFER_SD_ADC);
		float_to_buf(0.0f, buf_ADC_res, &spi_sd_count_ADC, SIZE_OF_BUFFER_SD_ADC);
		float_to_buf(0.0f, buf_ADC_res, &spi_sd_count_ADC, SIZE_OF_BUFFER_SD_ADC);

		ovf_sd_sbor_adc_stack = uxTaskGetStackHighWaterMark(sd_sbor_adc_handle);

	    }
    }
void prvSbor_buf_baro_mag(void *pvParameters)
    {
	while(1)
	    {
		xSemaphoreTake(xSD_MAG_BARO_collect_Semaphore, portMAX_DELAY);


		uint32_t_to_buf(counter_sys, buf_baro_mag, &spi_sd_count_baro_mag, SIZE_OF_BUFFER_SD_BARO_MAG);
		uint32_t_to_buf(cnt_cycle_baro, buf_baro_mag, &spi_sd_count_baro_mag, SIZE_OF_BUFFER_SD_BARO_MAG);
		uint32_t_to_buf(cnt_cycle_mag, buf_baro_mag, &spi_sd_count_baro_mag, SIZE_OF_BUFFER_SD_BARO_MAG);

		uint16_t_to_buf(mx, buf_baro_mag, &spi_sd_count_baro_mag, SIZE_OF_BUFFER_SD_BARO_MAG);
		uint16_t_to_buf(my, buf_baro_mag, &spi_sd_count_baro_mag, SIZE_OF_BUFFER_SD_BARO_MAG);
		uint16_t_to_buf(mz, buf_baro_mag, &spi_sd_count_baro_mag, SIZE_OF_BUFFER_SD_BARO_MAG);
		float_to_buf(Altitude, buf_baro_mag, &spi_sd_count_baro_mag, SIZE_OF_BUFFER_SD_BARO_MAG);
		float_to_buf(Temperature, buf_baro_mag, &spi_sd_count_baro_mag, SIZE_OF_BUFFER_SD_BARO_MAG);
		float_to_buf(temp_gyro, buf_baro_mag, &spi_sd_count_baro_mag, SIZE_OF_BUFFER_SD_BARO_MAG);
		float_to_buf(DataUKF.tru_pos_z, buf_baro_mag, &spi_sd_count_baro_mag, SIZE_OF_BUFFER_SD_BARO_MAG);
		float_to_buf(GPS_alt, buf_baro_mag, &spi_sd_count_baro_mag, SIZE_OF_BUFFER_SD_BARO_MAG);

		ovf_sd_sbor_mag_baro_stack = uxTaskGetStackHighWaterMark(sd_sbor_mag_baro_handle);


	    }

    }
void prvSbor_buf_gps(void *pvParameters)
    {
	while(1)
	    {
		xSemaphoreTake(xSD_GPS_collect_Semaphore, portMAX_DELAY);

		uint32_t_to_buf(counter_sys, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);
		uint32_t_to_buf(cnt_cycle_gps, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);
		uint16_t_to_buf((uint16_t)(((uint16_t)GPS_satellits<<8)|((uint16_t)flag_start_home)), buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);

		float_to_buf(GPS_X, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);
		float_to_buf(GPS_Y, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);
		float_to_buf(GPS_alt, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);

		float_to_buf(GPS_vel, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);
		float_to_buf(GPS_course, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);

		float_to_buf(GPS_X_speed, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);
		float_to_buf(GPS_Y_speed, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);
		float_to_buf(GPS_Z_speed, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);

		uint32_t_to_buf(GPS_lat, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);
		uint32_t_to_buf(GPS_lon, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);

		uint16_t_to_buf(GPS_hAcc, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);
		uint16_t_to_buf(GPS_vAcc, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);
		uint16_t_to_buf(GPS_sAcc, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);
		uint16_t_to_buf(GPS_vDop, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);
		uint16_t_to_buf(GPS_tDop, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);
		uint16_t_to_buf(GPS_eDop, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);
		uint16_t_to_buf(GPS_nDop, buf_gps, &spi_sd_count_gps, SIZE_OF_BUFFER_SD_GPS);

		ovf_sd_sbor_gps_stack = uxTaskGetStackHighWaterMark(sd_sbor_gps_handle);

	    }
    }

void prvSbor_buf_pid(void *pvParameters)
    {
	while(1)
	    {
		xSemaphoreTake(xSD_PID_collect_Semaphore, portMAX_DELAY);

		uint32_t_to_buf(counter_sys, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);
		uint32_t_to_buf(cnt_cycle_ins, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);

		uint16_t_to_buf(ax, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);
		uint16_t_to_buf(ay, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);
		uint16_t_to_buf(az, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);
		uint16_t_to_buf(gx, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);
		uint16_t_to_buf(gy, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);
		uint16_t_to_buf(gz, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);

		float_to_buf(control_action_x, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);
		float_to_buf(control_action_y, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);
		float_to_buf(control_action_z, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);
		float_to_buf(control_action_thr, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);

		float_to_buf(Ref_for_vel_x, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);
		float_to_buf(Ref_for_vel_y, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);

		float_to_buf(Ref_for_angle_Glob_x, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);
		float_to_buf(Ref_for_angle_Glob_y, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);

		float_to_buf(Ref_for_ugol_x, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);
		float_to_buf(Ref_for_ugol_y, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);

		float_to_buf(ref_alt_hold, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);

		float_to_buf(pid_velX_i, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);
		float_to_buf(pid_velY_i, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);
		float_to_buf(pid_velZ_i, buf_pid, &spi_sd_count_pid, SIZE_OF_BUFFER_SD_PID);


		ovf_sd_sbor_pid_stack = uxTaskGetStackHighWaterMark(sd_sbor_pid_handle);

	    }
    }
void prvSbor_buf_ukf(void *pvParameters)
    {
	while(1)
	    {
		xSemaphoreTake(xSD_UKF_collect_Semaphore, portMAX_DELAY);

		uint32_t_to_buf(counter_sys, buf_ukf, &spi_sd_count_ukf, SIZE_OF_BUFFER_SD_UKF);
		uint32_t_to_buf(cnt_cycle_ukf, buf_ukf, &spi_sd_count_ukf, SIZE_OF_BUFFER_SD_UKF);

		float_to_buf(DataUKF.posx, buf_ukf, &spi_sd_count_ukf, SIZE_OF_BUFFER_SD_UKF);
		float_to_buf(DataUKF.posy, buf_ukf, &spi_sd_count_ukf, SIZE_OF_BUFFER_SD_UKF);
		float_to_buf(DataUKF.posz, buf_ukf, &spi_sd_count_ukf, SIZE_OF_BUFFER_SD_UKF);
		float_to_buf(DataUKF.velx, buf_ukf, &spi_sd_count_ukf, SIZE_OF_BUFFER_SD_UKF);
		float_to_buf(DataUKF.vely, buf_ukf, &spi_sd_count_ukf, SIZE_OF_BUFFER_SD_UKF);
		float_to_buf(DataUKF.velz, buf_ukf, &spi_sd_count_ukf, SIZE_OF_BUFFER_SD_UKF);

		float_to_buf(DataUKF.abiasx, buf_ukf, &spi_sd_count_ukf, SIZE_OF_BUFFER_SD_UKF);
		float_to_buf(DataUKF.abiasy, buf_ukf, &spi_sd_count_ukf, SIZE_OF_BUFFER_SD_UKF);
		float_to_buf(DataUKF.abiasz, buf_ukf, &spi_sd_count_ukf, SIZE_OF_BUFFER_SD_UKF);

		float_to_buf(gxbias, buf_ukf, &spi_sd_count_ukf, SIZE_OF_BUFFER_SD_UKF);
		float_to_buf(gybias, buf_ukf, &spi_sd_count_ukf, SIZE_OF_BUFFER_SD_UKF);
		float_to_buf(gzbias, buf_ukf, &spi_sd_count_ukf, SIZE_OF_BUFFER_SD_UKF);

		float_to_buf(axisx, buf_ukf, &spi_sd_count_ukf, SIZE_OF_BUFFER_SD_UKF);
		float_to_buf(axisy, buf_ukf, &spi_sd_count_ukf, SIZE_OF_BUFFER_SD_UKF);
		float_to_buf(axisz, buf_ukf, &spi_sd_count_ukf, SIZE_OF_BUFFER_SD_UKF);

		ovf_sd_sbor_ukf_stack = uxTaskGetStackHighWaterMark(sd_sbor_ukf_handle);

	    }
    }
void prvSbor_buf_flags1(void *pvParameters)
    {
	float pam1_last, pam2_last, KD_ugla_xy_last;
	float pam3_last, pam4_last, pam5_last;
	float pam6_last, pam7_last, KD_ugla_z_last;
	float pam8_last, pam9_last, pam10_last;
	uint16_t full_flag_last;
	uint16_t err_num_i2c_def_last, err_num_i2c_line_last;

	while(1)
	    {
		xSemaphoreTake(xSD_FLAGS_collect_Semaphore, portMAX_DELAY);
		static uint16_t full_flag;
		uint32_t_to_buf(counter_sys, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);

		uint16_t_to_buf(full_flag_last, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);
		uint16_t_to_buf(err_num_i2c_def_last, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);
		uint16_t_to_buf(err_num_i2c_line_last, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);

		float_to_buf(pam1_last, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);
		float_to_buf(pam2_last, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);
		float_to_buf(pam3_last, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);
		float_to_buf(pam4_last, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);
		float_to_buf(pam5_last, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);

		float_to_buf(pam6_last, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);
		float_to_buf(pam7_last, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);
		float_to_buf(pam8_last, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);
		float_to_buf(pam9_last, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);
		float_to_buf(pam10_last, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);

		//float_to_buf_flags_last();

		uint32_t_to_buf(counter_sys+1, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);
		uint8_t Arming;
		Arming = (supervisorState&STATE_ARMED)>>2;
		full_flag = (uint16_t)((((uint16_t)flag_altWithoutGPS<<11)|((uint16_t)compass_not_health<<10))|((uint16_t)baro_not_health<<9)|((uint16_t)Arming<<8)|((uint16_t)buzzer_flag));
		uint16_t_to_buf(full_flag, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);
		uint16_t_to_buf(err_num_i2c_def, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);
		uint16_t_to_buf(err_num_i2c_line, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);

		float_to_buf(KP_Pos_Alt_Z, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);

		float_to_buf(KP_Vel_Alt_Z, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);
		float_to_buf(KD_Vel_Alt_Z, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);

		float_to_buf(KP_Acc_Alt_Z, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);
		float_to_buf(KI_Vel_Alt_Z, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);
		float_to_buf(KD_Acc_Alt_Z, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);

		float_to_buf(f_cut_err_vel_alt_z, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);
		float_to_buf(f_cut_diff_vel_alt_z, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);

		float_to_buf(KI_rate_z, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);
		float_to_buf(KD_rate_z, buf_flags, &spi_sd_count_flags, SIZE_OF_BUFFER_SD_FLAGS);

		//float_to_buf_flags();

		full_flag_last = full_flag;
		err_num_i2c_def_last = err_num_i2c_def;
		err_num_i2c_line_last = err_num_i2c_line;

		pam1_last = KP_Pos_Alt_Z;
		pam2_last = KP_Vel_Alt_Z;
		pam3_last = KD_Vel_Alt_Z;
		pam4_last = KP_Acc_Alt_Z;
		pam5_last = KI_Vel_Alt_Z;

		pam6_last = KD_Acc_Alt_Z;
		pam7_last = f_cut_err_vel_alt_z;
		pam8_last = f_cut_diff_vel_alt_z;

		pam9_last = KI_rate_z;
		pam10_last = KD_rate_z;

		ovf_sd_sbor_flags_stack = uxTaskGetStackHighWaterMark(sd_sbor_flags_handle);
		}
}


void prvTransfer_buf(void *pvParameters)
    {

	uint16_t fix_start, fix_end;
	uint8_t flag_wr;

	while(1)
	    {
		if (spi_sd_count_ukf_fix < spi_sd_count_ukf)
		    {
			if (spi_sd_count_ukf-spi_sd_count_ukf_fix > SIZE_OF_FRAME_SD_UKF*30-1)
			    {
				fix_start = spi_sd_count_ukf_fix;
				fix_end = spi_sd_count_ukf;
				spi_sd_count_ukf_fix = spi_sd_count_ukf;
				err_write_dan=f_write(&file_ukf, &buf_ukf[fix_start], fix_end-fix_start, &nWritten_ukf);
				err_sync=f_sync(&file_ukf);
				flag_wr = 1;
			    }
		    }
		if (spi_sd_count_ukf_fix > spi_sd_count_ukf)
		    {
			if (spi_sd_count_ukf+(SIZE_OF_BUFFER_SD_UKF-spi_sd_count_ukf_fix) > SIZE_OF_FRAME_SD_UKF*30-1)
			    {
				fix_start = spi_sd_count_ukf_fix;
				fix_end = spi_sd_count_ukf;
				spi_sd_count_ukf_fix = spi_sd_count_ukf;
				err_write_dan=f_write(&file_ukf, &buf_ukf[fix_start], SIZE_OF_BUFFER_SD_UKF-fix_start, &nWritten_ukf);
				err_write_dan=f_write(&file_ukf, buf_ukf, fix_end, &nWritten_ukf);

				err_sync=f_sync(&file_ukf);
				flag_wr = 1;
			    }
		    }
		//***********************************************************************//
		if (flag_wr == 0)
		    {
			if (spi_sd_count_pid_fix < spi_sd_count_pid)
			    {
				if (spi_sd_count_pid-spi_sd_count_pid_fix > SIZE_OF_FRAME_SD_PID*100-1)
				    {
					fix_start = spi_sd_count_pid_fix;
					fix_end = spi_sd_count_pid;
					spi_sd_count_pid_fix = spi_sd_count_pid;
					err_write_dan=f_write(&file_pid, &buf_pid[fix_start], fix_end-fix_start, &nWritten_pid);
					err_sync=f_sync(&file_pid);
					flag_wr = 1;
				    }
			    }
			if (spi_sd_count_pid_fix > spi_sd_count_pid)
			    {
				if (spi_sd_count_pid+(SIZE_OF_BUFFER_SD_PID-spi_sd_count_pid_fix) > SIZE_OF_FRAME_SD_PID*100-1)
				    {
					fix_start = spi_sd_count_pid_fix;
					fix_end = spi_sd_count_pid;
					spi_sd_count_pid_fix = spi_sd_count_pid;
					err_write_dan=f_write(&file_pid, &buf_pid[fix_start], SIZE_OF_BUFFER_SD_PID-fix_start, &nWritten_pid);
					err_write_dan=f_write(&file_pid, buf_pid, fix_end, &nWritten_pid);

					err_sync=f_sync(&file_pid);
					flag_wr = 1;
				    }
			    }
		    }

		//***********************************************************************//
		if (flag_wr == 0)
		    {
			if (spi_sd_count_pwm_fix < spi_sd_count_pwm)
			    {
				if (spi_sd_count_pwm-spi_sd_count_pwm_fix > SIZE_OF_FRAME_SD_PWM*60-1)
				    {
					cnt_ticks_sd_write = (uint32_t)DWT->CYCCNT;

					fix_start = spi_sd_count_pwm_fix;
					fix_end = spi_sd_count_pwm;
					spi_sd_count_pwm_fix = spi_sd_count_pwm;
					err_write_dan=f_write(&file_pwm, &buf_pwm[fix_start], fix_end-fix_start, &nWritten_pwm);
					err_sync=f_sync(&file_pwm);
					flag_wr = 1;
					cnt_cycle_sd_write = (uint32_t)DWT->CYCCNT - cnt_ticks_sd_write;
					if ((cnt_cycle_sd_write<800000)&&(cnt_cycle_sd_write>0))buzzer_flag=BUZZER_FLAG_FAIL_SD;
				    }
			    }
			if (spi_sd_count_pwm_fix > spi_sd_count_pwm)
			    {
				if (spi_sd_count_pwm+(SIZE_OF_BUFFER_SD_PWM-spi_sd_count_pwm_fix) > SIZE_OF_FRAME_SD_PWM*60-1)
				    {
					fix_start = spi_sd_count_pwm_fix;
					fix_end = spi_sd_count_pwm;
					spi_sd_count_pwm_fix = spi_sd_count_pwm;
					err_write_dan=f_write(&file_pwm, &buf_pwm[fix_start], SIZE_OF_BUFFER_SD_PWM-fix_start, &nWritten_pwm);
					err_write_dan=f_write(&file_pwm, buf_pwm, fix_end, &nWritten_pwm);

					err_sync=f_sync(&file_pwm);
					flag_wr = 1;
				    }
			    }
		    }

		//***********************************************************************//
		if (flag_wr == 0)
		    {
			if (spi_sd_count_ppm_fix < spi_sd_count_ppm)
			    {
				if (spi_sd_count_ppm-spi_sd_count_ppm_fix > SIZE_OF_FRAME_SD_PPM*20-1)
				    {
					fix_start = spi_sd_count_ppm_fix;
					fix_end = spi_sd_count_ppm;
					spi_sd_count_ppm_fix = spi_sd_count_ppm;
					err_write_dan=f_write(&file_ppm, &buf_ppm[fix_start], fix_end-fix_start, &nWritten_ppm);
					err_sync=f_sync(&file_ppm);
					flag_wr = 1;
				    }
			    }
			if (spi_sd_count_ppm_fix > spi_sd_count_ppm)
			    {
				if (spi_sd_count_ppm+(SIZE_OF_BUFFER_SD_PPM-spi_sd_count_ppm_fix)+1 > SIZE_OF_FRAME_SD_PPM*20-1)
				    {
					fix_start = spi_sd_count_ppm_fix;
					fix_end = spi_sd_count_ppm;
					spi_sd_count_ppm_fix = spi_sd_count_ppm;
					err_write_dan=f_write(&file_ppm, &buf_ppm[fix_start], SIZE_OF_BUFFER_SD_PPM-fix_start, &nWritten_ppm);
					err_write_dan=f_write(&file_ppm, buf_ppm, fix_end, &nWritten_ppm);

					err_sync=f_sync(&file_ppm);
					flag_wr = 1;
				    }
			    }
		    }
		flag_wr = 0;
		vTaskDelay(2);
		//***********************************************************************//
		if (flag_wr == 0)
		    {
			if (spi_sd_count_ADC_fix < spi_sd_count_ADC)
			    {
				if (spi_sd_count_ADC-spi_sd_count_ADC_fix > SIZE_OF_FRAME_SD_ADC*28-1)
				    {
					fix_start = spi_sd_count_ADC_fix;
					fix_end = spi_sd_count_ADC;
					spi_sd_count_ADC_fix = spi_sd_count_ADC;
					err_write_dan=f_write(&file_adc, &buf_ADC_res[fix_start], fix_end-fix_start, &nWritten_adc);
					err_sync=f_sync(&file_adc);
					flag_wr = 1;
				    }
			    }
			if (spi_sd_count_ADC_fix > spi_sd_count_ADC)
			    {
				if (spi_sd_count_ADC+(SIZE_OF_BUFFER_SD_ADC-spi_sd_count_ADC_fix)+1 > SIZE_OF_FRAME_SD_ADC*28-1)
				    {
					fix_start = spi_sd_count_ADC_fix;
					fix_end = spi_sd_count_ADC;
					spi_sd_count_ADC_fix = spi_sd_count_ADC;
					err_write_dan=f_write(&file_adc, &buf_ADC_res[fix_start], SIZE_OF_BUFFER_SD_ADC-fix_start, &nWritten_adc);
					err_write_dan=f_write(&file_adc, buf_ADC_res, fix_end, &nWritten_adc);

					err_sync=f_sync(&file_adc);
					flag_wr = 1;
				    }
			    }
		    }

		//***********************************************************************//
		if (flag_wr == 0)
		    {
			if (spi_sd_count_baro_mag_fix < spi_sd_count_baro_mag)
			    {
				if (spi_sd_count_baro_mag-spi_sd_count_baro_mag_fix > SIZE_OF_FRAME_SD_BARO_MAG*70-1)
				    {
					fix_start = spi_sd_count_baro_mag_fix;
					fix_end = spi_sd_count_baro_mag;
					spi_sd_count_baro_mag_fix = spi_sd_count_baro_mag;
					err_write_dan=f_write(&file_baro_mag, &buf_baro_mag[fix_start], fix_end-fix_start, &nWritten_baro_mag);
					err_sync=f_sync(&file_baro_mag);
					flag_wr = 1;
				    }
			    }
			if (spi_sd_count_baro_mag_fix > spi_sd_count_baro_mag)
			    {
				if (spi_sd_count_baro_mag+(SIZE_OF_BUFFER_SD_BARO_MAG-spi_sd_count_baro_mag_fix)+1 > SIZE_OF_FRAME_SD_BARO_MAG*70-1)
				    {
					fix_start = spi_sd_count_baro_mag_fix;
					fix_end = spi_sd_count_baro_mag;
					spi_sd_count_baro_mag_fix = spi_sd_count_baro_mag;
					err_write_dan=f_write(&file_baro_mag, &buf_baro_mag[fix_start], SIZE_OF_BUFFER_SD_BARO_MAG-fix_start, &nWritten_baro_mag);
					err_write_dan=f_write(&file_baro_mag, buf_baro_mag, fix_end, &nWritten_baro_mag);

					err_sync=f_sync(&file_baro_mag);
					flag_wr = 1;
				    }
			    }
		    }

		//***********************************************************************//
		if (flag_wr == 0)
		    {
			if (spi_sd_count_gps_fix < spi_sd_count_gps)
			    {
				if (spi_sd_count_gps-spi_sd_count_gps_fix > SIZE_OF_FRAME_SD_GPS*14-1)
				    {
					fix_start = spi_sd_count_gps_fix;
					fix_end = spi_sd_count_gps;
					spi_sd_count_gps_fix = spi_sd_count_gps;
					err_write_dan=f_write(&file_gps, &buf_gps[fix_start], fix_end-fix_start, &nWritten_gps);
					err_sync=f_sync(&file_gps);
					flag_wr = 1;
				    }
			    }
			if (spi_sd_count_gps_fix > spi_sd_count_gps)
			    {
				if (spi_sd_count_gps+(SIZE_OF_BUFFER_SD_GPS-spi_sd_count_gps_fix)+1 > SIZE_OF_FRAME_SD_GPS*14-1)
				    {
					fix_start = spi_sd_count_gps_fix;
					fix_end = spi_sd_count_gps;
					spi_sd_count_gps_fix = spi_sd_count_gps;
					err_write_dan=f_write(&file_gps, &buf_gps[fix_start], SIZE_OF_BUFFER_SD_GPS-fix_start, &nWritten_gps);
					err_write_dan=f_write(&file_gps, buf_gps, fix_end, &nWritten_gps);

					err_sync=f_sync(&file_gps);
					flag_wr = 1;
				    }
			    }
		    }


		//***********************************************************************//
		if (flag_wr == 0)
		    {
			if (spi_sd_count_flags_fix < spi_sd_count_flags)
			    {
				if (spi_sd_count_flags-spi_sd_count_flags_fix > SIZE_OF_FRAME_SD_FLAGS*3-1)
				    {
					fix_start = spi_sd_count_flags_fix;
					fix_end = spi_sd_count_flags;
					spi_sd_count_flags_fix = spi_sd_count_flags;
					err_write_dan=f_write(&file_flags, &buf_flags[fix_start], fix_end-fix_start, &nWritten_flags);
					err_sync=f_sync(&file_flags);
					flag_wr = 1;
				    }
			    }
			if (spi_sd_count_flags_fix > spi_sd_count_flags)
			    {
				if (spi_sd_count_flags+(SIZE_OF_BUFFER_SD_FLAGS-spi_sd_count_flags_fix) > SIZE_OF_FRAME_SD_FLAGS*3-1)
				    {
					fix_start = spi_sd_count_flags_fix;
					fix_end = spi_sd_count_flags;
					spi_sd_count_flags_fix = spi_sd_count_flags;
					err_write_dan=f_write(&file_flags, &buf_flags[fix_start], SIZE_OF_BUFFER_SD_FLAGS-fix_start, &nWritten_flags);
					err_write_dan=f_write(&file_flags, buf_flags, fix_end, &nWritten_flags);

					err_sync=f_sync(&file_flags);
					flag_wr = 1;
				    }
			    }
		    }

		flag_wr = 0;
		ovf_sd_trans_stack = uxTaskGetStackHighWaterMark(sd_trans_handle);
		close_op_new_file();
		vTaskDelay(2);
		//***********************************************************************//
		/*
		 * В этом же потоке можно работать с FRAM по тому же SPI
		 *
		 *
		 */
	}
    }


