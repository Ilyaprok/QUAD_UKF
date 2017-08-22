
#ifndef _SD_LOGS_H_
#define _SD_LOGS_H_

#include "UKF_lib.h"

//u32 1-19
#define TIME_LOG_ID 			1
#define CYCLE_TIME_PID_LOG_ID		2
#define CYCLE_TIME_PWM_LOG_ID		3
#define CYCLE_TIME_PPM_LOG_ID		4
#define CYCLE_TIME_BARO_LOG_ID		5
#define CYCLE_TIME_MAG_LOG_ID		6
#define CYCLE_TIME_GPS_LOG_ID		7
#define CYCLE_TIME_ADC_LOG_ID		8
#define CYCLE_TIME_UKF_LOG_ID		9

//u16 s16 20-69
#define YAW_PPM_LOG_ID 			20
#define THROTTLE_PPM_LOG_ID 		21
#define ROLL_PPM_LOG_ID 		22
#define PITCH_PPM_LOG_ID 		23
#define MODE_PPM_LOG_ID 		24
#define MODE2_PPM_LOG_ID 		25


#define M1_LOG_ID 			26
#define M2_LOG_ID 			27
#define M3_LOG_ID 			28
#define M4_LOG_ID 			29

#define AX_LOG_ID 			30
#define AY_LOG_ID 			31
#define AZ_LOG_ID 			32
#define GX_LOG_ID 			33
#define GY_LOG_ID 			34
#define GZ_LOG_ID 			35

#define ERR_I2C_DEF_LOG_ID		36
#define ERR_I2C_LINE_LOG_ID 		37

#define MX_RAW_LOG_ID 			38
#define MY_RAW_LOG_ID 			39
#define MZ_RAW_LOG_ID 			40

#define MODE3_PPM_LOG_ID 		41
#define OFF_POW_PPM_LOG_ID 		42

#define GPS_HACC_LOG_ID 		43
#define GPS_VACC_LOG_ID 		44
#define GPS_SACC_LOG_ID 		45
#define GPS_VDOP_LOG_ID 		46
#define GPS_TDOP_LOG_ID 		47
#define GPS_EDOP_LOG_ID 		48
#define GPS_NDOP_LOG_ID 		49


//u8 70-99
#define ARM_LOG_ID 			70
#define BUZZER_LOG_ID 			71
#define GPS_SATTELITS_LOG_ID 		72
#define GPS_START_HOME_LOG_ID		73
#define ALT_WITHOUT_GPS_LOG_ID		74

//float 100-229
//UKF
#define UKF_Q0_LOG_ID 			100
#define UKF_Q1_LOG_ID 			101
#define UKF_Q2_LOG_ID 			102
#define UKF_Q3_LOG_ID 			103

#define UKF_POSX_LOG_ID 		104
#define UKF_POSY_LOG_ID 		105
#define UKF_POSZ_LOG_ID 		106

#define UKF_VELX_LOG_ID 		107
#define UKF_VELY_LOG_ID 		108
#define UKF_VELZ_LOG_ID 		109

#define UKF_ABX_LOG_ID 			110
#define UKF_ABY_LOG_ID 			111
#define UKF_ABZ_LOG_ID 			112

#define UKF_GBX_LOG_ID 			113
#define UKF_GBY_LOG_ID 			114
#define UKF_GBZ_LOG_ID 			115

#define UKF_AXISX_LOG_ID 		116
#define UKF_AXISY_LOG_ID 		117
#define UKF_AXISZ_LOG_ID 		118

//PID
#define PID_U_X_LOG_ID 			125
#define PID_U_Y_LOG_ID 			126
#define PID_U_Z_LOG_ID 			127
#define PID_U_T_LOG_ID 			128

#define PID_REF_VEL_GLOB_X_LOG_ID 	129
#define PID_REF_VEL_GLOB_Y_LOG_ID 	130
#define PID_REF_ANG_GLOB_X_LOG_ID 	131
#define PID_REF_ANG_GLOB_Y_LOG_ID 	132
#define PID_REF_ANG_X_LOG_ID 		133
#define PID_REF_ANG_Y_LOG_ID 		134

#define PID_VEL_Z_LOG_ID 			135

#define PID_I_RATEX_LOG_ID		 	136
#define PID_I_RATEY_LOG_ID 			137
#define PID_I_RATEZ_LOG_ID 			138


//ADC
#define ALL_VOLT_LOG_ID 		140
#define ALL_CURR_LOG_ID 		141
#define ALL_POWR_LOG_ID 		142
#define ALL_ACTN_LOG_ID 		143
#define ALL_VOLT_SM_LOG_ID 		144

#define CURRENT1_LOG_ID 		145
#define CURRENT2_LOG_ID 		146
#define CURRENT3_LOG_ID 		147
#define CURRENT4_LOG_ID 		148

#define ALL_PERC_CAP_LOG_ID 		149

//BARO_MAG
#define ALT_BARO_LOG_ID 		155
#define TEMP_BARO_LOG_ID 		156

#define TEMP_GYRO_LOG_ID 		157

//GPS
#define GPS_X_LOG_ID 			160
#define GPS_Y_LOG_ID 			161
#define GPS_ALT_LOG_ID 			162
#define GPS_VEL_LOG_ID 			163
#define GPS_COURSE_LOG_ID 		164

#define GPS_X_SPEED_LOG_ID 		165
#define GPS_Y_SPEED_LOG_ID 		166
#define GPS_Z_SPEED_LOG_ID 		167

//FLAGS
#define KP_PID_UGOL_XY_LOG_ID 		175
#define KI_PID_UGOL_XY_LOG_ID 		176

#define KP_PID_VEL_XY_LOG_ID 		177
#define KI_PID_VEL_XY_LOG_ID 		178
#define KD_PID_VEL_XY_LOG_ID 		179

#define KP_PID_UGOL_Z_LOG_ID 		180
#define KI_PID_UGOL_Z_LOG_ID 		181

#define KP_PID_VEL_Z_LOG_ID 		182
#define KI_PID_VEL_Z_LOG_ID 		183
#define KD_PID_VEL_Z_LOG_ID 		184


#define KP_PID_ALT_POS_LOG_ID 		185
#define KP_PID_ALT_VEL_LOG_ID 		186
#define KD_PID_ALT_VEL_LOG_ID 		187

#define KP_PID_ALT_ACC_LOG_ID 		188
#define KI_PID_ALT_ACC_LOG_ID 		189
#define KD_PID_ALT_ACC_LOG_ID 		190

#define F_CUT_ERR_ALT_VEL_LOG_ID 	191
#define F_CUT_DIFF_ALT_VEL_LOG_ID 	192

//i32 - 230-255
#define GPS_LAT_LOG_ID 			230
#define GPS_LON_LOG_ID 			231

/********************************************************/

#define SIZE_OF_FRAME_SD_UKF		68
#define SIZE_OF_BUFFER_SD_UKF		SIZE_OF_FRAME_SD_UKF*45

#define SIZE_OF_FRAME_SD_PWM		16
#define SIZE_OF_BUFFER_SD_PWM		SIZE_OF_FRAME_SD_PWM*100

#define SIZE_OF_FRAME_SD_PPM		24
#define SIZE_OF_BUFFER_SD_PPM		SIZE_OF_FRAME_SD_PPM*40

#define SIZE_OF_FRAME_SD_BARO_MAG	38
#define SIZE_OF_BUFFER_SD_BARO_MAG	SIZE_OF_FRAME_SD_BARO_MAG*100

#define SIZE_OF_FRAME_SD_GPS		64
#define SIZE_OF_BUFFER_SD_GPS		SIZE_OF_FRAME_SD_GPS*18

#define SIZE_OF_FRAME_SD_FLAGS		100
#define SIZE_OF_BUFFER_SD_FLAGS		SIZE_OF_FRAME_SD_FLAGS*4

#define SIZE_OF_FRAME_SD_ADC		48
#define SIZE_OF_BUFFER_SD_ADC		SIZE_OF_FRAME_SD_ADC*40

#define SIZE_OF_FRAME_SD_PID		76
#define SIZE_OF_BUFFER_SD_PID		SIZE_OF_FRAME_SD_PID*150



//extern uint32_t cnt_ticks_baro, cnt_cycle_baro;
//
//extern uint32_t cnt_ticks_ins, cnt_cycle_ins;
//extern uint32_t cnt_ticks_pwm, cnt_cycle_pwm;
//extern uint32_t cnt_ticks_ppm, cnt_cycle_ppm;
//extern uint32_t cnt_ticks_mag, cnt_cycle_mag;
//extern uint32_t cnt_ticks_gps, cnt_cycle_gps;
//extern uint32_t cnt_ticks_adc, cnt_cycle_adc;

extern data_ukf DataUKF;



extern SemaphoreHandle_t xSD_MAG_BARO_collect_Semaphore;
extern SemaphoreHandle_t xSD_PPM_collect_Semaphore;
extern SemaphoreHandle_t xSD_PWM_collect_Semaphore;
extern SemaphoreHandle_t xSD_FLAGS_collect_Semaphore;
extern SemaphoreHandle_t xSD_ADC_collect_Semaphore;
extern SemaphoreHandle_t xSD_GPS_collect_Semaphore;
extern SemaphoreHandle_t xSD_UKF_collect_Semaphore;
extern SemaphoreHandle_t xSD_PID_collect_Semaphore;

////импорт
//extern int16_t ax, ay, az, gx, gy, gz;
//extern float q0, q1, q2, q3;
//extern float q0_mag, q1_mag, q2_mag, q3_mag;
//extern float acc_z, vel_z, pos_z, vel_z_bias, acc_z_bias;
//
//extern uint16_t ppm1_buf[9]; 		/* буфер для значений каналов ППМ */
//extern float control_action_y, Ref_for_rate_y, Ref_for_ugol_y;
//extern float control_action_x, Ref_for_rate_x, Ref_for_ugol_x;
//extern float control_action_z, Ref_for_rate_z, Ref_for_ugol_z;
//extern float control_action_thr;
//extern float Ref_for_vel_alt_z, Ref_for_acc_alt_z, ref_alt_hold, control_action_acc_alt_z;
//
//extern float yaw_rate_ppm_exp;
//extern uint16_t M1_lev_front, M2_prav_back, M3_prav_front, M4_lev_back;
//
//extern float KP_ugla_xy, KI_ugla_xy, KD_ugla_xy;
//extern float KP_rate_xy, KI_rate_xy, KD_rate_xy;
//extern float KP_ugla_z, KI_ugla_z, KD_ugla_z;
//extern float KP_rate_z, KI_rate_z, KD_rate_z;
//
////Высота
//extern float KP_Pos_Alt_Z;
//extern float KP_Vel_Alt_Z, KD_Vel_Alt_Z;
//extern float KP_Acc_Alt_Z, KI_Vel_Alt_Z, KD_Acc_Alt_Z;
//extern float f_cut_diff_vel_alt_z, f_cut_err_vel_alt_z;
//
//extern uint16_t err_num_i2c_def, err_num_i2c_line;
//
//extern float All_Current, All_voltage, All_power, All_action, All_voltage_smooth, All_percent_cap, All_capacity_batt;
//extern float current_1, current_2, current_3, current_4;
//extern float Temperature, Altitude, temp_gyro;
//extern float mxf, myf, mzf;
//extern float M_X, M_Y, M_Z;
//extern int16_t mx, my, mz;
//extern uint8_t compass_not_health, baro_not_health;
//
//extern uint8_t GPS_NS;
//extern uint8_t GPS_EW;
//extern float GPS_LA, GPS_LO, GPS_LO_start, GPS_LA_start;
//extern float GPS_vel, GPS_course;
//extern float X_GPS, Y_GPS;
//
//extern uint8_t GPS_hours, GPS_minutes, GPS_seconds;
//extern uint8_t GPS_satellits;
//
//
//extern float GPS_Y_speed, GPS_X_speed, GPS_Z_speed;
//extern int32_t GPS_lon, GPS_lat;
//
//extern float GPS_Hdop;
//extern float GPS_alt;
//
//extern uint8_t flag_start_home;





void Init_logs(void);
void SPI_1_Init(void);

void prvSbor_buf_ukf(void *pvParameters);
void prvSbor_buf_pwm(void *pvParameters);
void prvSbor_buf_ppm(void *pvParameters);
void prvSbor_buf_baro_mag(void *pvParameters);
void prvSbor_buf_flags1(void *pvParameters);
void prvSbor_buf_adc(void *pvParameters);
void prvSbor_buf_gps(void *pvParameters);
void prvSbor_buf_pid(void *pvParameters);

void prvTransfer_buf(void *pvParameters);

void close_op_new_file(void);

#endif /* _SD_LOGS_H_ */
