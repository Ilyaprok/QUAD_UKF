

#include "Config.h"

#include "IMU_INS.h"

#include "MPU6500.h"
#include "I2C_BARO_MAG.h"
#include "USART_TELEMETRY.h"
#include "ADC.h"
#include "GPS.h"
#include "PWM_PPM.h"
#include "UKF_lib.h"
#include "PID.h"
#include "NAV_POINTS.h"


#define LPF_ACC_SR	0.98f
#define K_GYRO 		(32767.0f/1000.0f*RAD2DEG)

#define constrainInt(v, lo, hi)	    (((int)(v) < (int)(lo)) ? (int)(lo) : (((int)(v) > (int)(hi)) ? (int)(hi) : (int)(v)))
#define constrainFloat(v, lo, hi)   (((float)(v) < (float)(lo)) ? (float)(lo) : (((float)(v) > (float)(hi)) ? (float)(hi) : (float)(v)))




//данные с инерциальных датчиков
int16_t ax, ay, az, gx, gy, gz;//сырые даные в 16 разр€дах
float ax_sr, ay_sr, az_sr;//‘Ќ„ ускорений дл€ вспомогательных целей
float temp_gyro;//температура датчика
float axf, ayf, azf, gxf, gyf, gzf;//правильные данные с датчиков в g и рад/сек
//****************************************************//

//вспомогательые служебные перменные
uint8_t flag_end_of_gyro_calib;//флаг конца калибровки
uint32_t cnt_cycle_ins, cnt_ticks_ins;//счетчики цикла
uint32_t ovf_ins_stack;//свободный стек
uint8_t flag_altWithoutGPS;
//****************************************************//

//«адача дл€ UKF
//семафор дл€ св€зи с модулем орбаботки
SemaphoreHandle_t xUKF_Semaphore = NULL;
SemaphoreHandle_t  xMPU_UKF_Mutex, xUKF_PID_Mutex;
TaskHandle_t ukf_handle;
uint32_t ovf_ukf_stack;
//****************************************************//
uint32_t cnt_cycle_ukf, cnt_ticks_ukf;//счетчики цикла

//калибровочные константы
const float Z_plus = 4160.0f, Z_minus = 4175.0f, Y_plus = 4120.0f, Y_minus = 4090.0f, X_plus = 4200.0f, X_minus = 4000.0f;

const float ax_bias = 89.564543f, ay_bias = 11.174040f, az_bias = -10.346222f;
const float am1 = 0.999500f, 	am2 = -0.000151f, 	am3 = 0.003176f;
const float			 am5 = 0.999091f, 	am6 = 0.005234f;
const float  						am9 = 0.983432f;
//****************************************************//

//****************************************************//
//–азличные полезные расчетые перменные//
//****************************************************//
float mq1, mq2, mq3, mq4, mq5, mq6, mq7, mq8, mq9;//матрица поворота
float ref_alt_hold;
float alt_offset = 0.0f;
float axisz_ref;

float e_mix_x, e_mix_y, e_mix_z;

float q0_ref_x, q1_ref_x;
float q0_ref_y, q2_ref_y;
float q0_ref_z = 1.0f, q3_ref_z = 0.0f;
float yaw_rate_ppm_exp;

float x_ref, y_ref;
float axisx2, axisy2, axisz2;
float g_mix_z;

float DeltaTimeUKF;
//double DeltaTimeUKF2;
float temp_sat = 0.002f;


void UKF_Cycle(void)
{
	static uint32_t cnt_loop, cnt_loop2;
	static uint32_t axis = 0;
	//«ащищенные переменные
	float gps_x, gps_y, gps_tdop, gps_alt, gps_hacc, gps_vacc, gps_y_speed, gps_x_speed, gps_z_speed, gps_sacc;
	uint32_t gps_micros;

	if (flag_start_state_init == 1)
	    {
		navUkfInitState();
		cnt_loop2++;
		if (!(cnt_loop2%5))
		    {
			xSemaphoreGive(xParcel_TX_Semaphore);
		    }
	    }
	else if (flag_start_state_init == 2)
	{
		runInit();
		flag_start_state_init = 3;
	}
	else if(flag_start_state_init == 3)
	    {

			cnt_loop++;

			DeltaTimeUKF = (float)cnt_cycle_ukf/168000000.0f;
			//DeltaTimeUKF2 = (double)cnt_cycle_ukf/(double)168000000.0;
			// soft start GPS accuracy
			runData.accMask *= 0.999f;

			navUkfInertialUpdate();



			// record history for acc & mag & pressure readings for smoothing purposes
			// acc
			runData.sumAcc[0] -= runData.accHist[0][runData.sensorHistIndex];
			runData.sumAcc[1] -= runData.accHist[1][runData.sensorHistIndex];
			runData.sumAcc[2] -= runData.accHist[2][runData.sensorHistIndex];
			//taskENTER_CRITICAL();
			xSemaphoreTake(xMPU_UKF_Mutex, portMAX_DELAY);
			runData.accHist[0][runData.sensorHistIndex] = axf;
			runData.accHist[1][runData.sensorHistIndex] = ayf;
			runData.accHist[2][runData.sensorHistIndex] = azf;
			xSemaphoreGive(xMPU_UKF_Mutex);
			//taskEXIT_CRITICAL();

			runData.sumAcc[0] += runData.accHist[0][runData.sensorHistIndex];
			runData.sumAcc[1] += runData.accHist[1][runData.sensorHistIndex];
			runData.sumAcc[2] += runData.accHist[2][runData.sensorHistIndex];

			// mag
			runData.sumMag[0] -= runData.magHist[0][runData.sensorHistIndex];
			runData.sumMag[1] -= runData.magHist[1][runData.sensorHistIndex];
			runData.sumMag[2] -= runData.magHist[2][runData.sensorHistIndex];
			//taskENTER_CRITICAL();
			xSemaphoreTake(xBAROMAG_UKF_Mutex, portMAX_DELAY);
			runData.magHist[0][runData.sensorHistIndex] = mxf;
			runData.magHist[1][runData.sensorHistIndex] = myf;
			runData.magHist[2][runData.sensorHistIndex] = mzf;
			xSemaphoreGive(xBAROMAG_UKF_Mutex);
			//taskEXIT_CRITICAL();
			runData.sumMag[0] += runData.magHist[0][runData.sensorHistIndex];
			runData.sumMag[1] += runData.magHist[1][runData.sensorHistIndex];
			runData.sumMag[2] += runData.magHist[2][runData.sensorHistIndex];

			// pressure
			runData.sumPres -= runData.presHist[runData.sensorHistIndex];
			//taskENTER_CRITICAL();
			xSemaphoreTake(xBAROMAG_UKF_Mutex, portMAX_DELAY);
			runData.presHist[runData.sensorHistIndex] = Altitude;
			xSemaphoreGive(xBAROMAG_UKF_Mutex);
			//taskEXIT_CRITICAL();
			runData.sumPres += runData.presHist[runData.sensorHistIndex];

			runData.sensorHistIndex = (runData.sensorHistIndex + 1) % RUN_SENSOR_HIST;


			if (!((cnt_loop+1) % 20)) {
			   simDoAccUpdate(runData.sumAcc[0]*(1.0f / (float)RUN_SENSOR_HIST), runData.sumAcc[1]*(1.0f / (float)RUN_SENSOR_HIST), runData.sumAcc[2]*(1.0f / (float)RUN_SENSOR_HIST));
			}
			if (!((cnt_loop+7) % 20)) {
			   simDoPresUpdate(runData.sumPres*(1.0f / (float)RUN_SENSOR_HIST));
			}
			//taskENTER_CRITICAL();
			xSemaphoreTake(xGPS_UKF_Mutex, portMAX_DELAY);
			gps_x = GPS_X;
			gps_y = GPS_Y;
			gps_tdop = GPS_tDopf;
			gps_alt = GPS_alt;
			gps_hacc = GPS_hAccf;
			gps_vacc = GPS_vAccf;
			gps_sacc = GPS_sAccf;
			gps_y_speed = GPS_Y_speed;
			gps_x_speed = GPS_X_speed;
			gps_z_speed = GPS_Z_speed;
			gps_sacc = GPS_sAccf;
			gps_micros = GPS_Micros_Update;
			xSemaphoreGive(xGPS_UKF_Mutex);


//			if (flag_get_pvt == 2 &&  gps_hacc < NAV_MIN_GPS_ACC && gps_tdop != 0.0f) {
//
//			    navUkfGpsVelUpdate(gps_micros, gps_y_speed, gps_x_speed, -gps_z_speed, gps_sacc + runData.accMask);
//			    flag_get_pvt = 0;
//			}
//
//			if (flag_get_pvt == 1 &&  gps_hacc < NAV_MIN_GPS_ACC && gps_tdop != 0.0f) {
//			    navUkfGpsPosUpdate(gps_micros, gps_y, gps_x, gps_alt, gps_hacc + runData.accMask, gps_vacc + runData.accMask);
//			    flag_get_pvt = 2;
//
//			    // refine static sea level pressure based on better GPS altitude fixes
//			    if (gps_hacc < runData.bestHacc && gps_hacc < NAV_MIN_GPS_ACC) {
//		                navPressureAdjust(gps_alt);
//				runData.bestHacc = gps_hacc;
//			    }
//			}

			if (flag_get_pvt == 1 &&  gps_hacc < NAV_MIN_GPS_ACC && gps_tdop != 0.0f) {
			    navUkfGpsPosUpdate(gps_micros, gps_y, gps_x, gps_alt, gps_hacc + runData.accMask, gps_vacc + runData.accMask);
			    navUkfGpsVelUpdate(gps_micros, gps_y_speed, gps_x_speed, -gps_z_speed, gps_sacc + runData.accMask);
			    flag_get_pvt = 0;

			    // refine static sea level pressure based on better GPS altitude fixes
			    if (gps_hacc < runData.bestHacc && gps_hacc < NAV_MIN_GPS_ACC) {
		                navPressureAdjust(gps_alt);
				runData.bestHacc = gps_hacc;
			    }
			}



			if((supervisorState&STATE_ARMED)&&(control_action_thr>5.0f)) supervisorState|=STATE_FLYING;
			else supervisorState &= ~STATE_FLYING;

			// observe zero position
			if (!((cnt_loop+4) % 20) && (gps_hacc >= NAV_MIN_GPS_ACC || gps_tdop == 0.0f) && (!(supervisorState & STATE_FLYING))) {
			    navUkfZeroPos();
			}
			// observer zero velocity
			if (!((cnt_loop+10) % 20) && (gps_sacc >= NAV_MIN_GPS_ACC/2.0f || gps_tdop == 0.0f) && (!(supervisorState & STATE_FLYING))) {
			    navUkfZeroVel();
			}
			// observe that the rates are exactly 0 if not flying or moving
			if (!(supervisorState & STATE_FLYING)) {
			    float stdX, stdY, stdZ;
			    float gx, gy, gz;

				//taskENTER_CRITICAL();
				xSemaphoreTake(xMPU_UKF_Mutex, portMAX_DELAY);
				gx = gxf;
				gy = gyf;
				gz = gzf;
				xSemaphoreGive(xMPU_UKF_Mutex);
				//taskEXIT_CRITICAL();

			    arm_std_f32(runData.accHist[0], RUN_SENSOR_HIST, &stdX);
			    arm_std_f32(runData.accHist[1], RUN_SENSOR_HIST, &stdY);
			    arm_std_f32(runData.accHist[2], RUN_SENSOR_HIST, &stdZ);

			    if ((stdX + stdY + stdZ) < (IMU_STATIC_STD*2)) {
				if (!((axis + 0) % 3))
				    navUkfZeroRate(gx, 0);
				else if (!((axis + 1) % 3))
				    navUkfZeroRate(gy, 1);
				else
				    navUkfZeroRate(gz, 2);
				axis++;
			    }
			}

			navUkfFinish();

	        altUkfProcess(runData.presHist[runData.sensorHistIndex]);

	        // determine which altitude estimate to use
	        if (gps_hacc > 2.2f) {
	            runData.altPos = &ALT_POS;
	            runData.altVel = &ALT_VEL;
	            flag_altWithoutGPS = 1;
	            xSemaphoreGive(xSD_FLAGS_collect_Semaphore);
	        }
	        else {
	            runData.altPos = &UKF_ALTITUDE;
	            runData.altVel = &UKF_VELD;
	            flag_altWithoutGPS = 0;
	            xSemaphoreGive(xSD_FLAGS_collect_Semaphore);
	        }
	        //cnt_cycle_all_ukf = (uint32_t)DWT->CYCCNT - cnt_ticks_all_ukf;
	        xSemaphoreGive(xSD_UKF_collect_Semaphore);

		if (!(cnt_loop%5))
		    {
			xSemaphoreGive(xParcel_TX_Semaphore);
		    }


	    }

}

void prvIMU_INS_UKF(void *pvParameters)
{

	while(1)
	{
		xSemaphoreTake(xUKF_Semaphore, portMAX_DELAY);

		cnt_cycle_ukf = (uint32_t)DWT->CYCCNT - cnt_ticks_ukf;
		cnt_ticks_ukf = (uint32_t)DWT->CYCCNT;

		UKF_Cycle();
	}

}

void X_Y_Ref_Cartesian(float x, float y, float delta, float angle_z)
{
	static float exp_x, exp_y;
	if ((x < DEAD_ZONE_XY_STICK)&&(x > -DEAD_ZONE_XY_STICK)) x = 0.0f;

	if ((y < DEAD_ZONE_XY_STICK)&&(y > -DEAD_ZONE_XY_STICK)) y = 0.0f;
	x *= delta*K_VEL_REF_X_Y;
	y *= delta*K_VEL_REF_X_Y;

	exp_x = exp_x + K_EXP_GLOB_REF_XY*(constrainFloat(x - exp_x, -temp_sat, temp_sat));
	exp_y = exp_y + K_EXP_GLOB_REF_XY*(constrainFloat(y - exp_y, -temp_sat, temp_sat));

	x_ref += exp_x*cosf(angle_z)-exp_y*sinf(angle_z);
	y_ref += exp_y*cosf(angle_z)+exp_x*sinf(angle_z);

//	x_ref += x*cosf(axisz_ref)-y*sinf(axisz_ref);
//	y_ref += y*cosf(axisz_ref)+x*sinf(axisz_ref);
}

void q_ref(float xx, float yy, float zz)
    {
	float qa, recipNorm;
	float x, y, max, sqrtxy;
	float q0x, q1x;
	float q0y, q2y;
	static float q0z = 1.0f, q3z = 0.0f;
	float ez;
	if (throtle_ppm > 0.03f)
	    {
		if ((zz < -DEAD_ZONE_YAW_STICK)||(zz > DEAD_ZONE_YAW_STICK))
		    {
			qa = q0z;
			zz *= 0.5f*DT_CYCLE_MPU;
			q0z +=q3z*zz;
			q3z +=-qa*zz;
			axisz_ref -= zz*2.0f;
			if (axisz_ref > M_PI) axisz_ref -= 2.0f*M_PI;
			if (axisz_ref < -M_PI) axisz_ref += 2.0f*M_PI;
		    }

	    }
	else
	{
		if ((!(supervisorState & STATE_FLYING)))
		    {
			ez = -atan2f((qmah1*qmah2-qmah0*qmah3),(qmah0*qmah0+qmah2*qmah2-0.5f));
			axisz_ref = ez;
			xSemaphoreTake(xUKF_PID_Mutex, portMAX_DELAY);
			ref_alt_hold = DataUKF.tru_pos_z;
			x_ref = DataUKF.posx;
			y_ref = DataUKF.posy;
			xSemaphoreGive(xUKF_PID_Mutex);
			q0z = cosf(ez/2.0f);
			q3z = sinf(ez/2.0f);
		    }
	}

	q0x = cosf(xx/2.0f);
	q1x = -sinf(xx/2.0f);

	q0y = cosf(yy/2.0f);
	q2y = -sinf(yy/2.0f);

	recipNorm = 1.0f/sqrtf(q0x*q0x + q1x*q1x);
	q0_ref_x = q0x*recipNorm;
	q1_ref_x = q1x*recipNorm;

	recipNorm = 1.0f/sqrtf(q0y*q0y + q2y*q2y);
	q0_ref_y = q0y*recipNorm;
	q2_ref_y = q2y*recipNorm;

	recipNorm = 1.0f/sqrtf(q0z*q0z + q3z*q3z);
	q0_ref_z = q0z*recipNorm;
	q3_ref_z = q3z*recipNorm;

    }
void q_err(void)
    {
	float q0x, q1x, q0ex, q1ex;
	float q0y, q2y, q0ey, q2ey;
	float q0z, q3z, q0ez, q3ez;
	float q0_y, q1_y, q2_y, q3_y;
	float ex, ey, ez_;

	float cx, sx, cy, sy;

	float e_x, e_y, e_z;
	float emixx, emixy, emixz;

	float ql0, ql1, ql2, ql3;
//	xSemaphoreTake(xUKF_PID_Mutex, portMAX_DELAY);
//	ql0 = q0;
//	ql1 = q1;
//	ql2 = q2;
//	ql3 = q3;
//	xSemaphoreGive(xUKF_PID_Mutex);

	ql0 = qmah0;
	ql1 = qmah1;
	ql2 = qmah2;
	ql3 = qmah3;

	//находим глобальные тангаж и рыскание
	ex = atan2f((ql2*ql3+ql0*ql1), (ql0*ql0+ql3*ql3-0.5f));
	ez_ = -atan2f((ql1*ql2-ql0*ql3),(ql0*ql0+ql2*ql2-0.5f));

	//создаем кватернион рыскани€
	q0z = cosf(ez_/2.0f);
	q3z = -sinf(ez_/2.0f);
	//дл€ оси ” придетс€ повозитьс€, убираем вращение по оси Z из глобального кватерниона
	q0_y = q0z*ql0  - q3z*ql3;
	q1_y = q0z*ql1 - q3z*ql2;
	q2_y = q0z*ql2  + q3z*ql1;
	q3_y = q0z*ql3  + q3z*ql0;

	//находим глобальный крен
	ey = -atan2f((q3_y*q1_y+q2_y*q0_y), (q0_y*q0_y+q3_y*q3_y-0.5f));

	axisz2 = ez_;
	axisy2 = ey;
	axisx2 = ex;
	//создаем кватернион тангажа
	q0x = cosf(ex/2.0f);
	q1x = -sinf(ex/2.0f);
	//создаем кватернион крена
	q0y = cosf(ey/2.0f);
	q2y = -sinf(ey/2.0f);
	//находим кватернион ошибки по оси ’
	q0ex = q0_ref_x*q0x - q1_ref_x*q1x;
	q1ex = q0_ref_x*q1x + q1_ref_x*q0x;
	//находим кватернион ошибки по оси Y
	q0ey = q0_ref_y*q0y - q2_ref_y*q2y;
	q2ey = q0_ref_y*q2y + q2_ref_y*q0y;
	//находим кватернион ошибки по оси Z
	q0ez = q0_ref_z*q0z - q3_ref_z*q3z;
	q3ez = q0_ref_z*q3z + q3_ref_z*q0z;

	//теперь имеем ошибки относительно текущего положени€ и задани€
	e_x = atan2f((q0ex*q1ex), (q0ex*q0ex-0.5f));
	e_y = atan2f((q0ey*q2ey), (q0ey*q0ey-0.5f));
	e_z = -atan2f((-q0ez*q3ez),(q0ez*q0ez-0.5f));

	if (mq9 > 0.0f)
	    {
		cx = cosf(axisx2);
		sx = sinf(axisx2);
		cy = cosf(axisy2);
		sy = sinf(axisy2);
		//общий по глобальным ос€м
//		e_mix_x = e_x*cy + e_z*sy;
//		e_mix_y = e_y*cx - e_z*sx;
//		e_mix_z = e_z*mq9 - e_x*sy + e_y*sx;

		//локальные X и Y, глобальнаое Z
//		emixx = e_x*cy - e_z*sy;//e_mix_x = e_x*cy + e_z*sy;
//		emixy = e_y*cx - e_z*sx;
//		emixz = e_z*mq9;

		emixx = e_x;
		emixy = e_y;
		emixz = e_z;


//		e_mix_x = e_x*cosf(axisy)+e_z*sinf(axisy);
//		e_mix_y = e_y*cosf(axisx)+e_z*sinf(-axisx);
//		e_mix_z = e_z*mq9;
	    }
	else
	    {
		emixx = e_x;
		emixy = e_y;
		emixz = e_z;
	    }
	e_mix_x = emixx;
	e_mix_y = emixy;
	e_mix_z = emixz;


    }
void Mixer(float u_x, float u_y, float u_z, float thr, uint8_t arm, enum Modes mod)
    {
	uint32_t M1_lev_front_, M2_prav_back_, M3_prav_front_, M4_lev_back_;
	float m1_l_f, m2_p_b, m3_p_f, m4_l_b;
	float saturation, maxmot;
	static float throttleLimiter = 0.0f;

	// calculate voltage factor
	float voltageFactor = 1.0f;
	float nominalBatVolts = 14.8f;
	xSemaphoreTake(xADC_Mutex, portMAX_DELAY);
	voltageFactor = 1.0f + (nominalBatVolts - All_voltage_smooth) / nominalBatVolts;
	xSemaphoreGive(xADC_Mutex);
	if (arm&STATE_ARMED)
		{
		    if (mod == MODE_ESC_CALIB)//ESC calibration
			{
			    m1_l_f = thr;
			    m2_p_b = thr;
			    m3_p_f = thr;
			    m4_l_b = thr;
			}
		    else
			{
			    //if (thr > 85.0f) thr = 85.0f;//ограничение, надо переделать
			    //if (thr < 0.0f) thr = 0.0f;

			    //thr = constrainFloat(thr - throttleLimiter, 0.0f, 100.0f);

			    if (thr > 5.0f)
				{

				    m1_l_f = (-u_y)*0.7071f+(u_x)*0.7071f+(-u_z)+thr;
				    m2_p_b = (u_y)*0.7071f+(-u_x)*0.7071f+(-u_z)+thr;
				    m3_p_f = (u_y)*0.7071f+(u_x)*0.7071f+(u_z)+thr;
				    m4_l_b = (-u_y)*0.7071f+(-u_x)*0.7071f+(u_z)+thr;



//				    if (m1_l_f > 100.0f) throttleLimiter += 3.663e-3f;
//				    if (m2_p_b > 100.0f) throttleLimiter += 3.663e-3f;
//				    if (m3_p_f > 100.0f) throttleLimiter += 3.663e-3f;
//				    if (m4_l_b > 100.0f) throttleLimiter += 3.663e-3f;

				    //throttleLimiter = constrainFloat(throttleLimiter - 3.663e-3f, 0.0f, 25.0f);

				    maxmot = MAX(MAX(m1_l_f, m2_p_b),MAX(m3_p_f, m4_l_b));
				    saturation = maxmot - 100.0f;
				    if (saturation > 0.0f)
					{
					    thr -= saturation;

					    m1_l_f = (-u_y)*0.7071f+(u_x)*0.7071f+(-u_z)+thr;
					    m2_p_b = (u_y)*0.7071f+(-u_x)*0.7071f+(-u_z)+thr;
					    m3_p_f = (u_y)*0.7071f+(u_x)*0.7071f+(u_z)+thr;
					    m4_l_b = (-u_y)*0.7071f+(-u_x)*0.7071f+(u_z)+thr;
					}

				    m1_l_f *= voltageFactor;
				    m2_p_b *= voltageFactor;
				    m3_p_f *= voltageFactor;
				    m4_l_b *= voltageFactor;

				    if (m1_l_f < 0.0f) m1_l_f = 0.0f;
				    if (m1_l_f> 100.0f) m1_l_f = 100.0f;

				    if (m2_p_b < 0.0f) m2_p_b = 0.0f;
				    if (m2_p_b > 100.0f) m2_p_b = 100.0f;

				    if (m3_p_f < 0.0f) m3_p_f = 0.0f;
				    if (m3_p_f > 100.0f) m3_p_f = 100.0f;

				    if (m4_l_b < 0.0f) m4_l_b = 0.0f;
				    if (m4_l_b > 100.0f) m4_l_b = 100.0f;
				}
			    else
				{
				    m1_l_f = 0.0f;
				    m2_p_b = 0.0f;
				    m3_p_f = 0.0f;
				    m4_l_b = 0.0f;
				}
			}
		}
	    else
		{
		    m1_l_f = -5.0f;
		    m2_p_b = -5.0f;
		    m3_p_f = -5.0f;
		    m4_l_b = -5.0f;
		}

	    M1_lev_front_=(uint32_t)(m1_l_f*22840.0f/((float)MAX_PID_vel_xy)+28550.0f);//490hz
	    M2_prav_back_=(uint32_t)(m2_p_b*22840.0f/((float)MAX_PID_vel_xy)+28550.0f);
	    M3_prav_front_=(uint32_t)(m3_p_f*22840.0f/((float)MAX_PID_vel_xy)+28550.0f);
	    M4_lev_back_=(uint32_t)(m4_l_b*22840.0f/((float)MAX_PID_vel_xy)+28550.0f);

	    //защитить
	    taskENTER_CRITICAL();
	    M1_lev_front = M1_lev_front_;
	    M2_prav_back = M2_prav_back_;
	    M3_prav_front = M3_prav_front_;
	    M4_lev_back = M4_lev_back_;
	    taskEXIT_CRITICAL();

	    /////////////////////
    }
void Fast_mahony(float *acc, float *gyr, float *yVec )
    {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	static float integralFBx, integralFBy, integralFBz;

	const float twoKi = 0.2f, twoKp = 1.6f;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((acc[0] == 0.0f) && (acc[1] == 0.0f) && (acc[2] == 0.0f)))
	    {
		// Normalise accelerometer measurement
		recipNorm = 1.0f/sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
		acc[0] *= recipNorm;
		acc[1] *= recipNorm;
		acc[2] *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = qmah1*qmah3 - qmah0*qmah2;
		halfvy = qmah0*qmah1 + qmah2*qmah3;
		halfvz = qmah0*qmah0 - 0.5f + qmah3*qmah3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (acc[1] * halfvz - acc[2] * halfvy);
		halfey = (acc[2] * halfvx - acc[0] * halfvz);
		halfez = (acc[0] * halfvy - acc[1] * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f)
		    {
			integralFBx += twoKi*halfex*DT_CYCLE_MPU;	// integral error scaled by Ki
			integralFBy += twoKi*halfey*DT_CYCLE_MPU;
			integralFBz += twoKi*halfez*DT_CYCLE_MPU;
			gyr[0] += integralFBx;	// apply integral feedback
			gyr[1] += integralFBy;
			gyr[2] += integralFBz;
		    }
		else
		    {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		    }

		// Apply proportional feedback
		gyr[0] += twoKp * halfex;
		gyr[1] += twoKp * halfey;
		gyr[2] += twoKp * halfez;

		if(!((yVec[0] == 0.0f) && (yVec[1] == 0.0f) && (yVec[2] == 0.0f)))
		    {
			// Normalise vec measurement
			recipNorm = 1.0f/sqrtf(yVec[0]*yVec[0] + yVec[1]*yVec[1] + yVec[2]*yVec[2]);
			yVec[0] *= recipNorm;
			yVec[1] *= recipNorm;
			yVec[2] *= recipNorm;

			// Estimated direction of vector y and vector perpendicular to magnetic flux
			halfvx = qmah1*qmah2 + qmah0*qmah3;
			halfvy = 0.5f - qmah1*qmah1 - qmah3*qmah3;
			halfvz = qmah2*qmah3 - qmah0*qmah1;

			// Error is sum of cross product between estimated and measured direction of gravity
			halfex = (yVec[1] * halfvz - yVec[2] * halfvy);
			halfey = (yVec[2] * halfvx - yVec[0] * halfvz);
			halfez = (yVec[0] * halfvy - yVec[1] * halfvx);

			// Compute and apply integral feedback if enabled
			if(twoKi > 0.0f)
			    {
				integralFBx += twoKi/4.0f*halfex*DT_CYCLE_MPU;	// integral error scaled by Ki
				integralFBy += twoKi/4.0f*halfey*DT_CYCLE_MPU;
				integralFBz += twoKi/4.0f*halfez*DT_CYCLE_MPU;
				gyr[0] += integralFBx;	// apply integral feedback
				gyr[1] += integralFBy;
				gyr[2] += integralFBz;
			    }
			else
			    {
				integralFBx = 0.0f;	// prevent integral windup
				integralFBy = 0.0f;
				integralFBz = 0.0f;
			    }

			// Apply proportional feedback
			gyr[0] += twoKp/0.5f * halfex;
			gyr[1] += twoKp/0.5f * halfey;
			gyr[2] += twoKp/0.5f * halfez;

		    }





		// Integrate rate of change of quaternion
		gyr[0] *= (0.5f*DT_CYCLE_MPU);		// pre-multiply common factors
		gyr[1] *= (0.5f*DT_CYCLE_MPU);
		gyr[2] *= (0.5f*DT_CYCLE_MPU);
		qa = qmah0;
		qb = qmah1;
		qc = qmah2;
		qmah0 += (-qb * gyr[0] - qc * gyr[1] - qmah3 * gyr[2]);
		qmah1 += (qa * gyr[0] + qc * gyr[2] - qmah3 * gyr[1]);
		qmah2 += (qa * gyr[1] - qb * gyr[2] + qmah3 * gyr[0]);
		qmah3 += (qa * gyr[2] + qb * gyr[1] - qc * gyr[0]);

		// Normalise quaternion
		recipNorm = 1.0f/sqrtf(qmah0*qmah0 + qmah1*qmah1 + qmah2*qmah2 + qmah3*qmah3);
		qmah0 *= recipNorm;
		qmah1 *= recipNorm;
		qmah2 *= recipNorm;
		qmah3 *= recipNorm;


	    }

    }
void Control(void)
    {

	float control_action_x_, control_action_y_, control_action_z_, control_action_thr_;
	float pitch_ppm_, roll_ppm_, yaw_rate_ppm_, throtle_ppm_;
	enum Modes mode_;

	static uint8_t cnt_parc_tx, cnt_collect_pos_alt = 5;
	static uint8_t reset_i_pid_acc_z, reset_i_pid_xy;

	static float RefAnglStabRoll, RefAnglStabPitch;

	float velZ, velX, velY, posX, posY, posZ, gbx, gby, gbz;
	float yawcos, yawsin, anglz;

	float accukf[3], gyrukf[3], yvec[3];
	float ql0, ql1, ql2, ql3;
	float angleVecStart;

	static uint8_t flag_enter_auto = 0, flag_reset_auto = 0;
	float k_vel_auto_= 1.0f;

	if (!(StateNavUKF&UKF_NAN))
	    {
		xSemaphoreTake(xUKF_PID_Mutex, portMAX_DELAY);

		///////
//		DataUKF.posx = x_ref;
//		DataUKF.posy = y_ref;
//		DataUKF.tru_pos_z = ref_alt_hold;
		//////

		posX = DataUKF.posx;
		posY = DataUKF.posy;
		posZ = DataUKF.tru_pos_z;

		velX = DataUKF.velx;
		velY = DataUKF.vely;
		velZ = DataUKF.tru_vel_z;

		gbx = gxbias;
		gby = gybias;
		gbz = gzbias;

		yawcos = navUkfData.yawCos;
		yawsin = navUkfData.yawSin;

		ql0 = q0;
		ql1 = q1;
		ql2 = q2;
		ql3 = q3;

		anglz = axisz;



		xSemaphoreGive(xUKF_PID_Mutex);
	    }
	else {
		posX = 0.0f;
		posY = 0.0f;
		posZ = 0.0f;

		velX = 0.0f;
		velY = 0.0f;
		velZ = 0.0f;

		gbx = 0.0f;
		gby = 0.0f;
		gbz = 0.0f;

		yawcos = 0.0f;
		yawsin = 0.0f;

		ql0 = 0.0f;
		ql1 = 0.0f;
		ql2 = 0.0f;
		ql3 = 0.0f;

		anglz = 0.0f;
	}

	//быстрый ћахони дл€ отслеживани€ угла
	if ((!(StateNavUKF&UKF_NAN)||(StateNavUKF&UKF_GLITCH_GPS)))
	    {
		// Estimated direction of gravity and vector perpendicular to magnetic flux
		accukf[0] = ql1*ql3 - ql0*ql2;
		accukf[1] = ql0*ql1 + ql2*ql3;
		accukf[2] = ql0*ql0 - 0.5f + ql3*ql3;

		// Estimated direction of vector y and vector perpendicular to magnetic flux
		yvec[0] = ql1*ql2 + ql0*ql3;
		yvec[1] = 0.5f - ql1*ql1 - ql3*ql3;
		yvec[2] = ql2*ql3 - ql0*ql1;

//		yvec[0] = 0.0f;
//		yvec[1] = 0.0f;
//		yvec[2] = 0.0f;

		gyrukf[0] = gxf + gbx;
		gyrukf[1] = gyf + gby;
		gyrukf[2] = gzf + gbz;


	    }
	else
	    {
		accukf[0] = axf;
		accukf[1] = ayf;
		accukf[2] = azf;

		yvec[0] = 0.0f;
		yvec[1] = 0.0f;
		yvec[2] = 0.0f;

		gyrukf[0] = gxf;
		gyrukf[1] = gyf;
		gyrukf[2] = gzf;
	    }
	Fast_mahony(accukf, gyrukf, yvec);

	//защита данных с приемника (прерывание PWM_Reciever)
	taskENTER_CRITICAL();
	pitch_ppm_ = pitch_ppm;
	roll_ppm_ = roll_ppm;
	yaw_rate_ppm_ = yaw_rate_ppm;
	throtle_ppm_ = throtle_ppm;
	mode_ = Mode;
	k_vel_auto_ = k_vel_auto_ppm;
	taskEXIT_CRITICAL();

	RefAnglStabRoll = K_EXP_PPM_REF_XY*RefAnglStabRoll + (1.0f-K_EXP_PPM_REF_XY)*roll_ppm_;
	RefAnglStabPitch = K_EXP_PPM_REF_XY*RefAnglStabPitch + (1.0f-K_EXP_PPM_REF_XY)*pitch_ppm_;


	if (flagSavePoint)
	    {
		initPoint(posX, posY, posZ-alt_offset);
		//initPoint(x_ref, y_ref, ref_alt_hold);
		flagSavePoint = 0;
	    }

	if (mode_ == MODE_AUTO)
	    {
		if (flag_enter_auto == 0)
		    {
			flag_enter_auto = 1;
			ref_alt_hold -= alt_offset;
		    }

		if (((throtle_ppm_- 0.5f) > DEAD_ZONE_ALT_STICK)||((throtle_ppm_- 0.5f) < -DEAD_ZONE_ALT_STICK))
		    {
			alt_offset += (throtle_ppm_ - 0.5f)*DT_CYCLE_MPU*5.0f;
			buzzer_flag = BUZZER_FLAG_LIFT_COMMAND;
		    }

		//firstEnterPoint(posX, posY, posZ-alt_offset, velX, velY, velZ);

		//firstEnterPoint(x_ref, y_ref, ref_alt_hold, 0.0f, 0.0f, 0.0f);



		//cycleLerp(&x_ref, &y_ref, &ref_alt_hold, posX, posY, posZ-alt_offset, 0.0f, 0.0f, 0.0f);

		//cycleLerp(&x_ref, &y_ref, &ref_alt_hold, x_ref, y_ref, ref_alt_hold, 0.0f, 0.0f, 0.0f);



		//DynamicLerp2(&x_ref, &y_ref, &ref_alt_hold, x_ref, y_ref, ref_alt_hold, flag_reset_auto, k_vel_auto_);
		DynamicLerp2(&x_ref, &y_ref, &ref_alt_hold, posX, posY, posZ-alt_offset, flag_reset_auto, k_vel_auto_);
		flag_reset_auto = 0;

		Ref_for_vel_alt_z = PID_Alt_Pos_Z(ref_alt_hold+alt_offset, posZ);
		control_action_acc_alt_z = PID_Alt_Vel_Z(Ref_for_vel_alt_z, velZ, reset_i_pid_acc_z);
		float absmq9;
		mq9 = 2.0f*(qmah0*qmah0+qmah3*qmah3-0.5f);
		absmq9 = fabsf(mq9);
		absmq9 = (absmq9 < 0.3f) ? 0.3f : absmq9;
		control_action_thr_ = control_action_acc_alt_z/absmq9+50.0f;

		PID_Pos_XY(x_ref, posX, y_ref, posY, &Ref_for_vel_x, &Ref_for_vel_y);

		Ref_for_angle_Glob_x = PID_Vel_X(Ref_for_vel_x, velX, reset_i_pid_xy);
		Ref_for_angle_Glob_y = PID_Vel_Y(Ref_for_vel_y, velY, reset_i_pid_xy);

		Ref_for_ugol_y = Ref_for_angle_Glob_x*yawcos+Ref_for_angle_Glob_y*yawsin;
		Ref_for_ugol_x = Ref_for_angle_Glob_y*yawcos-Ref_for_angle_Glob_x*yawsin;

		Ref_for_ugol_y += RefAnglStabRoll*cosf(anglz-axisz_ref) + RefAnglStabPitch*sinf(anglz-axisz_ref);;
		Ref_for_ugol_x += RefAnglStabPitch*cosf(anglz-axisz_ref) - RefAnglStabRoll*sinf(anglz-axisz_ref);;




		reset_i_pid_xy = 0;
		reset_i_pid_acc_z = 0;
	    }
	else
	    {
		exitPoint();
		if (flag_enter_auto == 1)
		    {
			flag_enter_auto = 0;
			ref_alt_hold += alt_offset;
		    }
		flag_reset_auto = 1;
	    }


	if ((mode_ == MODE_ALT_HOLD)||(mode_ == MODE_POS_HOLD)||(mode_ == MODE_HEADFREE))
	    {
		if (((throtle_ppm_- 0.5f) > DEAD_ZONE_ALT_STICK)||((throtle_ppm_- 0.5f) < -DEAD_ZONE_ALT_STICK))
		    {
			ref_alt_hold += (throtle_ppm_ - 0.5f)*DT_CYCLE_MPU*5.0f;
			buzzer_flag = BUZZER_FLAG_LIFT_COMMAND;
		    }
		Ref_for_vel_alt_z = PID_Alt_Pos_Z(ref_alt_hold, posZ);
		control_action_acc_alt_z = PID_Alt_Vel_Z(Ref_for_vel_alt_z, velZ, reset_i_pid_acc_z);
		float absmq9;
		mq9 = 2.0f*(qmah0*qmah0+qmah3*qmah3-0.5f);
		absmq9 = fabsf(mq9);
		absmq9 = (absmq9 < 0.3f) ? 0.3f : absmq9;
		control_action_thr_ = control_action_acc_alt_z/absmq9+50.0f;

		control_action_thr_ = constrainFloat(control_action_thr_, 10.0f, 100.0f);

		if (mode_ == MODE_ALT_HOLD)
		    {
			x_ref = posX;
			y_ref = posY;
			//reset_i_pid_xy = 1;
		    }

		if ((mode_ == MODE_POS_HOLD)||(mode_ == MODE_HEADFREE))
		    {
			if (mode_ == MODE_POS_HOLD)
			    X_Y_Ref_Cartesian(roll_ppm_, pitch_ppm_, DT_CYCLE_MPU, axisz_ref);
			if (mode_ == MODE_HEADFREE)
			    {

				float dist;
				float temp1, temp2;

				temp1 = (x_ref-GPS_X_home);
				temp2 = (y_ref-GPS_Y_home);
				dist = sqrtf(temp1*temp1 + temp2*temp2);

				angleVecStart = atan2f(temp2,temp1) - M_PI_2;

				if((dist < 3.0f)&&(pitch_ppm_<0.0f))
				    pitch_ppm_ = 0.0f;


				X_Y_Ref_Cartesian(roll_ppm_, pitch_ppm_, DT_CYCLE_MPU, angleVecStart);

			    }
			PID_Pos_XY(x_ref, posX, y_ref, posY, &Ref_for_vel_x, &Ref_for_vel_y);

			Ref_for_angle_Glob_x = PID_Vel_X(Ref_for_vel_x, velX, reset_i_pid_xy);
			Ref_for_angle_Glob_y = PID_Vel_Y(Ref_for_vel_y, velY, reset_i_pid_xy);

			Ref_for_ugol_y = Ref_for_angle_Glob_x*yawcos+Ref_for_angle_Glob_y*yawsin;
			Ref_for_ugol_x = Ref_for_angle_Glob_y*yawcos-Ref_for_angle_Glob_x*yawsin;

			if (mode_ == MODE_POS_HOLD)
			    {
				Ref_for_ugol_y += RefAnglStabRoll*cosf(anglz-axisz_ref) + RefAnglStabPitch*sinf(anglz-axisz_ref);;
				Ref_for_ugol_x += RefAnglStabPitch*cosf(anglz-axisz_ref) - RefAnglStabRoll*sinf(anglz-axisz_ref);;
			    }
			if (mode_ == MODE_HEADFREE)
			    {
				Ref_for_ugol_y += RefAnglStabRoll*cosf(anglz-angleVecStart) + RefAnglStabPitch*sinf(anglz-angleVecStart);
				Ref_for_ugol_x += RefAnglStabPitch*cosf(anglz-angleVecStart) - RefAnglStabRoll*sinf(anglz-angleVecStart);
			    }

			reset_i_pid_xy = 0;
		    }


		reset_i_pid_acc_z = 0;

	    }
	if ((mode_ == MODE_STAB)||(mode_ == MODE_ESC_CALIB))
	    {
		reset_i_pid_acc_z = 1;
		reset_i_pid_xy = 1;
		x_ref = posX;
		y_ref = posY;
		ref_alt_hold = posZ;
		control_action_thr_ = throtle_ppm_*MAX_PID_vel_xy;
	    }
	/////////////////

	if ((mode_ == MODE_STAB)||(mode_ == MODE_ALT_HOLD))
	    {
		Ref_for_ugol_x = RefAnglStabPitch;
		Ref_for_ugol_y = RefAnglStabRoll;
	    }

	yaw_rate_ppm_exp = K_EXP_PPM_REF_UGOL_Z*yaw_rate_ppm_exp+(1.0f-K_EXP_PPM_REF_UGOL_Z)*yaw_rate_ppm_;

	q_ref(Ref_for_ugol_x, Ref_for_ugol_y, yaw_rate_ppm_exp);
	q_err();

	Ref_for_rate_y = PID_Angle_Y(e_mix_y, throtle_ppm_);
	control_action_y_ = PID_Rate_Y(Ref_for_rate_y, -(gyf+gby), throtle_ppm_);

	Ref_for_rate_x = PID_Angle_X(e_mix_x, throtle_ppm_);
	control_action_x_ = PID_Rate_X(Ref_for_rate_x, (gxf+gbx), throtle_ppm_);

	Ref_for_rate_z = -PID_Angle_Z(e_mix_z, throtle_ppm_);
	control_action_z_ = PID_Rate_Z(Ref_for_rate_z, -(gzf+gbz), throtle_ppm_);




	Mixer(control_action_x_, control_action_y_, control_action_z_, control_action_thr_, (uint8_t)supervisorState, mode_);

	control_action_x = control_action_x_;
	control_action_y = control_action_y_;
	control_action_z = control_action_z_;
	control_action_thr = control_action_thr_;

	xSemaphoreGive(xSD_PID_collect_Semaphore);

	ovf_ins_stack = uxTaskGetStackHighWaterMark(ins_handle);
    }
void prvIMU_INS(void *pvParameters)
    {
	int16_t ax1, ay1, az1, gx1, gy1, gz1, temp_gyro1;
	static int16_t ax_last, ay_last, az_last, gx_last, gy_last, gz_last;
	static uint8_t first=0, one = 0;
	int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
	float ax_b, ay_b, az_b;

	static float K_gx, K_gy, K_gz;
	static int32_t gxs, gys, gzs;
	static uint16_t Counter_Gyro_calib;

	float Axf, Ayf, Azf, Gxf, Gyf, Gzf;


	while(1)
	    {
		xSemaphoreTake(xMPU_Semaphore, portMAX_DELAY);

		cnt_cycle_ins = (uint32_t)DWT->CYCCNT - cnt_ticks_ins;
		cnt_ticks_ins = (uint32_t)DWT->CYCCNT;

		ax1 = (int16_t)(((uint16_t)MPU_buff[0]<<8)+MPU_buff[1]);
		ay1 = (int16_t)(((uint16_t)MPU_buff[2]<<8)+MPU_buff[3]);
		az1 = (int16_t)(((uint16_t)MPU_buff[4]<<8)+MPU_buff[5]);

		gx1 = (int16_t)(((uint16_t)MPU_buff[8]<<8)+MPU_buff[9]);
		gy1 = (int16_t)(((uint16_t)MPU_buff[10]<<8)+MPU_buff[11]);
		gz1 = (int16_t)(((uint16_t)MPU_buff[12]<<8)+MPU_buff[13]);

		temp_gyro1 = (int16_t)(((uint16_t)MPU_buff[6]<<8)+MPU_buff[7]);

		if (first < 100)
		    {
			ax_last = ax1;
			ay_last = ay1;
			az_last = az1;
			gx_last = gx1;
			gy_last = gy1;
			gz_last = gz1;
			first++;
		    }
		if (ax1>ax_last) ax_r = ax1 - ax_last; else ax_r = ax_last - ax1;
		if (ay1>ay_last) ay_r = ay1 - ay_last; else ay_r = ay_last - ay1;
		if (az1>az_last) az_r = az1 - az_last; else az_r = az_last - az1;
		if (gx1>gx_last) gx_r = gx1 - gx_last; else gx_r = gx_last - gx1;
		if (gy1>gy_last) gy_r = gy1 - gy_last; else gy_r = gy_last - gy1;
		if (gz1>gz_last) gz_r = gz1 - gz_last; else gz_r = gz_last - gz1;
		if (((ax_r > 20000)||(ay_r > 20000)||(az_r > 20000)||(gx_r > 20000)||(gy_r > 20000)||(gz_r > 20000))&&one==0)
		    {
			ax = ax_last;
			ay = ay_last;
			az = az_last;
			gx = gx_last;
			gy = gy_last;
			gz = gz_last;
			one = 1;
			//counter_throw++;
		    }
		else
		{
			ax = ax1;
			ay = ay1;
			az = az1;
			gx = gx1;
			gy = gy1;
			gz = gz1;
			temp_gyro = ((float)temp_gyro1)/333.87f+21.0f;
			ax_last = ax1;
			ay_last = ay1;
			az_last = az1;
			gx_last = gx1;
			gy_last = gy1;
			gz_last = gz1;
			one = 0;
		}
		ax_sr = LPF_ACC_SR*ax_sr + (1.0f-LPF_ACC_SR)*(float)ax;
		ay_sr = LPF_ACC_SR*ay_sr + (1.0f-LPF_ACC_SR)*(float)ay;
		az_sr = LPF_ACC_SR*az_sr + (1.0f-LPF_ACC_SR)*(float)az;

	    	ax_b = (float)ax - ax_bias;
	    	ay_b = (float)ay - ay_bias;
	    	az_b = (float)az - az_bias;



//	    	Axf = am1*ax_b+am2*ay_b+am3*az_b;
//	    	Ayf = am4*ax_b+am5*ay_b+am6*az_b;
//	    	Azf = am7*ax_b+am8*ay_b+am9*az_b;
		Axf = am1*ax_b+am2*ay_b+am3*az_b;
		Ayf = am2*ax_b+am5*ay_b+am6*az_b;
		Azf = am3*ax_b+am6*ay_b+am9*az_b;

	    	Axf /=4096.0f;
	    	Ayf /=4096.0f;
	    	Azf /=4096.0f;

	    	Axf *=GRAVITY;
	    	Ayf *=GRAVITY;
	    	Azf *=GRAVITY;
	    	xSemaphoreTake(xMPU_UKF_Mutex, portMAX_DELAY);
	    	axf = Axf;
	    	ayf = Ayf;
	    	azf = Azf;
	    	xSemaphoreGive(xMPU_UKF_Mutex);
		if (Counter_Gyro_calib < 400)
		    {
			Counter_Gyro_calib++;
			gxs += gx;
			gys += gy;
			gzs += gz;
		    }
		else
		    {
			if (Counter_Gyro_calib == 400)
			    {
				K_gx = (double)gxs/(double)Counter_Gyro_calib;
				K_gy = (double)gys/(double)Counter_Gyro_calib;
				K_gz = (double)gzs/(double)Counter_Gyro_calib;
				flag_end_of_gyro_calib = 1;
				Counter_Gyro_calib++;
			    }
			Gxf = ((float)gx - K_gx)/K_GYRO;
			Gyf = ((float)gy - K_gy)/K_GYRO;
			Gzf = ((float)gz - K_gz)/K_GYRO;

			xSemaphoreTake(xMPU_UKF_Mutex, portMAX_DELAY);
			gxf = Gxf;
			gyf = Gyf;
			gzf = Gzf;
			xSemaphoreGive(xMPU_UKF_Mutex);


			Control();



		    }
	    }
    }
void runInit(void) {
    float acc[3], mag[3];
    float pres;
    int i;
    //taskENTER_CRITICAL();
	xSemaphoreTake(xMPU_UKF_Mutex, portMAX_DELAY);
    acc[0] = axf;
    acc[1] = ayf;
    acc[2] = azf;

    mag[0] = mxf;
    mag[1] = myf;
    mag[2] = mzf;

    pres = Altitude;
    xSemaphoreGive(xMPU_UKF_Mutex);
    //taskEXIT_CRITICAL();
    // initialize sensor history
    for (i = 0; i < RUN_SENSOR_HIST; i++) {
        runData.accHist[0][i] = acc[0];
        runData.accHist[1][i] = acc[1];
        runData.accHist[2][i] = acc[2];
        runData.magHist[0][i] = mag[0];
        runData.magHist[1][i] = mag[1];
        runData.magHist[2][i] = mag[2];
        runData.presHist[i] = pres;

        runData.sumAcc[0] += acc[0];
        runData.sumAcc[1] += acc[1];
        runData.sumAcc[2] += acc[2];
        runData.sumMag[0] += mag[0];
        runData.sumMag[1] += mag[1];
        runData.sumMag[2] += mag[2];
        runData.sumPres += pres;
    }

    runData.sensorHistIndex = 0;

    runData.bestHacc = 1000.0f;
    runData.accMask = 1000.0f;

    // use altUkf altitude & vertical velocity estimates to start with
    runData.altPos = &ALT_POS;
    runData.altVel = &ALT_VEL;

}

