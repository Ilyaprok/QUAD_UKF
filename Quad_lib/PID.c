#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "Config.h"

#include "PID.h"
#include "IMU_INS.h"
#include "PWM_PPM.h"



#define ON_INTEGRAL_PID		0.4f
#define OFF_INTEGRAL_PID	0.05f

//Угловые ПИДы
float KP_ugla_xy = 22.0f, KI_ugla_xy = 0.0f, KD_ugla_xy=0.0f;
float KP_rate_xy = 2.5f, KI_rate_xy = 2.0f, KD_rate_xy = 0.45f;

float KP_ugla_z = 9.0f, KI_ugla_z = 0.0f, KD_ugla_z = 0.0f;
float KP_rate_z = 14.0f, KI_rate_z = 0.7f, KD_rate_z = 0.4f;

float f_cut_diff_pid = 20.0f;

float control_action_y, Ref_for_rate_y, Ref_for_ugol_y;
float control_action_x, Ref_for_rate_x, Ref_for_ugol_x;
float control_action_z, Ref_for_rate_z, Ref_for_ugol_z;

float pid_rateX_i, pid_rateY_i, pid_rateZ_i;
//****************************************************//
//Высота
float KP_Pos_Alt_Z = 3.5f;
float KP_Vel_Alt_Z = 60.0f, KI_Vel_Alt_Z = 0.5f, KD_Vel_Alt_Z = 1.5f;

float KP_Acc_Alt_Z = 1.0f, KD_Acc_Alt_Z = 0.0f;

float f_cut_err_vel_alt_z = 4.0f;
float f_cut_diff_vel_alt_z = 6.0f;

float control_action_thr, Ref_for_vel_alt_z, Ref_for_acc_alt_z, control_action_acc_alt_z;
//****************************************************//
//X Y
float KP_Pos_XY = 1.25f;
float KP_Vel_XY = 0.2f, KI_Vel_XY = 0.06f, KD_Vel_XY = 0.01f;

float f_cut_err_vel_xy = 3.0f;
float f_cut_diff_vel_xy = 5.0f;

float Ref_for_vel_x, Ref_for_vel_y;
float Ref_for_angle_Glob_x, Ref_for_angle_Glob_y;

float pid_velX_i, pid_velY_i, pid_velZ_i;
//****************************************************//


float PID_Angle_Y (float Err_y_ugol, float thr)
    {
	float Dif_Err_y_ugol;
	static float Integ_y_ugol = 0, Last_Err_y_ugol = 0;
	static float Res_y_ugol;
	static float G_sr_y_ugol;
	float kp, ki, proport;
	//защитить
	taskENTER_CRITICAL();
	kp = KP_ugla_xy;
	ki = KI_ugla_xy;
	taskEXIT_CRITICAL();
	///////////////

	proport = kp*Err_y_ugol;


	if (thr > ON_INTEGRAL_PID)
	    {
		if ((Res_y_ugol < MAX_PID_ugla_xy)&&(Res_y_ugol > MIN_PID_ugla_xy)) Integ_y_ugol += Err_y_ugol*dt;
		if ((Res_y_ugol == MAX_PID_ugla_xy)&&(Err_y_ugol < 0.0f)) Integ_y_ugol += Err_y_ugol*dt;
		if ((Res_y_ugol <= MIN_PID_ugla_xy)&&(Err_y_ugol > 0.0f)) Integ_y_ugol += Err_y_ugol*dt;
	    }
	if (thr < OFF_INTEGRAL_PID) Integ_y_ugol = 0.0f;
	//else Integ_y_ugol = 0.0f;// возможно лучше закоментить

	Res_y_ugol = proport+ki*Integ_y_ugol;
	if(Res_y_ugol > MAX_PID_ugla_xy) Res_y_ugol = MAX_PID_ugla_xy;
	if(Res_y_ugol < MIN_PID_ugla_xy) Res_y_ugol = MIN_PID_ugla_xy;
	Last_Err_y_ugol = Err_y_ugol;
	return Res_y_ugol;
    }
float PID_Rate_Y (float ref, float real, float thr)
    {
	float Err_y_vel, Dif_Err_y_vel;
	static float Integ_y_vel = 0, Last_Err_y_vel = 0;
	static float Res_y_vel;
	static float G_sr_y_vel;
	float RC_filt, k_diff_exp;
	float kp, ki, kd, f_cut;
	//защитить
	taskENTER_CRITICAL();
	kp = KP_rate_xy;
	ki = KI_rate_xy;
	kd = KD_rate_xy;
	f_cut = f_cut_diff_pid;
	pid_rateY_i = Integ_y_vel;
	taskEXIT_CRITICAL();
	///////////////

	Err_y_vel = ref - real;
	if (thr > ON_INTEGRAL_PID)
	    {
		if ((Res_y_vel < MAX_PID_vel_xy)&&(Res_y_vel > MIN_PID_vel_xy)) Integ_y_vel += Err_y_vel*dt;
		if ((Res_y_vel == MAX_PID_vel_xy)&&(Err_y_vel < 0.0f)) Integ_y_vel += Err_y_vel*dt;
		if ((Res_y_vel <= MIN_PID_vel_xy)&&(Err_y_vel > 0.0f)) Integ_y_vel += Err_y_vel*dt;
	    }
	if (thr < OFF_INTEGRAL_PID) Integ_y_vel = 0.0f;
	//else Integ_y_vel = 0.0f;// возможно лучше закоментить

	Dif_Err_y_vel = (Err_y_vel - Last_Err_y_vel)/dt;
	RC_filt = 1.0f/(2.0f*M_PI*f_cut);
	k_diff_exp = dt/(RC_filt+dt);
	G_sr_y_vel = (1.0f-k_diff_exp)*G_sr_y_vel + k_diff_exp*Dif_Err_y_vel;
	//G_sr_y_vel = 0.9f*G_sr_y_vel + (1.0f-0.9f)*Dif_Err_y_vel;

	Res_y_vel = kp*Err_y_vel+ki*Integ_y_vel+kd*G_sr_y_vel;
	if(Res_y_vel > MAX_PID_vel_xy) Res_y_vel = MAX_PID_vel_xy;
	if(Res_y_vel < MIN_PID_vel_xy) Res_y_vel = MIN_PID_vel_xy;
	Last_Err_y_vel = Err_y_vel;
	return Res_y_vel;
    }
float PID_Angle_X (float Err_x_ugol, float thr)
    {
	float Dif_Err_x_ugol;
	static float Integ_x_ugol = 0, Last_Err_x_ugol = 0;
	static float Res_x_ugol;
	static float G_sr_x_ugol;
	float kp, ki, proport;
	//защитить
	taskENTER_CRITICAL();
	kp = KP_ugla_xy;
	ki = KI_ugla_xy;
	taskEXIT_CRITICAL();
	///////////////


	proport = kp*Err_x_ugol;


	if (thr > ON_INTEGRAL_PID)
	    {
		if ((Res_x_ugol < MAX_PID_ugla_xy)&&(Res_x_ugol > MIN_PID_ugla_xy)) Integ_x_ugol += Err_x_ugol*dt;
		if ((Res_x_ugol == MAX_PID_ugla_xy)&&(Err_x_ugol < 0.0f)) Integ_x_ugol += Err_x_ugol*dt;
		if ((Res_x_ugol <= MIN_PID_ugla_xy)&&(Err_x_ugol > 0.0f)) Integ_x_ugol += Err_x_ugol*dt;
	    }
	if (thr < OFF_INTEGRAL_PID) Integ_x_ugol = 0.0f;
	//else Integ_x_ugol = 0.0f;// возможно лучше закоментить

	Res_x_ugol = proport+ki*Integ_x_ugol;
	if(Res_x_ugol > MAX_PID_ugla_xy) Res_x_ugol = MAX_PID_ugla_xy;
	if(Res_x_ugol < MIN_PID_ugla_xy) Res_x_ugol = MIN_PID_ugla_xy;
	Last_Err_x_ugol = Err_x_ugol;
	return Res_x_ugol;
    }
float PID_Rate_X (float ref, float real, float thr)
    {
	float Err_x_vel, Dif_Err_x_vel;
	static float Integ_x_vel = 0, Last_Err_x_vel = 0;
	static float Res_x_vel;
	static float G_sr_x_vel;
	float RC_filt, k_diff_exp;
	float kp, ki, kd, f_cut;
	//защитить
	taskENTER_CRITICAL();
	kp = KP_rate_xy;
	ki = KI_rate_xy;
	kd = KD_rate_xy;
	f_cut = f_cut_diff_pid;
	pid_rateX_i = Integ_x_vel;
	taskEXIT_CRITICAL();
	///////////////
	Err_x_vel = ref - real;
	if (thr > ON_INTEGRAL_PID)
	    {
		if ((Res_x_vel < MAX_PID_vel_xy)&&(Res_x_vel > MIN_PID_vel_xy)) Integ_x_vel += Err_x_vel*dt;
		if ((Res_x_vel == MAX_PID_vel_xy)&&(Err_x_vel < 0.0f)) Integ_x_vel += Err_x_vel*dt;
		if ((Res_x_vel <= MIN_PID_vel_xy)&&(Err_x_vel > 0.0f)) Integ_x_vel += Err_x_vel*dt;
	    }
	if (thr < OFF_INTEGRAL_PID) Integ_x_vel = 0.0f;
	//else Integ_x_vel = 0.0f;// возможно лучше закоментить

	Dif_Err_x_vel = (Err_x_vel - Last_Err_x_vel)/dt;
	RC_filt = 1.0f/(2.0f*M_PI*f_cut);
	k_diff_exp = dt/(RC_filt+dt);
	G_sr_x_vel = (1.0f-k_diff_exp)*G_sr_x_vel + k_diff_exp*Dif_Err_x_vel;

	Res_x_vel = kp*Err_x_vel+ki*Integ_x_vel+kd*G_sr_x_vel;
	if(Res_x_vel > MAX_PID_vel_xy) Res_x_vel = MAX_PID_vel_xy;
	if(Res_x_vel < MIN_PID_vel_xy) Res_x_vel = MIN_PID_vel_xy;
	Last_Err_x_vel = Err_x_vel;
	return Res_x_vel;
    }
float PID_Angle_Z (float Err_z_ugol, float thr)
    {
	float Dif_Err_z_ugol;
	static float Integ_z_ugol = 0, Last_Err_z_ugol = 0;
	static float Res_z_ugol;
	static float G_sr_z_ugol;
	float kp, ki, proport;
	//защитить
	taskENTER_CRITICAL();
	kp = KP_ugla_z;
	ki = KI_ugla_z;
	taskEXIT_CRITICAL();
	///////////////

	proport = kp*Err_z_ugol;


	if (thr > ON_INTEGRAL_PID)
	    {
		if ((Res_z_ugol < MAX_PID_ugla_z)&&(Res_z_ugol > MIN_PID_ugla_z)) Integ_z_ugol += Err_z_ugol*dt;
		if ((Res_z_ugol == MAX_PID_ugla_z)&&(Err_z_ugol < 0.0f)) Integ_z_ugol += Err_z_ugol*dt;
		if ((Res_z_ugol <= MIN_PID_ugla_z)&&(Err_z_ugol > 0.0f)) Integ_z_ugol += Err_z_ugol*dt;
	    }
	if (thr < OFF_INTEGRAL_PID) Integ_z_ugol = 0.0f;
	//else Integ_z_ugol = 0.0f;// возможно лучше закоментить
	Res_z_ugol = proport+ki*Integ_z_ugol;
	if(Res_z_ugol > MAX_PID_ugla_z) Res_z_ugol = MAX_PID_ugla_z;
	if(Res_z_ugol < MIN_PID_ugla_z) Res_z_ugol = MIN_PID_ugla_z;
	Last_Err_z_ugol = Err_z_ugol;
	return Res_z_ugol;
    }
float PID_Rate_Z (float ref, float real, float thr)
    {
	float Err_z_vel, Dif_Err_z_vel;
	static float Integ_z_vel = 0, Last_Err_z_vel = 0;
	static float Res_z_vel;
	static float G_sr_z_vel;
	float RC_filt, k_diff_exp;
	float kp, ki, kd, f_cut;
	//защитить
	taskENTER_CRITICAL();
	kp = KP_rate_z;
	ki = KI_rate_z;
	kd = KD_rate_z;
	f_cut = f_cut_diff_pid;
	pid_rateZ_i = Integ_z_vel;
	taskEXIT_CRITICAL();
	///////////////
	Err_z_vel = ref - real;
	if (thr > ON_INTEGRAL_PID)
	    {
		if ((Res_z_vel < MAX_PID_vel_z)&&(Res_z_vel > MIN_PID_vel_z)) Integ_z_vel += Err_z_vel*dt;
		if ((Res_z_vel == MAX_PID_vel_z)&&(Err_z_vel < 0.0f)) Integ_z_vel += Err_z_vel*dt;
		if ((Res_z_vel <= MIN_PID_vel_z)&&(Err_z_vel > 0.0f)) Integ_z_vel += Err_z_vel*dt;
	    }
	if (thr < OFF_INTEGRAL_PID) Integ_z_vel = 0.0f;
	//else Integ_z_vel = 0.0f;// возможно лучше закоментить
	Dif_Err_z_vel = (Err_z_vel - Last_Err_z_vel)/dt;
	RC_filt = 1.0f/(2.0f*M_PI*f_cut);
	k_diff_exp = dt/(RC_filt+dt);
	G_sr_z_vel = (1.0f-k_diff_exp)*G_sr_z_vel + k_diff_exp*Dif_Err_z_vel;
	Res_z_vel = kp*Err_z_vel+ki*Integ_z_vel+kd*G_sr_z_vel;
	if(Res_z_vel > MAX_PID_vel_z) Res_z_vel = MAX_PID_vel_z;
	if(Res_z_vel < MIN_PID_vel_z) Res_z_vel = MIN_PID_vel_z;
	Last_Err_z_vel = Err_z_vel;
	return Res_z_vel;
    }

float PID_Alt_Pos_Z(float ref, float real)
    {
	float Err_pos_alt_z, Res_pos_alt_z;
	float kp;
	taskENTER_CRITICAL();
	kp = KP_Pos_Alt_Z;
	taskEXIT_CRITICAL();
	Err_pos_alt_z = ref - real;

	Res_pos_alt_z = kp*Err_pos_alt_z;


	if(Res_pos_alt_z > MAX_PID_pos_alt_z) Res_pos_alt_z = MAX_PID_pos_alt_z;
	if(Res_pos_alt_z < MIN_PID_pos_alt_z) Res_pos_alt_z = MIN_PID_pos_alt_z;
	return Res_pos_alt_z;
    }
float PID_Alt_Vel_Z(float ref, float real, uint8_t reset)
    {
	float Err_vel_alt_z, Res_vel_alt_z, Dif_Err_vel_alt_z;
	static float Last_Err_vel_alt_z = 0;
	static float G_sr_z_vel_alt, G_sr_err, Integ_vel_alt_z;
	float kp, kd, ki, f_cut, f_cut2;
	float RC_filt, k_diff_exp;

	float err_fo_diff, err_sr_for_dif, k_for_diff = 0.4f;

	taskENTER_CRITICAL();
	kp = KP_Vel_Alt_Z;
	kd = KD_Vel_Alt_Z;
	f_cut = f_cut_diff_vel_alt_z;
	f_cut2 = f_cut_err_vel_alt_z;
	ki = KI_Vel_Alt_Z;
	pid_velZ_i = Integ_vel_alt_z;
	taskEXIT_CRITICAL();

	Err_vel_alt_z = ref - real;

	RC_filt = 1.0f/(2.0f*M_PI*f_cut2);
	k_diff_exp = dt/(RC_filt+dt);

	G_sr_err = G_sr_err*(1.0f - k_diff_exp) + k_diff_exp*Err_vel_alt_z;

	if (reset == 0)
	    {
		if ((Res_vel_alt_z < MAX_PID_vel_alt_z)&&(Res_vel_alt_z > MIN_PID_vel_alt_z)) Integ_vel_alt_z += ki*G_sr_err*dt;
		if ((Res_vel_alt_z == MAX_PID_vel_alt_z)&&(G_sr_err < 0.0f)) Integ_vel_alt_z += ki*G_sr_err*dt;
		if ((Res_vel_alt_z <= MIN_PID_vel_alt_z)&&(G_sr_err > 0.0f)) Integ_vel_alt_z += ki*G_sr_err*dt;
	    }
	else Integ_vel_alt_z = 0.0f;

	err_fo_diff = k_for_diff*ref - real;
	//err_sr_for_dif = err_sr_for_dif*K_EXP_ERR_VEL_ALT_Z + (1.0f - K_EXP_ERR_VEL_ALT_Z)*err_fo_diff;

	Dif_Err_vel_alt_z = (err_fo_diff - Last_Err_vel_alt_z)/dt;

	RC_filt = 1.0f/(2.0f*M_PI*f_cut);
	k_diff_exp = dt/(RC_filt+dt);
	G_sr_z_vel_alt = (1.0f-k_diff_exp)*G_sr_z_vel_alt + k_diff_exp*Dif_Err_vel_alt_z;

	Res_vel_alt_z = kp*G_sr_err+kd*G_sr_z_vel_alt+Integ_vel_alt_z;
	if(Res_vel_alt_z > MAX_PID_vel_alt_z) Res_vel_alt_z = MAX_PID_vel_alt_z;
	if(Res_vel_alt_z < MIN_PID_vel_alt_z) Res_vel_alt_z = MIN_PID_vel_alt_z;
	Last_Err_vel_alt_z = err_fo_diff;
	return Res_vel_alt_z;
    }
float PID_Pos_XY(float ref_x, float real_x, float ref_y, float real_y, float *res_x, float *res_y)
    {
	float Err_pos_x, Res_pos_x;
	float Err_pos_y, Res_pos_y;
	float kp;
	taskENTER_CRITICAL();
	kp = KP_Pos_XY;
	taskEXIT_CRITICAL();
	Err_pos_x = ref_x - real_x;
	Err_pos_y = ref_y - real_y;

	Res_pos_x = kp*Err_pos_x;
	Res_pos_y = kp*Err_pos_y;

	if(Res_pos_x > MAX_PID_pos_XY) Res_pos_x = MAX_PID_pos_XY;
	if(Res_pos_x < -MAX_PID_pos_XY) Res_pos_x = -MAX_PID_pos_XY;

	if(Res_pos_y > MAX_PID_pos_XY) Res_pos_y = MAX_PID_pos_XY;
	if(Res_pos_y < -MAX_PID_pos_XY) Res_pos_y = -MAX_PID_pos_XY;

	*res_x = Res_pos_x;
	*res_y = Res_pos_y;

    }
float PID_Vel_X(float ref, float real, uint8_t reset)
    {
	float Err_vel_x, Res_vel_x, Dif_Err_vel_x;
	static float Last_Err_vel_x = 0;
	static float G_sr_z_vel_x, G_sr_err_x, Integ_vel_x;
	float kp, kd, ki, f_cut, f_cut2;
	float RC_filt, k_diff_exp;

	float err_fo_diff, err_sr_for_dif, k_for_diff = 0.2f;

	taskENTER_CRITICAL();
	kp = KP_Vel_XY;
	kd = KD_Vel_XY;
	f_cut = f_cut_diff_vel_xy;
	f_cut2 = f_cut_err_vel_xy;
	ki = KI_Vel_XY;
	pid_velX_i = Integ_vel_x;
	taskEXIT_CRITICAL();

	Err_vel_x = ref - real;

	RC_filt = 1.0f/(2.0f*M_PI*f_cut2);
	k_diff_exp = dt/(RC_filt+dt);

	G_sr_err_x = G_sr_err_x*(1.0f - k_diff_exp) + k_diff_exp*Err_vel_x;

	if (reset == 0)
	    {
		if ((Res_vel_x < MAX_PID_vel_XY)&&(Res_vel_x > -MAX_PID_vel_XY)) Integ_vel_x += ki*G_sr_err_x*dt;
		if ((Res_vel_x == MAX_PID_vel_XY)&&(G_sr_err_x < 0.0f)) Integ_vel_x += ki*G_sr_err_x*dt;
		if ((Res_vel_x <= -MAX_PID_vel_XY)&&(G_sr_err_x > 0.0f)) Integ_vel_x += ki*G_sr_err_x*dt;
	    }
	else Integ_vel_x = 0.0f;

	err_fo_diff = k_for_diff*ref - real;
	//err_sr_for_dif = err_sr_for_dif*K_EXP_ERR_VEL_ALT_Z + (1.0f - K_EXP_ERR_VEL_ALT_Z)*err_fo_diff;

	Dif_Err_vel_x = (err_fo_diff - Last_Err_vel_x)/dt;

	RC_filt = 1.0f/(2.0f*M_PI*f_cut);
	k_diff_exp = dt/(RC_filt+dt);
	G_sr_z_vel_x = (1.0f-k_diff_exp)*G_sr_z_vel_x + k_diff_exp*Dif_Err_vel_x;

	Res_vel_x = kp*G_sr_err_x+kd*G_sr_z_vel_x+Integ_vel_x;
	if(Res_vel_x > MAX_PID_vel_XY) Res_vel_x = MAX_PID_vel_XY;
	if(Res_vel_x < -MAX_PID_vel_XY) Res_vel_x = -MAX_PID_vel_XY;
	Last_Err_vel_x = err_fo_diff;
	return Res_vel_x;
    }
float PID_Vel_Y(float ref, float real, uint8_t reset)
    {
	float Err_vel_y, Res_vel_y, Dif_Err_vel_y;
	static float Last_Err_vel_y = 0;
	static float G_sr_z_vel_y, G_sr_err_y, Integ_vel_y;
	float kp, kd, ki, f_cut, f_cut2;
	float RC_filt, k_diff_exp;

	float err_fo_diff, err_sr_for_dif, k_for_diff = 0.2f;

	taskENTER_CRITICAL();
	kp = KP_Vel_XY;
	kd = KD_Vel_XY;
	f_cut = f_cut_diff_vel_xy;
	f_cut2 = f_cut_err_vel_xy;
	ki = KI_Vel_XY;
	pid_velY_i = Integ_vel_y;
	taskEXIT_CRITICAL();

	Err_vel_y = ref - real;

	RC_filt = 1.0f/(2.0f*M_PI*f_cut2);
	k_diff_exp = dt/(RC_filt+dt);

	G_sr_err_y = G_sr_err_y*(1.0f - k_diff_exp) + k_diff_exp*Err_vel_y;

	if (reset == 0)
	    {
		if ((Res_vel_y < MAX_PID_vel_XY)&&(Res_vel_y > -MAX_PID_vel_XY)) Integ_vel_y += ki*G_sr_err_y*dt;
		if ((Res_vel_y == MAX_PID_vel_XY)&&(G_sr_err_y < 0.0f)) Integ_vel_y += ki*G_sr_err_y*dt;
		if ((Res_vel_y <= -MAX_PID_vel_XY)&&(G_sr_err_y > 0.0f)) Integ_vel_y += ki*G_sr_err_y*dt;
	    }
	else Integ_vel_y = 0.0f;

	err_fo_diff = k_for_diff*ref - real;
	//err_sr_for_dif = err_sr_for_dif*K_EXP_ERR_VEL_ALT_Z + (1.0f - K_EXP_ERR_VEL_ALT_Z)*err_fo_diff;

	Dif_Err_vel_y = (err_fo_diff - Last_Err_vel_y)/dt;

	RC_filt = 1.0f/(2.0f*M_PI*f_cut);
	k_diff_exp = dt/(RC_filt+dt);
	G_sr_z_vel_y = (1.0f-k_diff_exp)*G_sr_z_vel_y + k_diff_exp*Dif_Err_vel_y;

	Res_vel_y = kp*G_sr_err_y+kd*G_sr_z_vel_y+Integ_vel_y;
	if(Res_vel_y > MAX_PID_vel_XY) Res_vel_y = MAX_PID_vel_XY;
	if(Res_vel_y < -MAX_PID_vel_XY) Res_vel_y = -MAX_PID_vel_XY;
	Last_Err_vel_y = err_fo_diff;
	return Res_vel_y;
    }




