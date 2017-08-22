/*    PID   */

#ifndef _PID_H_
#define _PID_H_

#include "Config.h"

#define dt			DT_CYCLE_MPU

//Угловые ПИДы
extern float KP_ugla_xy, KI_ugla_xy, KD_ugla_xy;
extern float KP_rate_xy, KI_rate_xy, KD_rate_xy;

extern float KP_ugla_z, KI_ugla_z, KD_ugla_z;
extern float KP_rate_z, KI_rate_z, KD_rate_z;

extern float f_cut_diff_pid;

extern float control_action_y, Ref_for_rate_y, Ref_for_ugol_y;
extern float control_action_x, Ref_for_rate_x, Ref_for_ugol_x;
extern float control_action_z, Ref_for_rate_z, Ref_for_ugol_z;

extern float pid_rateX_i, pid_rateY_i, pid_rateZ_i;
//****************************************************//
//Высота
extern float KP_Pos_Alt_Z;
extern float KP_Vel_Alt_Z, KD_Vel_Alt_Z;
extern float KP_Acc_Alt_Z, KI_Vel_Alt_Z, KD_Acc_Alt_Z;

extern float control_action_thr, Ref_for_vel_alt_z, Ref_for_acc_alt_z, control_action_acc_alt_z;

extern float f_cut_diff_vel_alt_z, f_cut_err_vel_alt_z;
//****************************************************//
//X Y
extern float KP_Pos_XY;
extern float KP_Vel_XY, KI_Vel_XY, KD_Vel_XY;

extern float f_cut_err_vel_xy;
extern float f_cut_diff_vel_xy;

extern float Ref_for_vel_x, Ref_for_vel_y;
extern float Ref_for_angle_Glob_x, Ref_for_angle_Glob_y;
extern float pid_velX_i, pid_velY_i, pid_velZ_i;
//****************************************************//

//угловые
#define MAX_PID_ugla_xy		 15.0f//задание для регулятора скорости
#define MIN_PID_ugla_xy		-15.0f//задание для регулятора скорости
#define MAX_PID_vel_xy 		 100.0f//выход на микшер
#define MIN_PID_vel_xy 		-100.0f//выход на микшер

#define MAX_PID_ugla_z 		 6.0f//задание для регулятора скорости
#define MIN_PID_ugla_z 		-6.0f//задание для регулятора скорости
#define MAX_PID_vel_z 		 20.0f//выход на микшер
#define MIN_PID_vel_z 		-20.0f//выход на микшер
//****************************************************//

#define F_CUT_PPM_REF_XY		3.11f
#define K_EXP_PPM_REF_XY 		(float)(1.0f - 2.0f*M_PI*F_CUT_PPM_REF_XY*dt/(1.0f+2.0f*M_PI*F_CUT_PPM_REF_XY*dt))

#define F_CUT_PPM_REF_VEL_Z		6.062f
#define K_EXP_PPM_REF_VEL_Z 		(float)(1.0f - 2.0f*M_PI*F_CUT_PPM_REF_VEL_Z*dt/(1.0f+2.0f*M_PI*F_CUT_PPM_REF_VEL_Z*dt))

#define F_CUT_PPM_REF_UGOL_Z		6.61f
#define K_EXP_PPM_REF_UGOL_Z		(float)(1.0f - 2.0f*M_PI*F_CUT_PPM_REF_UGOL_Z*dt/(1.0f+2.0f*M_PI*F_CUT_PPM_REF_UGOL_Z*dt))

#define DEAD_ZONE_YAW_STICK		0.08f
//****************************************************//
//высота
#define MAX_PID_pos_alt_z 	 4.5f//задание для регулятора скорости
#define MIN_PID_pos_alt_z 	-4.5f//задание для регулятора скорости
#define MAX_PID_vel_alt_z 	 35.0f//выход на микшер
#define MIN_PID_vel_alt_z 	-30.0f//выход на микшер
//****************************************************//
//X Y
#define MAX_PID_pos_XY	 	 10.5f//задание для регулятора глобальной скорости
#define MAX_PID_vel_XY		 0.9f//выход на регулятор угла

#define F_CUT_GLOB_REF_XY		2.8f
#define K_EXP_GLOB_REF_XY 		(float)(2.0f*M_PI*F_CUT_GLOB_REF_XY*dt/(1.0f+2.0f*M_PI*F_CUT_GLOB_REF_XY*dt))

#define K_VEL_REF_X_Y		13.0f

#define DEAD_ZONE_XY_STICK		0.02f
//****************************************************//
/* END PID */


float PID_Angle_Y (float Err_y_ugol, float thr);
float PID_Rate_Y (float ref, float real, float thr);
float PID_Angle_X (float Err_x_ugol, float thr);
float PID_Rate_X (float ref, float real, float thr);
float PID_Angle_Z (float Err_z_ugol, float thr);
float PID_Rate_Z (float ref, float real, float thr);

float PID_Alt_Pos_Z(float ref, float real);
float PID_Alt_Vel_Z(float ref, float real, uint8_t reset);

float PID_Pos_XY(float ref_x, float real_x, float ref_y, float real_y, float *res_x, float *res_y);
float PID_Vel_X(float ref, float real, uint8_t reset);
float PID_Vel_Y(float ref, float real, uint8_t reset);



#endif /* _PID_H_ */
