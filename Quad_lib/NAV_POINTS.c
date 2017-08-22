
#include "Config.h"

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"

#include "IMU_INS.h"
#include "UKF_lib.h"

#include "NAV_POINTS.h"



#define VEL_AVRG_HOR		 1.8f
#define VEL_AVRG_VERT		 0.7f

uint16_t NumPoints = 0, CurrentPoint = 0;

point_t ArrayPoints[MAX_POINTS];

uint32_t usOffset;

uint8_t flag_next_point = 0, flag_first_enter=0;



struct{
	polinom_t x,y,z;
	float time;
} currentPol;

void initPoint(float x, float y, float z)
{
	if (NumPoints < MAX_POINTS)
	    {
		ArrayPoints[NumPoints].x = x;
		ArrayPoints[NumPoints].y = y;
		ArrayPoints[NumPoints].z = z;

		NumPoints++;
	    }
	else
	    {

	    }

}
void freeAllPoints(void)
{
	NumPoints = 0;
	CurrentPoint = 0;
	flag_first_enter = 0;
}

float FindMaxTime(float xpos, float ypos, float zpos)
    {
	float temp1, temp2, temp3, dist, timeH, timeV;

	temp1 = ArrayPoints[CurrentPoint].x - xpos;
	temp2 = ArrayPoints[CurrentPoint].y - ypos;
	temp3 = ArrayPoints[CurrentPoint].z - zpos;


	dist = sqrtf(temp1*temp1 + temp2*temp2);

	timeH = dist/(float)VEL_AVRG_HOR;
	timeV = fabsf(temp3)/(float)VEL_AVRG_VERT;

	return MAX(timeH, timeV);

    }
float FindMaxTime2(float xpos, float ypos, float zpos, float k)
    {
	float temp1, temp2, temp3, dist, timeH, timeV;

	temp1 = ArrayPoints[CurrentPoint].x - xpos;
	temp2 = ArrayPoints[CurrentPoint].y - ypos;
	temp3 = ArrayPoints[CurrentPoint].z - zpos;


	dist = sqrtf(temp1*temp1 + temp2*temp2);


	timeH = dist/((float)VEL_AVRG_HOR*k);
	timeV = fabsf(temp3)/((float)VEL_AVRG_VERT);

	return MAX(timeH, timeV);

    }
void SolvePolinom(polinom_t *p, float pos0, float vel0, float t1, float pos1, float vel1)
    {
	float temp1, temp2;

	p->c = vel0;
	p->d = pos0;

	temp1 = -t1/3.0f*(vel1+2.0f*vel0);
	temp1 += pos1-pos0;
	p->b = temp1*3.0f/(t1*t1);

	temp2 = -2.0f*t1*p->b;
	temp2 += vel1-vel0;
	p->a = temp2/(3.0f*t1*t1);

    }
void SolveAllOnce(float x, float y, float z, float vx, float vy, float vz, float kvel)
    {

//	posX = DataUKF.posx;
//	posY = DataUKF.posy;
//	posZ = DataUKF.tru_pos_z;
//
//	velX = DataUKF.velx;
//	velY = DataUKF.vely;
//	velZ = DataUKF.tru_vel_z;


	currentPol.time = FindMaxTime2(x, y, z, kvel);

	SolvePolinom(&currentPol.x, x, vx, currentPol.time, ArrayPoints[CurrentPoint].x, 0.0f);
	SolvePolinom(&currentPol.y, y, vy, currentPol.time, ArrayPoints[CurrentPoint].y, 0.0f);
	SolvePolinom(&currentPol.z, z, vz, currentPol.time, ArrayPoints[CurrentPoint].z, 0.0f);

	usOffset = timerMicros();




    }
void lerpTrajectory(uint32_t usCur, point_t *p)
    {
	float time = (float)usCur/1000000.0f;

	float temp1, temp2;

	temp1 = currentPol.x.d;
	temp1 += currentPol.x.c*time;
		temp2 = time*time;
	temp1 += currentPol.x.b*temp2;
		temp2 *= time;
	temp1 += currentPol.x.a*temp2;

	p->x = temp1;

	temp1 = currentPol.y.d;
	temp1 += currentPol.y.c*time;
		temp2 = time*time;
	temp1 += currentPol.y.b*temp2;
		temp2 *= time;
	temp1 += currentPol.y.a*temp2;

	p->y = temp1;

	temp1 = currentPol.z.d;
	temp1 += currentPol.z.c*time;
		temp2 = time*time;
	temp1 += currentPol.z.b*temp2;
		temp2 *= time;
	temp1 += currentPol.z.a*temp2;

	p->z = temp1;


    }
void cycleLerp(float *x_ref, float *y_ref, float *z_ref, float x, float y, float z, float vx, float vy, float vz)
    {
	if ((NumPoints>1)&&flag_first_enter)
	    {
		uint32_t usCur;
		static point_t point;
		float temp1, temp2, temp3, temp4;

		usCur = timerMicros() - usOffset;
		if(currentPol.time>((float)usCur/1000000.0f))
		    {
			lerpTrajectory(usCur, &point);
		    }
		else
		    {
			// проверка на достижение точки, если точку достигли, то можно двигаться дальше
			temp1 = point.x - x;
			temp2 = point.y - y;
			temp3 = point.z - z;

			temp4 = sqrtf(temp1*temp1 + temp2*temp2 + temp3*temp3);
			if (temp4 < 0.3f)
			    {
				CurrentPoint = (CurrentPoint + 1) % NumPoints;
				SolveAllOnce(*x_ref, *y_ref, *z_ref, 0.0f, 0.0f, 0.0f, 1.0f); // по текущему реальному положению расчет следующей точки
			    }
//			CurrentPoint = (CurrentPoint + 1) % NumPoints;
//			SolveAllOnce(*x_ref, *y_ref, *z_ref, 0.0f, 0.0f, 0.0f);// по заданию расчет следующей точки
		    }
		*x_ref = point.x;
		*y_ref = point.y;
		*z_ref = point.z;
	    }
    }
void firstEnterPoint(float x, float y, float z, float vx, float vy, float vz)
    {
	if ((flag_first_enter==0)&&(NumPoints>1))
	    {
		CurrentPoint = 0;
		SolveAllOnce(x, y, z, vx, vy, vz, 1.0f);
		flag_first_enter = 1;
	    }

    }
void exitPoint(void)
    {
	CurrentPoint = 0;
	flag_first_enter = 0;

    }

void lerpTrajectory2(uint32_t usCur, point_t *p, point_t *v)
    {
	float time = (float)usCur/1000000.0f;

	float temp1, temp2;
	float temp3;

	temp1 = currentPol.x.d;
	    temp3 = currentPol.x.c;
	temp1 += currentPol.x.c*time;
	    temp3 += 2.0f*currentPol.x.b*time;
	temp2 = time*time;
	temp1 += currentPol.x.b*temp2;
	    temp3 += 3.0f*currentPol.x.a*temp2;
	temp2 *= time;
	temp1 += currentPol.x.a*temp2;

	p->x = temp1;
	v->x = temp3;

	temp1 = currentPol.y.d;
	    temp3 = currentPol.y.c;
	temp1 += currentPol.y.c*time;
	    temp3 += 2.0f*currentPol.y.b*time;
	temp2 = time*time;
	temp1 += currentPol.y.b*temp2;
	    temp3 += 3.0f*currentPol.y.a*temp2;
	temp2 *= time;
	temp1 += currentPol.y.a*temp2;

	p->y = temp1;
	v->y = temp3;

	temp1 = currentPol.z.d;
	    temp3 = currentPol.z.c;
	temp1 += currentPol.z.c*time;
	    temp3 += 2.0f*currentPol.z.b*time;
	temp2 = time*time;
	temp1 += currentPol.z.b*temp2;
	    temp3 += 3.0f*currentPol.z.a*temp2;
	temp2 *= time;
	temp1 += currentPol.z.a*temp2;

	p->z = temp1;
	v->z = temp3;


    }
void DynamicLerp2(float *x_ref, float *y_ref, float *z_ref, float x, float y, float z, uint8_t flagR, float kvel)
    {

	static uint8_t cntDiv = 0;
	static point_t pos={0}, vel={0};
	float temp1, temp2, temp3, temp4;
	uint32_t usCur;

	cntDiv = (cntDiv + 1) % 8;

	if(flagR)
	    {
		pos.x = x;
		pos.y = y;
		pos.z = z;
		vel.x = 0.0f;
		vel.y = 0.0f;
		vel.z = 0.0f;
		cntDiv = 0;
	    }

	if(!(cntDiv))
	    {
		temp1 = ArrayPoints[CurrentPoint].x - x;
		temp2 = ArrayPoints[CurrentPoint].y - y;
		temp3 = ArrayPoints[CurrentPoint].z - z;

		temp4 = sqrtf(temp1*temp1 + temp2*temp2 + temp3*temp3);
		if ((temp4 > 3.0f)||(flagR))
		SolveAllOnce(pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, kvel);
	    }
	usCur = timerMicros() - usOffset;
	if(currentPol.time>((float)usCur/1000000.0f))
	    {
		lerpTrajectory2(usCur, &pos, &vel);
	    }
	else
	    {
		// проверка на достижение точки, если точку достигли, то можно двигаться дальше
		temp1 = ArrayPoints[CurrentPoint].x - x;
		temp2 = ArrayPoints[CurrentPoint].y - y;
		temp3 = ArrayPoints[CurrentPoint].z - z;

		temp4 = sqrtf(temp1*temp1 + temp2*temp2 + temp3*temp3);
		if (temp4 < 0.3f)
		    {
			CurrentPoint = (CurrentPoint + 1) % NumPoints;
			SolveAllOnce(pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, kvel); // по текущему реальному положению расчет следующей точки
		    }

	    }
	*x_ref = pos.x;
	*y_ref = pos.y;
	*z_ref = pos.z;
    }


