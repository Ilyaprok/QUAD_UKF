

#ifndef _NAV_POINTS_H_
#define _NAV_POINTS_H_

#include "Config.h"

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"

#define MAX_POINTS 20

typedef struct {
    float x;
    float y;
    float z;
} point_t;

typedef struct {
	float a,b,c,d;
}polinom_t;



extern uint16_t NumPoints, CurrentPoint;
extern point_t ArrayPoints[MAX_POINTS];

void initPoint(float x, float y, float z);
void freeAllPoints(void);

void cycleLerp(float *x_ref, float *y_ref, float *z_ref, float x, float y, float z, float vx, float vy, float vz);
void firstEnterPoint(float x, float y, float z, float vx, float vy, float vz);
void exitPoint(void);

void DynamicLerp2(float *x_ref, float *y_ref, float *z_ref, float x, float y, float z, uint8_t flagR, float kvel);

#endif
