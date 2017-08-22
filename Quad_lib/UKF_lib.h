
#ifndef _NAV_UKF_H
#define _NAV_UKF_H

#include "Config.h"

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"


#define SRCDKF_H	(__sqrtf(3.0f) * 3.0f)
#define SRCDKF_RM	0.0001f		// Robbins-Monro stochastic term

typedef void SRCDKFTimeUpdate_t(float32_t *x_I, float32_t *noise_I, float32_t *x_O, float32_t *u, float32_t dt, int n);
typedef void SRCDKFMeasurementUpdate_t(float32_t *u, float32_t *x, float32_t *noise_I, float32_t *y);

typedef struct {
	int S;
	int V;
	int M;		// only used for parameter estimation
	int N;		// only used for parameter estimation
	int L;

	float32_t h;
	float32_t hh;
	float32_t w0m, wim, wic1, wic2;
	float32_t rm;

	arm_matrix_instance_f32 Sx;	// state covariance
	arm_matrix_instance_f32 SxT;	// Sx transposed
	arm_matrix_instance_f32 Sv;	// process noise
	arm_matrix_instance_f32 Sn;	// observation noise
	arm_matrix_instance_f32 x;	// state estimate vector
	arm_matrix_instance_f32 Xa;	// augmented sigma points
	float32_t *xIn;
	float32_t *xNoise;
	float32_t *xOut;
	arm_matrix_instance_f32 qrTempS;
	arm_matrix_instance_f32 Y;	// resultant measurements from sigma points
	arm_matrix_instance_f32 y;	// measurement estimate vector
	arm_matrix_instance_f32 qrTempM;
	arm_matrix_instance_f32 Sy;	// measurement covariance
	arm_matrix_instance_f32 SyT;	// Sy transposed
	arm_matrix_instance_f32 SyC;	// copy of Sy
	arm_matrix_instance_f32 Pxy;
	arm_matrix_instance_f32 C1;
	arm_matrix_instance_f32 C1T;
	arm_matrix_instance_f32 C2;
	arm_matrix_instance_f32 D;
	arm_matrix_instance_f32 K;
	arm_matrix_instance_f32 KT;	// only used for param est
	arm_matrix_instance_f32 inov;	// inovation
	arm_matrix_instance_f32 inovT;// only used for param est
	arm_matrix_instance_f32 xUpdate;
	arm_matrix_instance_f32 qrFinal;
	arm_matrix_instance_f32 rDiag;
	arm_matrix_instance_f32 Q, R, AQ;	// scratch

	SRCDKFTimeUpdate_t *timeUpdate;
	SRCDKFMeasurementUpdate_t *map;	// only used for param est
} srcdkf_t;


#define SIM_S                   17		// states
#define SIM_M                   3		// max measurements
#define SIM_V                   12//16		// process noise
#define SIM_N                   3		// max observation noise

#define UKF_GYO_AVG_NUM		40

#define UKF_STATE_VELN		0
#define UKF_STATE_VELE		1
#define UKF_STATE_VELD		2
#define UKF_STATE_POSN		3
#define UKF_STATE_POSE		4
#define UKF_STATE_POSD		5
#define UKF_STATE_ACC_BIAS_X	6
#define UKF_STATE_ACC_BIAS_Y	7
#define UKF_STATE_ACC_BIAS_Z	8
#define UKF_STATE_GYO_BIAS_X	9
#define UKF_STATE_GYO_BIAS_Y	10
#define UKF_STATE_GYO_BIAS_Z	11
#define UKF_STATE_Q1		12
#define UKF_STATE_Q2		13
#define UKF_STATE_Q3		14
#define UKF_STATE_Q4		15
#define UKF_STATE_PRES_ALT	16

#define UKF_V_NOISE_ACC_BIAS_X	0
#define UKF_V_NOISE_ACC_BIAS_Y	1
#define UKF_V_NOISE_ACC_BIAS_Z	2
#define UKF_V_NOISE_GYO_BIAS_X	3
#define UKF_V_NOISE_GYO_BIAS_Y	4
#define UKF_V_NOISE_GYO_BIAS_Z	5
#define UKF_V_NOISE_RATE_X	6
#define UKF_V_NOISE_RATE_Y	7
#define UKF_V_NOISE_RATE_Z	8
#define UKF_V_NOISE_VELN	9
#define UKF_V_NOISE_VELE	10
#define UKF_V_NOISE_VELD	11

#define UKF_VELN		navUkfData.x[UKF_STATE_VELN]
#define UKF_VELE		navUkfData.x[UKF_STATE_VELE]
#define UKF_VELD		navUkfData.x[UKF_STATE_VELD]
#define UKF_POSN		navUkfData.x[UKF_STATE_POSN]
#define UKF_POSE		navUkfData.x[UKF_STATE_POSE]
#define UKF_POSD		navUkfData.x[UKF_STATE_POSD]
#define UKF_ACC_BIAS_X		navUkfData.x[UKF_STATE_ACC_BIAS_X]
#define UKF_ACC_BIAS_Y		navUkfData.x[UKF_STATE_ACC_BIAS_Y]
#define UKF_ACC_BIAS_Z		navUkfData.x[UKF_STATE_ACC_BIAS_Z]
#define UKF_GYO_BIAS_X		navUkfData.x[UKF_STATE_GYO_BIAS_X]
#define UKF_GYO_BIAS_Y		navUkfData.x[UKF_STATE_GYO_BIAS_Y]
#define UKF_GYO_BIAS_Z		navUkfData.x[UKF_STATE_GYO_BIAS_Z]
#define UKF_Q1			navUkfData.x[UKF_STATE_Q1]
#define UKF_Q2			navUkfData.x[UKF_STATE_Q2]
#define UKF_Q3			navUkfData.x[UKF_STATE_Q3]
#define UKF_Q4			navUkfData.x[UKF_STATE_Q4]
#define UKF_PRES_ALT		navUkfData.x[UKF_STATE_PRES_ALT]

#define USE_PRES_ALT

#ifdef USE_PRES_ALT
#define UKF_ALTITUDE	UKF_PRES_ALT
#else
#define UKF_ALTITUDE	UKF_POSD
#endif

#define UKF_HIST		40
#define UKF_P0			101325.0f			    // standard static pressure at sea level



#define ALT_S           3   // states
#define ALT_M           1   // measurements
#define ALT_V           2   // process noise
#define ALT_N           1   // measurement noise

#define ALT_STATE_POS   0
#define ALT_STATE_VEL   1
#define ALT_STATE_BIAS  2

#define ALT_NOISE_BIAS  0
#define ALT_NOISE_VEL   1

#define ALT_POS         altUkfData.x[ALT_STATE_POS]
#define ALT_VEL         altUkfData.x[ALT_STATE_VEL]
#define ALT_BIAS        altUkfData.x[ALT_STATE_BIAS]

#define ALT_PRES_NOISE  0.02f
#define ALT_BIAS_NOISE  5e-4f//5e-5f
#define ALT_VEL_NOISE   5e-4f

typedef struct {
    srcdkf_t *kf;
    float *x;               // states
} altUkfStruct_t;

extern altUkfStruct_t altUkfData;

extern void altUkfInit(void);
extern void altUkfProcess(float measuredPres);

#define ALTITUDE_UKF                 (*runData.altPos)
#define VELOCITYD_UKF                (*runData.altVel)
/////////////////////////////////////////////////////////////

#define UKF_VEL_Q               +3.2545e-02     // +0.032544903471       0.000000350530 +0.000037342305
#define UKF_VEL_ALT_Q           +1.4483e-01     // +0.144827254833       0.000000347510 -0.000055111229
#define UKF_POS_Q               +7.1562e+03     // +7156.240473309331    0.000000352142 +2.727925965284749
#define UKF_POS_ALT_Q           +5.3884e+03     // +5388.369673129109    0.000000351319 -6.187843541372100
#define UKF_ACC_BIAS_Q          +1.3317e-03     // +0.001331748045       0.000000359470 +0.000000039113
#define UKF_GYO_BIAS_Q          +4.5256e-02     // +0.045255679186       0.000000349060 +0.000045999290
#define UKF_QUAT_Q              +5.4005e-04     // +0.000540045060       0.000000353882 +0.000000029711
#define UKF_PRES_ALT_Q          +6.3105e+01     // +63.104671424320      0.000000353790 +0.0166164673283
#define UKF_ACC_BIAS_V          +7.8673e-07     // +0.000000786725       0.000000345847 -0.000000000977
#define UKF_GYO_BIAS_V          +4.0297e-09     // +0.000000004030       0.000000359017 +0.000000000000
#define UKF_RATE_V              +1.7538e-05     // +0.000017538388       0.000000358096 +0.000000000397
#define UKF_VEL_V               +2.8605e-07     // +0.000000286054       0.000000351709 +0.000000000183
#define UKF_ALT_VEL_V           +6.8304e-08     // +0.000000068304       0.000000362348 -0.000000000050
#define UKF_GPS_POS_N           +8.0703e-06     // +0.000008070349       0.000000353490 +0.000000005602
#define UKF_GPS_POS_M_N         +3.0245e-05     // +0.000030245341       0.000000345021 -0.000000008396
#define UKF_GPS_ALT_N           +1.1796e-05     // +0.000011795879       0.000000356036 -0.000000010027
#define UKF_GPS_ALT_M_N         +3.8329e-05     // +0.000038328879       0.000000346581 +0.000000027268
#define UKF_GPS_VEL_N           +1.7640e-01     // +0.176404763511       0.000000355574 -0.000094105688
#define UKF_GPS_VEL_M_N         +3.0138e-02     // +0.030138272888       0.000000343584 -0.000002668997
#define UKF_GPS_VD_N            +4.6379e+00     // +4.637855992835       0.000000358079 +0.000310962082
#define UKF_GPS_VD_M_N          +1.3127e-02     // +0.013127146795       0.000000347978 -0.000001550944
//#define UKF_ALT_N               +9.5913e-02     // +0.095913477777       0.000000356359 -0.000049781087
#define UKF_ALT_N               +4.5913e-01     // +0.4595913477777       0.000000356359 -0.000049781087
#define UKF_ACC_N               +6.3287e-05     // +0.000063286884       0.000000342761 -0.000000022717
#define UKF_DIST_N              +9.7373e-03     // +0.009737270392       0.000000356147 +0.000009059372
#define UKF_MAG_N               +5.2355e-01     // +0.523549973965       0.000000500000 +0.000000000000
#define UKF_POS_DELAY           +2.1923e+03     // +2192.300048828125    0.000000500000 +0.000000000000125
#define UKF_VEL_DELAY           -1.0182e+05     // -101820.000000000000  0.000000500000 +0.00000000000000000


#define RUN_SENSOR_HIST		10				// number of timesteps to average observation sensors' data



#define NAV_MIN_GPS_ACC		3.0f					    // minimum gps hAcc needed to enter auto nav modes, in meters
#define IMU_STATIC_STD		0.05f
//***************************************************************************************************//

#define UKF_VEL_POS_USE
#define UKF_BIAS_QUAT_USE


extern uint8_t flag_start_state_init;
extern float q0, q1, q2, q3, gxbias, gybias, gzbias;
extern float qmah0, qmah1, qmah2, qmah3;
extern float axisx, axisy, axisz;

extern float yaw_start;


typedef struct {
	float presAltOffset;
    float bestHacc;
    float accMask;
    float accHist[3][RUN_SENSOR_HIST];
    float magHist[3][RUN_SENSOR_HIST];
    float presHist[RUN_SENSOR_HIST];
    float sumAcc[3];
    float sumMag[3];
    float sumPres;
    int sensorHistIndex;
    float *altPos;
    float *altVel;
} runStruct_t;
extern runStruct_t runData;

typedef struct {
    srcdkf_t *kf;
    float v0a[3];
    float v0m[3];

    float posN[UKF_HIST];
    float posE[UKF_HIST];
    float posD[UKF_HIST];


    float velN[UKF_HIST];
    float velE[UKF_HIST];
    float velD[UKF_HIST];

    #ifdef UKF_BIAS_QUAT_USE

    float abiasx[UKF_HIST];
    float abiasy[UKF_HIST];
    float abiasz[UKF_HIST];

    float gbiasx[UKF_HIST];
    float gbiasy[UKF_HIST];
    float gbiasz[UKF_HIST];

    float q0[UKF_HIST];
    float q1[UKF_HIST];
    float q2[UKF_HIST];
    float q3[UKF_HIST];

    float gx[UKF_HIST];
    float gy[UKF_HIST];
    float gz[UKF_HIST];

    #endif

    int navHistIndex;

    float yaw, pitch, roll;
    float yawCos, yawSin;
    float *x;			// states
} navUkfStruct_t;
typedef struct{
	float velx, vely, velz;
	float posx, posy, posz;
	float pres_alt;
	float baro_alt, baro_vel;
	float abiasx, abiasy, abiasz;
	float tru_pos_z, tru_vel_z;
	float course;
} data_ukf;

extern navUkfStruct_t navUkfData;
extern data_ukf DataUKF;
extern uint32_t ovf_ukf_stack;
//***************************************************************************************************//

void *aqDataCalloc(uint16_t count, uint16_t size);
void aqFree(void *ptr, size_t count, size_t size);

float32_t* matrixInit(arm_matrix_instance_f32 *m, int rows, int cols);
void matrixFree(arm_matrix_instance_f32 *m);
int qrDecompositionT_f32(arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R);
void matrixDiv_f32(arm_matrix_instance_f32 *X, arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *B, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R, arm_matrix_instance_f32 *AQ);
srcdkf_t *srcdkfInit(int s, int m, int v, int n, SRCDKFTimeUpdate_t *timeUpdate);
void Cacl_Sigma_Points(srcdkf_t *f, arm_matrix_instance_f32 *Sn);
void srcdkfTimeUpdate(srcdkf_t *f, float32_t *u, float32_t dt);
void srcdkfMeasurementUpdate(srcdkf_t *f, float32_t *u, float32_t *ym, int M, int N, float32_t *noise, SRCDKFMeasurementUpdate_t *measurementUpdate);

void navUkfNormalizeVec3(float *vr, float *v);
void navUkfNormalizeQuat(float *qr, float *q);
void navUkfRotateVectorByQuat(float *vr, float *v, float *q);
void navUkfRotateVectorByRevQuat(float *vr, float *v, float *q);
void navUkfRotateVecByMatrix(float *vr, float *v, float *m);
static void navUkfRotateVecByRevMatrix(float *vr, float *v, float *m);
static void navUkfQuatToMatrix(float *m, float *q, int normalize);
void navUkfMatrixExtractEuler(float *m, float *yaw, float *pitch, float *roll);
void navUkfQuatExtractEuler(float *q, float *yaw, float *pitch, float *roll);
static void navUkfRotateQuat(float *qOut, float *qIn, float *rate);

void navUkfAccUpdate(float *u, float *x, float *noise, float *y);
void navUkfMagUpdate(float *u, float *x, float *noise, float *y);
void simDoAccUpdate(float accX, float accY, float accZ);
float navUkfPresToAlt(float pressure) ;
void navUkfPresUpdate(float *u, float *x, float *noise, float *y);
void navUkfPresGPSAltUpdate(float *u, float *x, float *noise, float *y) ;
void simDoPresUpdate(float pres);

void simDoMagUpdate(float magX, float magY, float magZ);
void navUkfGpsPosUpdate(uint32_t gpsMicros, float y_gps, float x_gps, float alt, float hAcc, float vAcc);
void navUkfGpsVelUpdate(uint32_t gpsMicros, float velN, float velE, float velD, float sAcc);
void navPressureAdjust(float altitude);
void navUkfPosUpdate(float *u, float *x, float *noise, float *y);
void navUkfVelUpdate(float *u, float *x, float *noise, float *y);
void navUkfZeroPos(void);
void navUkfZeroVel(void);
void navUkfRateUpdate(float *u, float *x, float *noise, float *y);
void navUkfZeroRate(float rate, int axis);


void altUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt, int n) ;
void altUkfPresUpdate(float *u, float *x, float *noise, float *y) ;
static void altDoPresUpdate(float measuredPres) ;
void altUkfProcess(float measuredPres);
void altUkfInit(void);

void navUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt, int n);
void navUkfInitState(void);
void navUkfInit(void);
void srcdkfSetVariance(srcdkf_t *f, float32_t *q, float32_t *v, float32_t *n, int nn);
float *srcdkfGetState(srcdkf_t *f);
void navUkfInertialUpdate(void);
void navUkfFinish(void);

void MahonyAHRSupdateIMU_Corr_Mag(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float Decl, unsigned char Enable_cont_corr);

void quatMultiply(float *qr, float *q1, float *q2);
void eulerToQuatYPR(float *q, float *angles);
void eulerToQuatRPY(float *q, float *angles);
void quatMultiply(float *qr, float *q1, float *q2);



#endif
