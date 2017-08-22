
#include "Config.h"

#include "IMU_INS.h"
#include "MPU6500.h"
#include "I2C_BARO_MAG.h"
#include "GPS.h"

#include "UKF_lib.h"


//#define USE_UKF_MAG


//#define MIN(a, b) ((a < b) ? a : b)
//#define MAX(a, b) ((a > b) ? a : b)

//Структуры для работы 2-х UKF
navUkfStruct_t navUkfData;
altUkfStruct_t altUkfData;
//****************************************************//
//Служебные перменные для работы с памятью
#define UTIL_CCM_HEAP_SIZE	    (5490)
uint32_t  dataSramUsed;
uint32_t *ccmHeap[UTIL_CCM_HEAP_SIZE] __attribute__((section(".ccm")));
uint8_t flag_out_SRAM;
//****************************************************//
//Вспомогательные
uint8_t flag_start_state_init;
float32_t  *adrmatrix_Pxy, *adrmatrix_Sx, *adrmatrix_Sy, *adrLastMatrix;
float32_t  matrix_Pxy[21], matrix_Sy[9], matrix_Sx[49];
size_t kal;
int Histindex_gyr;

float cclinVelX, cclinVelY, cclinVelZ;
float ccGlobLinVelX, ccGlobLinVelY, ccGlobLinVelZ;
float ccVelX, ccVelY, ccVelZ;

float ccglobPosX, ccglobPosY, ccglobPosZ;
float ccPosX, ccPosY, ccPosZ;
//****************************************************//

//основные данные
#define F_CUT_GYO_GPS			30.0f
#define K_EXP_GYO_GPS	 		(float)(1.0f - 2.0f*M_PI*F_CUT_GYO_GPS*DT_UKF/(1.0f+2.0f*M_PI*F_CUT_GYO_GPS*DT_UKF))

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f, gxbias, gybias, gzbias;
float qmah0 = 1.0f, qmah1 = 0.0f, qmah2 = 0.0f, qmah3 = 0.0f;
float axisx, axisy, axisz;
float yaw_start;
data_ukf DataUKF __attribute__((section(".ccm")));
runStruct_t runData __attribute__((section(".ccm")));
//////////////////////////////

// allocates memory from 64KB CCM
void *aqDataCalloc(uint16_t count, uint16_t size) {
    uint32_t words;

    // round up to word size
    words = (count*size + sizeof(int)-1) / sizeof(int);

    if ((dataSramUsed + words) > UTIL_CCM_HEAP_SIZE) {
        //AQ_NOTICE("Out of data SRAM!\n");
    	flag_out_SRAM = 1;
    }
    else {
        dataSramUsed += words;
    }

    return (void *)(ccmHeap + dataSramUsed - words);
}
void aqFree(void *ptr, size_t count, size_t size) {
    if (ptr) {
        free(ptr);
        dataSramUsed -= count * size;
    }
}

float32_t* matrixInit(arm_matrix_instance_f32 *m, int rows, int cols) {
    float32_t *d;

    d = (float32_t *)aqDataCalloc(rows*cols, sizeof(float32_t));
    //d = (float32_t *)pvPortMalloc(rows*cols*sizeof(float32_t));

    arm_mat_init_f32(m, rows, cols, d);
    arm_fill_f32(0, d, rows*cols);
    return d;
}
void matrix_equ(float32_t *adr, uint8_t size, float32_t *dst)
    {
	uint8_t i;
	for (i = 0; i<size; i++)
	    {
		dst[i]=adr[i];
	    }
    }
void matrixFree(arm_matrix_instance_f32 *m) {
    if (m && m->pData)
	free(m->pData);
}
int qrDecompositionT_f32(arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R) {
    int minor;
    int row, col;
    int m = A->numCols;
    int n = A->numRows;
    int min;

    // clear R
    arm_fill_f32(0, R->pData, R->numRows*R->numCols);

    min = MIN(m, n);

    /*
    * The QR decomposition of a matrix A is calculated using Householder
    * reflectors by repeating the following operations to each minor
    * A(minor,minor) of A:
    */
    for (minor = 0; minor < min; minor++) {
	    float xNormSqr = 0.0f;
	    float a;

	    /*
	    * Let x be the first column of the minor, and a^2 = |x|^2.
	    * x will be in the positions A[minor][minor] through A[m][minor].
	    * The first column of the transformed minor will be (a,0,0,..)'
	    * The sign of a is chosen to be opposite to the sign of the first
	    * component of x. Let's find a:
	    */
	    for (row = minor; row < m; row++)
		    xNormSqr += A->pData[minor*m + row]*A->pData[minor*m + row];

	    a = __sqrtf(xNormSqr);
	    if (A->pData[minor*m + minor] > 0.0f)
		    a = -a;

	    if (a != 0.0f) {
		    R->pData[minor*R->numCols + minor] = a;

		    /*
		    * Calculate the normalized reflection vector v and transform
		    * the first column. We know the norm of v beforehand: v = x-ae
		    * so |v|^2 = <x-ae,x-ae> = <x,x>-2a<x,e>+a^2<e,e> =
		    * a^2+a^2-2a<x,e> = 2a*(a - <x,e>).
		    * Here <x, e> is now A[minor][minor].
		    * v = x-ae is stored in the column at A:
		    */
		    A->pData[minor*m + minor] -= a; // now |v|^2 = -2a*(A[minor][minor])

		    /*
		    * Transform the rest of the columns of the minor:
		    * They will be transformed by the matrix H = I-2vv'/|v|^2.
		    * If x is a column vector of the minor, then
		    * Hx = (I-2vv'/|v|^2)x = x-2vv'x/|v|^2 = x - 2<x,v>/|v|^2 v.
		    * Therefore the transformation is easily calculated by
		    * subtracting the column vector (2<x,v>/|v|^2)v from x.
		    *
		    * Let 2<x,v>/|v|^2 = alpha. From above we have
		    * |v|^2 = -2a*(A[minor][minor]), so
		    * alpha = -<x,v>/(a*A[minor][minor])
		    */
		    for (col = minor+1; col < n; col++) {
			    float alpha = 0.0f;

			    for (row = minor; row < m; row++)
				    alpha -= A->pData[col*m + row]*A->pData[minor*m + row];

			    alpha /= a*A->pData[minor*m + minor];

			    // Subtract the column vector alpha*v from x.
			    for (row = minor; row < m; row++)
				    A->pData[col*m + row] -= alpha*A->pData[minor*m + row];
		    }
	    }
	    // rank deficient
	    else
		return 0;
    }

    // Form the matrix R of the QR-decomposition.
    //      R is supposed to be m x n, but only calculate n x n
    // copy the upper triangle of A
    for (row = min-1; row >= 0; row--)
	    for (col = row+1; col < n; col++)
		    R->pData[row*R->numCols + col] = A->pData[col*m + row];

    // Form the matrix Q of the QR-decomposition.
    //      Q is supposed to be m x m

    // only compute Q if requested
    if (Q) {
	    arm_fill_f32(0, Q->pData, Q->numRows*Q->numCols);

	    /*
	    * Q = Q1 Q2 ... Q_m, so Q is formed by first constructing Q_m and then
	    * applying the Householder transformations Q_(m-1),Q_(m-2),...,Q1 in
	    * succession to the result
	    */
	    for (minor = m-1; minor >= min; minor--)
		    Q->pData[minor*m + minor] = 1.0f;

	    for (minor = min-1; minor >= 0; minor--) {
		    Q->pData[minor * m + minor] = 1.0f;

		    if (A->pData[minor*m + minor] != 0.0f) {
			    for (col = minor; col < m; col++) {
				    float alpha = 0.0f;

				    for (row = minor; row < m; row++)
					    alpha -= Q->pData[row*m + col]*A->pData[minor*m + row];

				    alpha /= R->pData[minor*R->numCols + minor]*A->pData[minor*m + minor];

				    for (row = minor; row < m; row++)
					    Q->pData[row*m + col] -= alpha*A->pData[minor*m + row];
			    }
		    }
	    }
    }
    main_stack = __get_MSP();
    return 1;
}
void matrixDiv_f32(arm_matrix_instance_f32 *X, arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *B, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R, arm_matrix_instance_f32 *AQ) {
        int i, j, k;
        int m, n;

        // this is messy (going into a class's private data structure),
        // but it is better than malloc/free
        Q->numRows = B->numRows;
        Q->numCols = B->numRows;
        R->numRows = B->numRows;
        R->numCols = B->numCols;
        AQ->numRows = A->numRows;
        AQ->numCols = B->numRows;

        m = A->numRows;
        n = B->numCols;

        qrDecompositionT_f32(B, Q, R);
	arm_mat_mult_f32(A, Q, AQ);

        // solve for X by backsubstitution
        for (i = 0; i < m; i++) {
                for (j = n-1; j >= 0; j--) {
                        for (k = j+1; k < n; k++)
                                AQ->pData[i*n + j] -= R->pData[j*n + k] * X->pData[i*n + k];
                        X->pData[i*n + j] = AQ->pData[i*n + j] / R->pData[j*n + j];
                }
        }
}
srcdkf_t *srcdkfInit(int s, int m, int v, int n, SRCDKFTimeUpdate_t *timeUpdate)
{
	srcdkf_t *f;
	int maxN = MAX(v, n);

	f = (srcdkf_t *)aqDataCalloc(1, sizeof(srcdkf_t));
	kal = sizeof(srcdkf_t);
	f->S = s;
	f->V = v;

	adrmatrix_Sx = matrixInit(&f->Sx, s, s);
	matrixInit(&f->SxT, s, s);
	matrixInit(&f->Sv, v, v);
	matrixInit(&f->Sn, n, n);
	matrixInit(&f->x, s, 1);
	matrixInit(&f->Xa, s+maxN, 1+(s+maxN)*2);

	matrixInit(&f->qrTempS, s, (s+v)*2);
	matrixInit(&f->y, m, 1);
	matrixInit(&f->Y, m, 1+(s+n)*2);
	matrixInit(&f->qrTempM, m, (s+n)*2);
	adrmatrix_Sy = matrixInit(&f->Sy, m, m);
	matrixInit(&f->SyT, m, m);
	matrixInit(&f->SyC, m, m);
	adrmatrix_Pxy = matrixInit(&f->Pxy, s, m);
	matrixInit(&f->C1, m, s);
	matrixInit(&f->C1T, s, m);
	matrixInit(&f->C2, m, n);
	matrixInit(&f->D, m, s+n);
	matrixInit(&f->K, s, m);
	matrixInit(&f->inov, m, 1);
	matrixInit(&f->xUpdate, s, 1);
	matrixInit(&f->qrFinal, s, 2*s + 2*n);
	matrixInit(&f->Q, s, s+n);	// scratch
	matrixInit(&f->R, n, n);	// scratch
	adrLastMatrix = matrixInit(&f->AQ, s, n);	// scratch



	f->xOut = (float32_t *)aqDataCalloc(s, sizeof(float32_t));
	f->xNoise = (float32_t *)aqDataCalloc(maxN, sizeof(float32_t));
	f->xIn = (float32_t *)aqDataCalloc(s, sizeof(float32_t));

	f->h = SRCDKF_H;
	f->hh = f->h*f->h;
//	f->w0m = (f->hh - (float32_t)s) / f->hh;	// calculated in process
	f->wim = 1.0f / (2.0f * f->hh);
	f->wic1 = __sqrtf(1.0f / (4.0f * f->hh));
	f->wic2 = __sqrtf((f->hh - 1.0f) / (4.0f * f->hh*f->hh));

        f->timeUpdate = timeUpdate;

	return f;
}
static void srcdkfCalcSigmaPoints(srcdkf_t *f, arm_matrix_instance_f32 *Sn) {
	int S = f->S;			// number of states
	int N = Sn->numRows;		// number of noise variables
	int A = S+N;			// number of agumented states
	int L = 1+A*2;			// number of sigma points
	float32_t *x = f->x.pData;	// state
	float32_t *Sx = f->Sx.pData;	// state covariance
	float32_t *Xa = f->Xa.pData;	// augmented sigma points
	int i, j;

	// set the number of sigma points
	f->L = L;

	// resize output matrix
	f->Xa.numRows = A;
	f->Xa.numCols = L;

	//	-	   -
	// Sa =	| Sx	0  |
	//	| 0	Sn |
	//	-	   -
	// xa = [ x 	0  ]
	// Xa = [ xa  (xa + h*Sa)  (xa - h*Sa) ]
	//
	for (i = 0; i < A; i++) {
		int rOffset = i*L;
		float32_t base = (i < S) ? x[i] : 0.0f;

		Xa[rOffset + 0] = base;

		for (j = 1; j <= A; j++) {
			float32_t t = 0.0f;

			if (i < S && j < S+1)
				t = Sx[i*S + (j-1)]*f->h;

			if (i >= S && j >= S+1)
				t = Sn->pData[(i-S)*N + (j-S-1)]*f->h;

			Xa[rOffset + j]     = base + t;
			Xa[rOffset + j + A] = base - t;
		}
	}
	ovf_ukf_stack = uxTaskGetStackHighWaterMark(ukf_handle);
}
void srcdkfTimeUpdate(srcdkf_t *f, float32_t *u, float32_t dt) {
	int S = f->S;			// number of states
	int V = f->V;			// number of noise variables
	int L;				// number of sigma points
	float32_t *x = f->x.pData;	// state estimate
	float32_t *Xa = f->Xa.pData;	// augmented sigma points
//	float32_t *xIn = f->xIn;	// callback buffer
//	float32_t *xOut = f->xOut;	// callback buffer
//	float32_t *xNoise = f->xNoise;	// callback buffer
	float32_t *qrTempS = f->qrTempS.pData;
	int i, j;

	srcdkfCalcSigmaPoints(f, &f->Sv);
	L = f->L;

	// Xa = f(Xx, Xv, u, dt)
//	for (i = 0; i < L; i++) {
//		for (j = 0; j < S; j++)
//			xIn[j] = Xa[j*L + i];
//
//		for (j = 0; j < V; j++)
//			xNoise[j] = Xa[(S+j)*L + i];
//
//		f->timeUpdate(xIn, xNoise, xOut, u, dt);
//
//		for (j = 0; j < S; j++)
//			Xa[j*L + i] = xOut[j];
//	}
	f->timeUpdate(&Xa[0], &Xa[S*L], &Xa[0], u, dt, L);

	// sum weighted resultant sigma points to create estimated state
	f->w0m = (f->hh - (float32_t)(S+V)) / f->hh;
	for (i = 0; i < S; i++) {
		int rOffset = i*L;

		x[i] = Xa[rOffset + 0] * f->w0m;

		for (j = 1; j < L; j++)
			x[i] += Xa[rOffset + j] * f->wim;
	}

	// update state covariance
	for (i = 0; i < S; i++) {
		int rOffset = i*(S+V)*2;

		for (j = 0; j < S+V; j++) {
			qrTempS[rOffset + j] = (Xa[i*L + j + 1] - Xa[i*L + S+V + j + 1]) * f->wic1;
			qrTempS[rOffset + S+V + j] = (Xa[i*L + j + 1] + Xa[i*L + S+V + j + 1] - 2.0f*Xa[i*L + 0]) * f->wic2;
		}
	}

	qrDecompositionT_f32(&f->qrTempS, NULL, &f->SxT);   // with transposition
	arm_mat_trans_f32(&f->SxT, &f->Sx);
}

void srcdkfMeasurementUpdate(srcdkf_t *f, float32_t *u, float32_t *ym, int M, int N, float32_t *noise, SRCDKFMeasurementUpdate_t *measurementUpdate) {
	int S = f->S;				// number of states
	float32_t *Xa = f->Xa.pData;			// sigma points
	float32_t *xIn = f->xIn;			// callback buffer
	float32_t *xNoise = f->xNoise;		// callback buffer
	float32_t *xOut = f->xOut;			// callback buffer
	float32_t *Y = f->Y.pData;			// measurements from sigma points
	float32_t *y = f->y.pData;			// measurement estimate
	float32_t *Sn = f->Sn.pData;			// observation noise covariance
	float32_t *qrTempM = f->qrTempM.pData;
	float32_t *C1 = f->C1.pData;
	float32_t *C1T = f->C1T.pData;
	float32_t *C2 = f->C2.pData;
	float32_t *D = f->D.pData;
	float32_t *inov = f->inov.pData;		// M x 1 matrix
	float32_t *xUpdate = f->xUpdate.pData;	// S x 1 matrix
	float32_t *x = f->x.pData;			// state estimate
	float32_t *Sx = f->Sx.pData;
	float32_t *Q = f->Q.pData;
	float32_t *qrFinal = f->qrFinal.pData;
	int L;					// number of sigma points
	int i, j;

	// make measurement noise matrix if provided
	if (noise) {
		f->Sn.numRows = N;
		f->Sn.numCols = N;
		arm_fill_f32(0.0f, f->Sn.pData, N*N);
		for (i = 0; i < N; i++)
			arm_sqrt_f32(fabsf(noise[i]), &Sn[i*N + i]);
	}

	// generate sigma points
	srcdkfCalcSigmaPoints(f, &f->Sn);
	L = f->L;

	// resize all N and M based storage as they can change each iteration
	f->y.numRows = M;
	f->Y.numRows = M;
	f->Y.numCols = L;
	f->qrTempM.numRows = M;
	f->qrTempM.numCols = (S+N)*2;
	f->Sy.numRows = M;
	f->Sy.numCols = M;
	f->SyT.numRows = M;
	f->SyT.numCols = M;
	f->SyC.numRows = M;
	f->SyC.numCols = M;
	f->Pxy.numCols = M;
	f->C1.numRows = M;
	f->C1T.numCols = M;
	f->C2.numRows = M;
	f->C2.numCols = N;
	f->D.numRows = M;
	f->D.numCols = S+N;
	f->K.numCols = M;
	f->inov.numRows = M;
	f->qrFinal.numCols = 2*S + 2*N;

	// Y = h(Xa, Xn)
	for (i = 0; i < L; i++) {
		for (j = 0; j < S; j++)
			xIn[j] = Xa[j*L + i];

		for (j = 0; j < N; j++)
			xNoise[j] = Xa[(S+j)*L + i];

		measurementUpdate(u, xIn, xNoise, xOut);

		for (j = 0; j < M; j++)
			Y[j*L + i] = xOut[j];
	}

	// sum weighted resultant sigma points to create estimated measurement
	f->w0m = (f->hh - (float32_t)(S+N)) / f->hh;
	for (i = 0; i < M; i++) {
		int rOffset = i*L;

		y[i] = Y[rOffset + 0] * f->w0m;

		for (j = 1; j < L; j++)
			y[i] += Y[rOffset + j] * f->wim;
	}

	// calculate measurement covariance components
	for (i = 0; i < M; i++) {
		int rOffset = i*(S+N)*2;

		for (j = 0; j < S+N; j++) {
			float32_t c, d;

			c = (Y[i*L + j + 1] - Y[i*L + S+N + j + 1]) * f->wic1;
			d = (Y[i*L + j + 1] + Y[i*L + S+N + j + 1] - 2.0f*Y[i*L]) * f->wic2;

			qrTempM[rOffset + j] = c;
			qrTempM[rOffset + S+N + j] = d;

			// save fragments for future operations
			if (j < S) {
				C1[i*S + j] = c;
				C1T[j*M + i] = c;
			}
			else {
				C2[i*N + (j-S)] = c;
			}
			D[i*(S+N) + j] = d;
		}
	}

	qrDecompositionT_f32(&f->qrTempM, NULL, &f->SyT);	// with transposition

	arm_mat_trans_f32(&f->SyT, &f->Sy);
	arm_mat_trans_f32(&f->SyT, &f->SyC);		// make copy as later Div is destructive

	// create Pxy
	arm_mat_mult_f32(&f->Sx, &f->C1T, &f->Pxy);

	// K = (Pxy / SyT) / Sy
	matrixDiv_f32(&f->K, &f->Pxy, &f->SyT, &f->Q, &f->R, &f->AQ);
	matrixDiv_f32(&f->K, &f->K, &f->Sy, &f->Q, &f->R, &f->AQ);

	// x = x + k(ym - y)
	for (i = 0; i < M; i++)
		inov[i] = ym[i] - y[i];
	arm_mat_mult_f32(&f->K, &f->inov, &f->xUpdate);

	for (i = 0; i < S; i++)
		x[i] += xUpdate[i];

	// build final QR matrix
	//	rows = s
	//	cols = s + n + s + n
	//	use Q as temporary result storage

	f->Q.numRows = S;
	f->Q.numCols = S;
	arm_mat_mult_f32(&f->K, &f->C1, &f->Q);
	for (i = 0; i < S; i++) {
		int rOffset = i*(2*S + 2*N);

		for (j = 0; j < S; j++)
			qrFinal[rOffset + j] = Sx[i*S + j] - Q[i*S + j];
	}

	f->Q.numRows = S;
	f->Q.numCols = N;
	arm_mat_mult_f32(&f->K, &f->C2, &f->Q);
	for (i = 0; i < S; i++) {
		int rOffset = i*(2*S + 2*N);

		for (j = 0; j < N; j++)
			qrFinal[rOffset + S+j] = Q[i*N + j];
	}

	f->Q.numRows = S;
	f->Q.numCols = S+N;
	arm_mat_mult_f32(&f->K, &f->D, &f->Q);
	for (i = 0; i < S; i++) {
		int rOffset = i*(2*S + 2*N);

		for (j = 0; j < S+N; j++)
			qrFinal[rOffset + S+N+j] = Q[i*(S+N) + j];
	}

	// Sx = qr([Sx-K*C1 K*C2 K*D]')
	// this method is not susceptable to numeric instability like the Cholesky is
	qrDecompositionT_f32(&f->qrFinal, NULL, &f->SxT);	// with transposition
	arm_mat_trans_f32(&f->SxT, &f->Sx);
}



void navUkfNormalizeVec3(float *vr, float *v) {
    float norm;

    norm = __sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

    vr[0] = v[0] / norm;
    vr[1] = v[1] / norm;
    vr[2] = v[2] / norm;
}
void navUkfNormalizeQuat(float *qr, float *q) {
    float norm;

    norm = 1.0f / __sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

    qr[0] *= norm;
    qr[1] *= norm;
    qr[2] *= norm;
    qr[3] *= norm;
}
void navUkfRotateVectorByQuat(float *vr, float *v, float *q) {
    float w, x, y, z;

    w = q[0];
    x = q[1];
    y = q[2];
    z = q[3];

    vr[0] = w*w*v[0] + 2.0f*y*w*v[2] - 2.0f*z*w*v[1] + x*x*v[0] + 2.0f*y*x*v[1] + 2.0f*z*x*v[2] - z*z*v[0] - y*y*v[0];
    vr[1] = 2.0f*x*y*v[0] + y*y*v[1] + 2.0f*z*y*v[2] + 2.0f*w*z*v[0] - z*z*v[1] + w*w*v[1] - 2.0f*x*w*v[2] - x*x*v[1];
    vr[2] = 2.0f*x*z*v[0] + 2.0f*y*z*v[1] + z*z*v[2] - 2.0f*w*y*v[0] - y*y*v[2] + 2.0f*w*x*v[1] - x*x*v[2] + w*w*v[2];
}
void navUkfRotateVectorByRevQuat(float *vr, float *v, float *q) {
    float qc[4];

    qc[0] = q[0];
    qc[1] = -q[1];
    qc[2] = -q[2];
    qc[3] = -q[3];

    navUkfRotateVectorByQuat(vr, v, qc);
}
void navUkfRotateVecByMatrix(float *vr, float *v, float *m) {
    vr[0] = m[0*3 + 0]*v[0] + m[0*3 + 1]*v[1] + m[0*3 + 2]*v[2];
    vr[1] = m[1*3 + 0]*v[0] + m[1*3 + 1]*v[1] + m[1*3 + 2]*v[2];
    vr[2] = m[2*3 + 0]*v[0] + m[2*3 + 1]*v[1] + m[2*3 + 2]*v[2];
}
static void navUkfRotateVecByRevMatrix(float *vr, float *v, float *m) {
    vr[0] = m[0*3 + 0]*v[0] + m[1*3 + 0]*v[1] + m[2*3 + 0]*v[2];
    vr[1] = m[0*3 + 1]*v[0] + m[1*3 + 1]*v[1] + m[2*3 + 1]*v[2];
    vr[2] = m[0*3 + 2]*v[0] + m[1*3 + 2]*v[1] + m[2*3 + 2]*v[2];
}
static void navUkfQuatToMatrix(float *m, float *q, int normalize) {
    float sqw = q[0]*q[0];
    float sqx = q[1]*q[1];
    float sqy = q[2]*q[2];
    float sqz = q[3]*q[3];
    float tmp1, tmp2;
    float invs;

    // get the invert square length
    if (normalize)
	invs = 1.0f / (sqx + sqy + sqz + sqw);
    else
	invs = 1.0f;

    // rotation matrix is scaled by inverse square length
    m[0*3 + 0] = ( sqx - sqy - sqz + sqw) * invs;
    m[1*3 + 1] = (-sqx + sqy - sqz + sqw) * invs;
    m[2*3 + 2] = (-sqx - sqy + sqz + sqw) * invs;

    tmp1 = q[1]*q[2];
    tmp2 = q[3]*q[0];
    m[1*3 + 0] = 2.0f * (tmp1 + tmp2) * invs;
    m[0*3 + 1] = 2.0f * (tmp1 - tmp2) * invs;

    tmp1 = q[1]*q[3];
    tmp2 = q[2]*q[0];
    m[2*3 + 0] = 2.0f * (tmp1 - tmp2) * invs;
    m[0*3 + 2] = 2.0f * (tmp1 + tmp2) * invs;

    tmp1 = q[2]*q[3];
    tmp2 = q[1]*q[0];
    m[2*3 + 1] = 2.0f * (tmp1 + tmp2) * invs;
    m[1*3 + 2] = 2.0f * (tmp1 - tmp2) * invs;
}
void navUkfMatrixExtractEuler(float *m, float *yaw, float *pitch, float *roll) {
    if (m[1*3+0] > 0.998f) { // singularity at north pole
	*pitch = atan2f(m[0*3+2], m[2*3+2]);
	*yaw = M_PI/2.0f;
	*roll = 0.0f;
    } else if (m[1*3+0] < -0.998f) { // singularity at south pole
	*pitch = atan2f(m[0*3+2] ,m[2*3+2]);
	*yaw = -M_PI/2.0f;
	*roll = 0.0f;
    }
    else {
	*pitch = atan2f(-m[2*3+0] ,m[0*3+0]);
	*yaw = asinf(m[1*3+0]);
	*roll = atan2f(-m[1*3+2], m[1*3+1]);
    }
}
void navUkfQuatExtractEuler(float *q, float *yaw, float *pitch, float *roll) {
    float q0, q1, q2, q3;
    float ex, ey, ez;
    float q0z, q3z;
    float q0_y, q1_y, q2_y, q3_y;

//    q0 = q[1];
//    q1 = q[2];
//    q2 = q[3];
//    q3 = q[0];

    q0 = q[0];
    q1 = q[1];
    q2 = q[2];
    q3 = q[3];

//    *yaw = atan2f((2.0f * (q0 * q1 + q3 * q2)), (q3*q3 - q2*q2 - q1*q1 + q0*q0));
//    *pitch = asinf(-2.0f * (q0 * q2 - q1 * q3));
//    *roll = atanf((2.0f * (q1 * q2 + q0 * q3)) / (q3*q3 + q2*q2 - q1*q1 -q0*q0));

	ex = atan2f((q2*q3+q0*q1), (q0*q0+q3*q3-0.5f));
	ez = -atan2f((q1*q2-q0*q3),(q0*q0+q2*q2-0.5f));

	//создаем кватернион рыскания
	q0z = cosf(ez/2.0f);
	q3z = -sinf(ez/2.0f);
	//для оси У придется повозиться, убираем вращение по оси Z из глобального кватерниона
	q0_y = q0z*q0  - q3z*q3;
	q1_y = q0z*q1 - q3z*q2;
	q2_y = q0z*q2  + q3z*q1;
	q3_y = q0z*q3  + q3z*q0;

	*yaw = ez;
	*pitch = -atan2f((q3_y*q1_y+q2_y*q0_y), (q0_y*q0_y+q3_y*q3_y-0.5f));
	*roll = ex;
}
static void navUkfRotateQuat(float *qOut, float *qIn, float *rate) {
    float q[4];
    float r[3];

    r[0] = rate[0] * -0.5f;
    r[1] = rate[1] * -0.5f;
    r[2] = rate[2] * -0.5f;

    q[0] = qIn[0];
    q[1] = qIn[1];
    q[2] = qIn[2];
    q[3] = qIn[3];

    // rotate
    qOut[0] =       q[0] + r[0]*q[1] + r[1]*q[2] + r[2]*q[3];
    qOut[1] = -r[0]*q[0] +      q[1] - r[2]*q[2] + r[1]*q[3];
    qOut[2] = -r[1]*q[0] + r[2]*q[1] +      q[2] - r[0]*q[3];
    qOut[3] = -r[2]*q[0] - r[1]*q[1] + r[0]*q[2] +      q[3];
}
void crossVector(float *w, float *r, float *wr)
    {
	wr[0] = -w[2]*r[1] + w[1]*r[2];
	wr[1] = w[2]*r[0] - w[0]*r[2];
	wr[2] = -w[1]*r[0] + w[0]*r[1];
    }
void navUkfAccUpdate(float *u, float *x, float *noise, float *y) {
    navUkfRotateVectorByRevQuat(y, navUkfData.v0a, &x[UKF_STATE_Q1]);
    y[0] += noise[0];
    y[1] += noise[1];
    y[2] += noise[2];
}
void navUkfMagUpdate(float *u, float *x, float *noise, float *y) {
    navUkfRotateVectorByRevQuat(y, navUkfData.v0m, &x[UKF_STATE_Q1]);
    y[0] += noise[0];
    y[1] += noise[1];
    y[2] += noise[2];
}
void simDoAccUpdate(float accX, float accY, float accZ) {
    float noise[3];        // measurement variance
    float y[3];            // measurement(s)
    float norm;

    // remove bias
    accX += UKF_ACC_BIAS_X;
    accY += UKF_ACC_BIAS_Y;
    accZ += UKF_ACC_BIAS_Z;

    // normalize vector
    norm =  __sqrtf(accX*accX + accY*accY + accZ*accZ);
    y[0] = accX / norm;
    y[1] = accY / norm;
    y[2] = accZ / norm;

    noise[0] = UKF_ACC_N + fabsf(GRAVITY - norm) * UKF_DIST_N;
    if (!(supervisorState & STATE_FLYING)) {
	accX -= UKF_ACC_BIAS_X;
	accY -= UKF_ACC_BIAS_Y;
	noise[0] *= 0.001f;
    }

    noise[1] = noise[0];
    noise[2] = noise[0];



    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfAccUpdate);
}
float navUkfPresToAlt(float pressure) {
    return (1.0f -  powf(pressure / UKF_P0, 0.19f)) * (1.0f / 22.558e-6f);
}
void navUkfPresUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[UKF_STATE_PRES_ALT] + noise[0]; // return altitude
}
void navUkfPresGPSAltUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[UKF_STATE_PRES_ALT] + noise[0];// return pres altitude
    y[1] = x[UKF_STATE_POSD] + noise[1]; // return GPS altitude
}
void simDoPresUpdate(float pres) {
    float noise[2];        // measurement variance
    float y[2];            // measurment(s)

    noise[0] = UKF_ALT_N;

    noise[1] = noise[0];

    //y[0] = Altitude;
    //y[0] = navUkfPresToAlt(pres);//pravka
    y[0] = pres;
    y[1] = y[0];

    // if GPS altitude data has been available, only update pressure altitude
    if (runData.presAltOffset != 0.0f)
	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 1, 1, noise, navUkfPresUpdate);
    // otherwise update pressure and GPS altitude from the single pressure reading
    else
	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 2, 2, noise, navUkfPresGPSAltUpdate);
}
void simDoMagUpdate(float magX, float magY, float magZ) {
    float noise[3];        // measurement variance
    float y[3];            // measurement(s)
    float norm;

    noise[0] = UKF_MAG_N;
    if (!(supervisorState & STATE_FLYING))
	noise[0] *= 0.001f;

    noise[1] = noise[0];
    noise[2] = noise[0];

    // normalize vector
    norm = 1.0f / __sqrtf(magX*magX + magY*magY + magZ*magZ);
    y[0] = magX * norm;
    y[1] = magY * norm;
    y[2] = magZ * norm;

    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfMagUpdate);
    matrix_equ(adrmatrix_Pxy, sizeof(matrix_Pxy)>>2, matrix_Pxy);
    matrix_equ(adrmatrix_Sx, sizeof(matrix_Sx)>>2, matrix_Sx);
    matrix_equ(adrmatrix_Sy, sizeof(matrix_Sy)>>2, matrix_Sy);
}
void GPSLocGlob(float *vloc, float *vglob, float *q) {
    float sqw = q[0]*q[0];
    float sqx = q[1]*q[1];
    float sqy = q[2]*q[2];
    float sqz = q[3]*q[3];
    float tmp1, tmp2;
    float invs;
    float m[9];

    invs = 1.0f / (sqx + sqy + sqz + sqw);

    // rotation matrix is scaled by inverse square length
    m[0*3 + 0] = ( sqx - sqy - sqz + sqw) * invs;
    m[1*3 + 1] = (-sqx + sqy - sqz + sqw) * invs;
    m[2*3 + 2] = (-sqx - sqy + sqz + sqw) * invs;

    tmp1 = q[1]*q[2];
    tmp2 = q[3]*q[0];
    m[1*3 + 0] = 2.0f * (tmp1 + tmp2) * invs;
    m[0*3 + 1] = 2.0f * (tmp1 - tmp2) * invs;

    tmp1 = q[1]*q[3];
    tmp2 = q[2]*q[0];
    m[2*3 + 0] = 2.0f * (tmp1 - tmp2) * invs;
    m[0*3 + 2] = 2.0f * (tmp1 + tmp2) * invs;

    tmp1 = q[2]*q[3];
    tmp2 = q[1]*q[0];
    m[2*3 + 1] = 2.0f * (tmp1 + tmp2) * invs;
    m[1*3 + 2] = 2.0f * (tmp1 - tmp2) * invs;

    navUkfRotateVecByMatrix(vglob, vloc, m);



}
void navUkfGpsPosUpdate(uint32_t gpsMicros, float y_gps, float x_gps, float alt, float hAcc, float vAcc) {
    float y[3];
    float noise[3];
    float posDelta[3];
    float velDelta[3];

    float abDelta[3];
    float gbDelta[3];
    float qDelta[4];

    int histIndex;


	// determine how far back this GPS position update came from
	histIndex = (timerMicros() - (gpsMicros + UKF_POS_DELAY)) / (int)(1e6f * DeltaTimeUKF);
	histIndex = navUkfData.navHistIndex - histIndex;
	if (histIndex < 0)
	    histIndex += UKF_HIST;
	if (histIndex < 0 || histIndex >= UKF_HIST)
	    histIndex = 0;



	y[0] = y_gps;
	y[1] = x_gps;
	y[2] = alt;

	// calculate delta from current position
	posDelta[0] = UKF_POSN - navUkfData.posN[histIndex];
	posDelta[1] = UKF_POSE - navUkfData.posE[histIndex];
	posDelta[2] = UKF_POSD - navUkfData.posD[histIndex];
		    #ifdef UKF_VEL_POS_USE
		    // calculate delta from current position
		    velDelta[0] = UKF_VELN - navUkfData.velN[histIndex];
		    velDelta[1] = UKF_VELE - navUkfData.velE[histIndex];
		    velDelta[2] = UKF_VELD - navUkfData.velD[histIndex];
		    #endif
		    #ifdef UKF_BIAS_QUAT_USE
		    // calculate delta from current position
		    abDelta[0] = UKF_ACC_BIAS_X - navUkfData.abiasx[histIndex];
		    abDelta[1] = UKF_ACC_BIAS_Y - navUkfData.abiasy[histIndex];
		    abDelta[2] = UKF_ACC_BIAS_Z - navUkfData.abiasz[histIndex];
		    // calculate delta from current position
		    gbDelta[0] = UKF_GYO_BIAS_X - navUkfData.gbiasx[histIndex];
		    gbDelta[1] = UKF_GYO_BIAS_Y - navUkfData.gbiasy[histIndex];
		    gbDelta[2] = UKF_GYO_BIAS_Z - navUkfData.gbiasz[histIndex];
		    // calculate delta from current position
		    qDelta[0] = UKF_Q1 - navUkfData.q0[histIndex];
		    qDelta[1] = UKF_Q2 - navUkfData.q1[histIndex];
		    qDelta[2] = UKF_Q3 - navUkfData.q2[histIndex];
		    qDelta[3] = UKF_Q4 - navUkfData.q3[histIndex];
		    #endif

	// set current position state to historic data
	UKF_POSN = navUkfData.posN[histIndex];
	UKF_POSE = navUkfData.posE[histIndex];
	UKF_POSD = navUkfData.posD[histIndex];
		    #ifdef UKF_VEL_POS_USE
		    // set current position state to historic data
		    UKF_VELN = navUkfData.velN[histIndex];
		    UKF_VELE = navUkfData.velE[histIndex];
		    UKF_VELD = navUkfData.velD[histIndex];
		    #endif
		    #ifdef UKF_BIAS_QUAT_USE
		    // set current position state to historic data
		    UKF_ACC_BIAS_X = navUkfData.abiasx[histIndex];
		    UKF_ACC_BIAS_Y = navUkfData.abiasy[histIndex];
		    UKF_ACC_BIAS_Z = navUkfData.abiasz[histIndex];
		    // set current position state to historic data
		    UKF_GYO_BIAS_X = navUkfData.gbiasx[histIndex];
		    UKF_GYO_BIAS_Y = navUkfData.gbiasy[histIndex];
		    UKF_GYO_BIAS_Z = navUkfData.gbiasz[histIndex];
		    // set current position state to historic data
		    UKF_Q1 = navUkfData.q0[histIndex];
		    UKF_Q2 = navUkfData.q1[histIndex];
		    UKF_Q3 = navUkfData.q2[histIndex];
		    UKF_Q4 = navUkfData.q3[histIndex];
		    #endif

		    float tdop, edop, ndop, vdop;
			//taskENTER_CRITICAL();
			xSemaphoreTake(xGPS_UKF_Mutex, portMAX_DELAY);
			tdop = GPS_tDopf;
			ndop = GPS_nDopf;
			edop = GPS_eDopf;
			vdop = GPS_vDopf;
			xSemaphoreGive(xGPS_UKF_Mutex);
			//taskEXIT_CRITICAL();

	noise[0] = UKF_GPS_POS_N + hAcc * __sqrtf(tdop*tdop + ndop*ndop) * UKF_GPS_POS_M_N;
	noise[1] = UKF_GPS_POS_N + hAcc * __sqrtf(tdop*tdop + edop*edop) * UKF_GPS_POS_M_N;
	noise[2] = UKF_GPS_ALT_N + vAcc * __sqrtf(tdop*tdop + vdop*vdop) * UKF_GPS_ALT_M_N;

	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfPosUpdate);

	// add the historic position delta back to the current state
	UKF_POSN += posDelta[0];
	UKF_POSE += posDelta[1];
	UKF_POSD += posDelta[2];
		    #ifdef UKF_VEL_POS_USE
		    // add the historic position delta back to the current state
		    UKF_VELN += velDelta[0];
		    UKF_VELE += velDelta[1];
		    UKF_VELD += velDelta[2];
		    #endif
		    #ifdef UKF_BIAS_QUAT_USE
		    // set current position state to historic data
		    UKF_ACC_BIAS_X += abDelta[0];
		    UKF_ACC_BIAS_Y += abDelta[1];
		    UKF_ACC_BIAS_Z += abDelta[2];
		    // set current position state to historic data
		    UKF_GYO_BIAS_X += gbDelta[0];
		    UKF_GYO_BIAS_Y += gbDelta[1];
		    UKF_GYO_BIAS_Z += gbDelta[2];
		    // set current position state to historic data
		    UKF_Q1 += qDelta[0];
		    UKF_Q2 += qDelta[1];
		    UKF_Q3 += qDelta[2];
		    UKF_Q4 += qDelta[3];
		    #endif
}
void navUkfGpsVelUpdate(uint32_t gpsMicros, float velN, float velE, float velD, float sAcc) {
    float y[3];
    float noise[3];
    float velDelta[3];
    float posDelta[3];

    float abDelta[3];
    float gbDelta[3];
    float qDelta[4];

    int histIndex, histIndex2;

    y[0] = velN;
    y[1] = velE;
    y[2] = velD;

    // determine how far back this GPS velocity update came from
    histIndex = (timerMicros() - (gpsMicros + UKF_VEL_DELAY)) / (int)(1e6f * DeltaTimeUKF);
    histIndex2 = histIndex-2;
    histIndex = navUkfData.navHistIndex - histIndex;
    histIndex2 = navUkfData.navHistIndex - histIndex2;
    if (histIndex < 0)
	histIndex += UKF_HIST;
    if (histIndex < 0 || histIndex >= UKF_HIST){
	histIndex = 0;}

    if (histIndex2 < 0)
	histIndex2 += UKF_HIST;
    if (histIndex2 < 0 || histIndex2 >= UKF_HIST){
	histIndex2 = 0;}

    /**************************************************/
    Histindex_gyr = histIndex;
    /**************************************************/
    // calculate delta from current position
    velDelta[0] = UKF_VELN - navUkfData.velN[histIndex];
    velDelta[1] = UKF_VELE - navUkfData.velE[histIndex];
    velDelta[2] = UKF_VELD - navUkfData.velD[histIndex];
	    #ifdef UKF_VEL_POS_USE
	    // calculate delta from current position
	    posDelta[0] = UKF_POSN - navUkfData.posN[histIndex];
	    posDelta[1] = UKF_POSE - navUkfData.posE[histIndex];
	    posDelta[2] = UKF_POSD - navUkfData.posD[histIndex];
	    #endif
	    #ifdef UKF_BIAS_QUAT_USE
	    // calculate delta from current position
	    abDelta[0] = UKF_ACC_BIAS_X - navUkfData.abiasx[histIndex];
	    abDelta[1] = UKF_ACC_BIAS_Y - navUkfData.abiasy[histIndex];
	    abDelta[2] = UKF_ACC_BIAS_Z - navUkfData.abiasz[histIndex];
	    // calculate delta from current position
	    gbDelta[0] = UKF_GYO_BIAS_X - navUkfData.gbiasx[histIndex];
	    gbDelta[1] = UKF_GYO_BIAS_Y - navUkfData.gbiasy[histIndex];
	    gbDelta[2] = UKF_GYO_BIAS_Z - navUkfData.gbiasz[histIndex];
	    // calculate delta from current position
	    qDelta[0] = UKF_Q1 - navUkfData.q0[histIndex];
	    qDelta[1] = UKF_Q2 - navUkfData.q1[histIndex];
	    qDelta[2] = UKF_Q3 - navUkfData.q2[histIndex];
	    qDelta[3] = UKF_Q4 - navUkfData.q3[histIndex];
	    #endif


    y[0] = velN;
    y[1] = velE;
    y[2] = velD;


    // set current position state to historic data
    UKF_VELN = navUkfData.velN[histIndex];
    UKF_VELE = navUkfData.velE[histIndex];
    UKF_VELD = navUkfData.velD[histIndex];
//    velHistX = UKF_VELE;
//    velHistY = UKF_VELN;
//    cousreHist = atan2f(velHistY, velHistX);
	    #ifdef UKF_VEL_POS_USE
	    // set current position state to historic data
	    UKF_POSN = navUkfData.posN[histIndex];
	    UKF_POSE = navUkfData.posE[histIndex];
	    UKF_POSD = navUkfData.posD[histIndex];
	    #endif
	    #ifdef UKF_BIAS_QUAT_USE
	    // set current position state to historic data
	    UKF_ACC_BIAS_X = navUkfData.abiasx[histIndex];
	    UKF_ACC_BIAS_Y = navUkfData.abiasy[histIndex];
	    UKF_ACC_BIAS_Z = navUkfData.abiasz[histIndex];
	    // set current position state to historic data
	    UKF_GYO_BIAS_X = navUkfData.gbiasx[histIndex];
	    UKF_GYO_BIAS_Y = navUkfData.gbiasy[histIndex];
	    UKF_GYO_BIAS_Z = navUkfData.gbiasz[histIndex];
	    // set current position state to historic data
	    UKF_Q1 = navUkfData.q0[histIndex];
	    UKF_Q2 = navUkfData.q1[histIndex];
	    UKF_Q3 = navUkfData.q2[histIndex];
	    UKF_Q4 = navUkfData.q3[histIndex];
	    #endif

	    float tdop, edop, ndop, vdop;
		//taskENTER_CRITICAL();
		xSemaphoreTake(xGPS_UKF_Mutex, portMAX_DELAY);
		tdop = GPS_tDopf;
		ndop = GPS_nDopf;
		edop = GPS_eDopf;
		vdop = GPS_vDopf;
		xSemaphoreGive(xGPS_UKF_Mutex);
		//taskEXIT_CRITICAL();

//    noise[0] = UKF_GPS_VEL_N + sAcc * __sqrtf(GPS_tDopf*GPS_tDopf + GPS_nDopf*GPS_nDopf) * UKF_GPS_VEL_M_N;
//    noise[1] = UKF_GPS_VEL_N + sAcc * __sqrtf(GPS_tDopf*GPS_tDopf + GPS_eDopf*GPS_eDopf) * UKF_GPS_VEL_M_N;
//    noise[2] = UKF_GPS_VD_N  + sAcc * __sqrtf(GPS_tDopf*GPS_tDopf + GPS_vDopf*GPS_vDopf) * UKF_GPS_VD_M_N;

	noise[0] = UKF_GPS_VEL_N + sAcc * __sqrtf(tdop*tdop + ndop*ndop) * UKF_GPS_VEL_M_N;
	noise[1] = UKF_GPS_VEL_N + sAcc * __sqrtf(tdop*tdop + edop*edop) * UKF_GPS_VEL_M_N;
	noise[2] = UKF_GPS_VD_N + sAcc * __sqrtf(tdop*tdop + vdop*vdop) * UKF_GPS_VD_M_N;

    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfVelUpdate);

    // add the historic position delta back to the current state
    UKF_VELN += velDelta[0];
    UKF_VELE += velDelta[1];
    UKF_VELD += velDelta[2];
	    #ifdef UKF_VEL_POS_USE
	    // add the historic position delta back to the current state
	    UKF_POSN += posDelta[0];
	    UKF_POSE += posDelta[1];
	    UKF_POSD += posDelta[2];
	    #endif
	    #ifdef UKF_BIAS_QUAT_USE
	    // set current position state to historic data
	    UKF_ACC_BIAS_X += abDelta[0];
	    UKF_ACC_BIAS_Y += abDelta[1];
	    UKF_ACC_BIAS_Z += abDelta[2];
	    // set current position state to historic data
	    UKF_GYO_BIAS_X += gbDelta[0];
	    UKF_GYO_BIAS_Y += gbDelta[1];
	    UKF_GYO_BIAS_Z += gbDelta[2];
	    // set current position state to historic data
	    UKF_Q1 += qDelta[0];
	    UKF_Q2 += qDelta[1];
	    UKF_Q3 += qDelta[2];
	    UKF_Q4 += qDelta[3];
	    #endif

}

void navPressureAdjust(float altitude) {
	runData.presAltOffset = altitude - ALTITUDE_UKF;
}
void navUkfPosUpdate(float *u, float *x, float *noise, float *y) {

    float xpos,ypos,zpos;
    xpos = x[UKF_STATE_POSE];
    ypos = x[UKF_STATE_POSN];
    zpos = x[UKF_STATE_POSD];
    //////
    #ifdef USE_GPS_FRAME_POS_CMPNST

    float vloc[3], vglov[3];
    vloc[0] = GPS_FRAME_POS_X;
    vloc[1] = GPS_FRAME_POS_Y;
    vloc[2] = GPS_FRAME_POS_Z;

    //GPSLocGlob(vloc, vglov, &x[UKF_STATE_Q1]);
    navUkfRotateVectorByQuat(vglov, vloc, &x[UKF_STATE_Q1]);

    xpos += vglov[0];
    ypos += vglov[1];
    zpos +=  vglov[2];

    #endif

    /////
    y[0] = ypos + noise[0]; // return position
    y[1] = xpos + noise[1];
    y[2] = zpos + noise[2];
}
void navUkfVelUpdate(float *u, float *x, float *noise, float *y) {
    float xvel, yvel, zvel;
    xvel = x[UKF_STATE_VELE];
    yvel = x[UKF_STATE_VELN];
    zvel = x[UKF_STATE_VELD];
    //////
    #ifdef USE_GPS_FRAME_VEL_CMPNST

    float vloc[3], vglov[3], w[3], wr[3];

    w[0] = (navUkfData.gx[Histindex_gyr]+UKF_GYO_BIAS_X);
    w[1] = (navUkfData.gy[Histindex_gyr]+UKF_GYO_BIAS_Y);
    w[2] = (navUkfData.gz[Histindex_gyr]+UKF_GYO_BIAS_Z);

    vloc[0] = GPS_FRAME_POS_X;
    vloc[1] = GPS_FRAME_POS_Y;
    vloc[2] = GPS_FRAME_POS_Z;

    crossVector(w, vloc, wr);


    //GPSLocGlob(wr, vglov, &x[UKF_STATE_Q1]);
    navUkfRotateVectorByQuat(vglov, wr, &x[UKF_STATE_Q1]);
    xvel += vglov[0];
    yvel += vglov[1];
    zvel -= vglov[2];

    #endif

    /////
    y[0] = yvel + noise[0]; // return velocity
    y[1] = xvel + noise[1];
    y[2] = zvel + noise[2];
}

void navUkfZeroPos(void) {
    float y[3];
    float noise[3];

    y[0] = 0.0f;
    y[1] = 0.0f;
    xSemaphoreTake(xBAROMAG_UKF_Mutex, portMAX_DELAY);
    y[2] = Altitude;
    xSemaphoreGive(xBAROMAG_UKF_Mutex);

	#ifdef USE_GPS_FRAME_POS_CMPNST
	float vloc[3], vglov[3];
	vloc[0] = GPS_FRAME_POS_X;
	vloc[1] = GPS_FRAME_POS_Y;
	vloc[2] = GPS_FRAME_POS_Z;

	//GPSLocGlob(vloc, vglov, &UKF_Q1);
	navUkfRotateVectorByQuat(vglov, vloc, &UKF_Q1);
	y[0] += vglov[1];
	y[1] += vglov[0];
	y[2] += vglov[2];

	    ccglobPosX = vglov[0];
	    ccglobPosY = vglov[1];
	    ccglobPosZ = vglov[2];

	    ccPosX = y[1];
	    ccPosY = y[0];
	    ccPosZ = y[2];
	#endif

    if (supervisorState & STATE_FLYING) {
	noise[0] = 1e1f;
	noise[1] = 1e1f;
	noise[2] = 1e1f;
    }
    else {
	noise[0] = 1e-7f;
	noise[1] = 1e-7f;
	noise[2] = 1.0f;
    }

    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfPosUpdate);
}
void navUkfZeroVel(void) {
    float y[3];
    float noise[3];

    y[0] = 0.0f;
    y[1] = 0.0f;
    y[2] = 0.0f;
	#ifdef USE_GPS_FRAME_VEL_CMPNST

	float vloc[3], vglov[3], w[3], wr[3];

	w[0] = (gxf+UKF_GYO_BIAS_X);
	w[1] = (gyf+UKF_GYO_BIAS_Y);
	w[2] = (gzf+UKF_GYO_BIAS_Z);

	vloc[0] = GPS_FRAME_POS_X;
	vloc[1] = GPS_FRAME_POS_Y;
	vloc[2] = GPS_FRAME_POS_Z;

	crossVector(w, vloc, wr);


	//GPSLocGlob(wr, vglov, &x[UKF_STATE_Q1]);
	navUkfRotateVectorByQuat(vglov, wr, &UKF_Q1);

	cclinVelX = wr[0];
	cclinVelY = wr[1];
	cclinVelZ = wr[2];

	ccGlobLinVelX = vglov[0];
	ccGlobLinVelY = vglov[1];
	ccGlobLinVelZ = vglov[2];

	y[0] = vglov[1];
	y[1] = vglov[0];
	y[2] = -vglov[2];

	#endif

    if (supervisorState & STATE_FLYING) {
	noise[0] = 5.0f;
	noise[1] = 5.0f;
	noise[2] = 2.0f;
    }
    else {
	noise[0] = 1e-7f;
	noise[1] = 1e-7f;
	noise[2] = 1e-7f;
    }

    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfVelUpdate);
}
void navUkfRateUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = -x[UKF_STATE_GYO_BIAS_X+(int)u[0]] + noise[0];
}
void navUkfZeroRate(float rate, int axis) {
    float noise[1];        // measurement variance
    float y[1];            // measurment(s)
    float u[1];		   // user data

    noise[0] = 0.00001f;
    y[0] = rate;
    u[0] = (float)axis;

    srcdkfMeasurementUpdate(navUkfData.kf, u, y, 1, 1, noise, navUkfRateUpdate);
}


void srcdkfSetVariance(srcdkf_t *f, float32_t *q, float32_t *v, float32_t *n, int nn) {
	float32_t *Sx = f->Sx.pData;
	float32_t *Sv = f->Sv.pData;
	float32_t *Sn = f->Sn.pData;
	int i;

	// state variance
	if (q)
		for (i = 0; i < f->S; i++)
			Sx[i*f->S + i] = __sqrtf(fabsf(q[i]));

	// process noise
	if (v)
		for (i = 0; i < f->V; i++)
			Sv[i*f->V + i] = __sqrtf(fabsf(v[i]));

	// observation noise
	if (n && nn) {
		// resize Sn
		f->Sn.numRows = nn;
		f->Sn.numCols = nn;

		for (i = 0; i < nn; i++)
			Sn[i*nn + i] = __sqrtf(fabsf(n[i]));
	}
}
float *srcdkfGetState(srcdkf_t *f) {
    return f->x.pData;
}
void navUkfInertialUpdate(void) {
    float u[6];
    static float gmx, gmy, gmz;

   //защитить
    //taskENTER_CRITICAL();
    xSemaphoreTake(xMPU_UKF_Mutex, portMAX_DELAY);
    u[0] = axf;
    u[1] = ayf;
    u[2] = azf;

    u[3] = gxf;
    u[4] = gyf;
    u[5] = gzf;
    xSemaphoreGive(xMPU_UKF_Mutex);
    //taskEXIT_CRITICAL();
    //

    srcdkfTimeUpdate(navUkfData.kf, u, DeltaTimeUKF);



    // store history
    navUkfData.posN[navUkfData.navHistIndex] = UKF_POSN;
    navUkfData.posE[navUkfData.navHistIndex] = UKF_POSE;
    navUkfData.posD[navUkfData.navHistIndex] = UKF_POSD;

    navUkfData.velN[navUkfData.navHistIndex] = UKF_VELN;
    navUkfData.velE[navUkfData.navHistIndex] = UKF_VELE;
    navUkfData.velD[navUkfData.navHistIndex] = UKF_VELD;
    #ifdef UKF_BIAS_QUAT_USE
    navUkfData.abiasx[navUkfData.navHistIndex] = UKF_ACC_BIAS_X;
    navUkfData.abiasy[navUkfData.navHistIndex] = UKF_ACC_BIAS_Y;
    navUkfData.abiasz[navUkfData.navHistIndex] = UKF_ACC_BIAS_Z;

    navUkfData.gbiasx[navUkfData.navHistIndex] = UKF_GYO_BIAS_X;
    navUkfData.gbiasy[navUkfData.navHistIndex] = UKF_GYO_BIAS_Y;
    navUkfData.gbiasz[navUkfData.navHistIndex] = UKF_GYO_BIAS_Z;

    navUkfData.q0[navUkfData.navHistIndex] = UKF_Q1;
    navUkfData.q1[navUkfData.navHistIndex] = UKF_Q2;
    navUkfData.q2[navUkfData.navHistIndex] = UKF_Q3;
    navUkfData.q3[navUkfData.navHistIndex] = UKF_Q4;

    gmx = K_EXP_GYO_GPS*gmx + (1.0f - K_EXP_GYO_GPS)*gxf;
    gmy = K_EXP_GYO_GPS*gmy + (1.0f - K_EXP_GYO_GPS)*gyf;
    gmz = K_EXP_GYO_GPS*gmz + (1.0f - K_EXP_GYO_GPS)*gzf;

    navUkfData.gx[navUkfData.navHistIndex] = gmx;
    navUkfData.gy[navUkfData.navHistIndex] = gmy;
    navUkfData.gz[navUkfData.navHistIndex] = gmz;
    #endif
    navUkfData.navHistIndex = (navUkfData.navHistIndex + 1) % UKF_HIST;
}
void navUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt, int n) {
    float tmp[3], acc[3];
    float rate[3];
    float mat3x3[3*3];
    float q[4];
    int i;

    // assume out == in
    out = in;

    for (i = 0; i < n; i++) {
	// pos
	out[UKF_STATE_POSN*n + i] = in[UKF_STATE_POSN*n + i] + in[UKF_STATE_VELN*n + i] * dt;
	out[UKF_STATE_POSE*n + i] = in[UKF_STATE_POSE*n + i] + in[UKF_STATE_VELE*n + i] * dt;
	out[UKF_STATE_POSD*n + i] = in[UKF_STATE_POSD*n + i] - in[UKF_STATE_VELD*n + i] * dt;

	// pres alt
	out[UKF_STATE_PRES_ALT*n + i] = in[UKF_STATE_PRES_ALT*n + i] - in[UKF_STATE_VELD*n + i] * dt;

	// create rot matrix from current quat
	q[0] = in[UKF_STATE_Q1*n + i];
	q[1] = in[UKF_STATE_Q2*n + i];
	q[2] = in[UKF_STATE_Q3*n + i];
	q[3] = in[UKF_STATE_Q4*n + i];
	navUkfQuatToMatrix(mat3x3, q, 1);



	// acc
	tmp[0] = u[0] + in[UKF_STATE_ACC_BIAS_X*n + i];
	tmp[1] = u[1] + in[UKF_STATE_ACC_BIAS_Y*n + i];
	tmp[2] = u[2] + in[UKF_STATE_ACC_BIAS_Z*n + i];

	// rotate acc to world frame
	navUkfRotateVecByMatrix(acc, tmp, mat3x3);

	//navUkfRotateVectorByQuat(acc, tmp, q);

	acc[2] -= GRAVITY;

	// vel
	out[UKF_STATE_VELN*n + i] = in[UKF_STATE_VELN*n + i] + acc[1] * dt + noise[UKF_V_NOISE_VELN*n + i];
	out[UKF_STATE_VELE*n + i] = in[UKF_STATE_VELE*n + i] + acc[0] * dt + noise[UKF_V_NOISE_VELE*n + i];
	out[UKF_STATE_VELD*n + i] = in[UKF_STATE_VELD*n + i] - acc[2] * dt + noise[UKF_V_NOISE_VELD*n + i];
	//	out[UKF_STATE_VELD*n + i] = in[UKF_STATE_VELD*n + i] + acc[2] * dt + noise[UKF_V_NOISE_VELD*n + i];//pravka

	// acc bias
	out[UKF_STATE_ACC_BIAS_X*n + i] = in[UKF_STATE_ACC_BIAS_X*n + i] + noise[UKF_V_NOISE_ACC_BIAS_X*n + i] * dt;
	out[UKF_STATE_ACC_BIAS_Y*n + i] = in[UKF_STATE_ACC_BIAS_Y*n + i] + noise[UKF_V_NOISE_ACC_BIAS_Y*n + i] * dt;
	out[UKF_STATE_ACC_BIAS_Z*n + i] = in[UKF_STATE_ACC_BIAS_Z*n + i] + noise[UKF_V_NOISE_ACC_BIAS_Z*n + i] * dt;

	// rate = rate + bias + noise
	rate[0] = (u[3] + in[UKF_STATE_GYO_BIAS_X*n + i] + noise[UKF_V_NOISE_RATE_X*n + i]) * dt;
	rate[1] = (u[4] + in[UKF_STATE_GYO_BIAS_Y*n + i] + noise[UKF_V_NOISE_RATE_Y*n + i]) * dt;
	rate[2] = (u[5] + in[UKF_STATE_GYO_BIAS_Z*n + i] + noise[UKF_V_NOISE_RATE_Z*n + i]) * dt;

	// rotate quat
	navUkfRotateQuat(q, q, rate);
	out[UKF_STATE_Q1*n + i] = q[0];
	out[UKF_STATE_Q2*n + i] = q[1];
	out[UKF_STATE_Q3*n + i] = q[2];
	out[UKF_STATE_Q4*n + i] = q[3];

	// gbias
	out[UKF_STATE_GYO_BIAS_X*n + i] = in[UKF_STATE_GYO_BIAS_X*n + i] + noise[UKF_V_NOISE_GYO_BIAS_X*n + i] * dt;
	out[UKF_STATE_GYO_BIAS_Y*n + i] = in[UKF_STATE_GYO_BIAS_Y*n + i] + noise[UKF_V_NOISE_GYO_BIAS_Y*n + i] * dt;
	out[UKF_STATE_GYO_BIAS_Z*n + i] = in[UKF_STATE_GYO_BIAS_Z*n + i] + noise[UKF_V_NOISE_GYO_BIAS_Z*n + i] * dt;
    }
}
void navUkfInitState(void) {
    float acc[3], mag[3];
    float estAcc[3], estMag[3];
    float m[3*3];
    static int i;


	float rotError[3];

	//supervisorState = STATE_FLYING;
	//защитить
	//taskENTER_CRITICAL();
	 xSemaphoreTake(xBAROMAG_UKF_Mutex, portMAX_DELAY);
	if((mxf != 0.0f)&&(myf != 0.0f)&&(mzf != 0.0f))
	{
	mag[0] = mxf;
	mag[1] = myf;
	mag[2] = mzf;
	}
	xSemaphoreGive(xBAROMAG_UKF_Mutex);
	xSemaphoreTake(xMPU_UKF_Mutex, portMAX_DELAY);
	acc[0] = axf;
	acc[1] = ayf;
	acc[2] = azf;
	xSemaphoreGive(xMPU_UKF_Mutex);
	//taskEXIT_CRITICAL();

	navUkfNormalizeVec3(acc, acc);
	navUkfNormalizeVec3(mag, mag);

	navUkfQuatToMatrix(m, &UKF_Q1, 1);

	// rotate gravity to body frame of reference
	navUkfRotateVecByRevMatrix(estAcc, navUkfData.v0a, m);

	// rotate mags to body frame of reference
	navUkfRotateVecByRevMatrix(estMag, navUkfData.v0m, m);

	// measured error, starting with ACC vector
	rotError[0] = -(acc[2] * estAcc[1] - estAcc[2] * acc[1]) * 1.0f;
	rotError[1] = -(acc[0] * estAcc[2] - estAcc[0] * acc[2]) * 1.0f;
	rotError[2] = -(acc[1] * estAcc[0] - estAcc[1] * acc[0]) * 1.0f;


	rotError[0] += -(mag[2] * estMag[1] - estMag[2] * mag[1]) * 1.0f;
	rotError[1] += -(mag[0] * estMag[2] - estMag[0] * mag[2]) * 1.0f;
	rotError[2] += -(mag[1] * estMag[0] - estMag[1] * mag[0]) * 1.0f;


	navUkfRotateQuat(&UKF_Q1, &UKF_Q1, rotError);
	navUkfNormalizeQuat(&UKF_Q1, &UKF_Q1);

	q0 = UKF_Q1;
	q1 = UKF_Q2;
	q2 = UKF_Q3;
	q3 = UKF_Q4;

	navUkfQuatExtractEuler(&UKF_Q1, &navUkfData.yaw, &navUkfData.pitch, &navUkfData.roll);
	yaw_start = navUkfData.yaw;

	i++;
	if (i > 5*UKF_GYO_AVG_NUM) flag_start_state_init = 2;
	//if ((rotError[0] < 0.0001f)&&(rotError[1] < 0.0001f)&&(rotError[2] < 0.0001f)) flag_start_state_init = 2;
}
void navUkfInit(void) {
    float Q[SIM_S];		// state variance
    float V[SIM_V];		// process variance
    float mag[3];


    memset((void *)&runData, 0, sizeof(runData));
    memset((void *)&navUkfData, 0, sizeof(navUkfData));

    navUkfData.v0a[0] = 0.0f;
    navUkfData.v0a[1] = 0.0f;
    navUkfData.v0a[2] = 1.0f;

    // calculate mag vector based on inclination
    mag[0] = 0.0f;
    mag[1] = cosf(INCL*DEG2RAD);
    mag[2] = -sinf(INCL*DEG2RAD);

    // rotate local mag vector to align with true north
    navUkfData.v0m[0] = mag[0] * cosf(DECL* DEG2RAD) - mag[1] * sinf(DECL* DEG2RAD);
    navUkfData.v0m[1] = mag[1] * cosf(DECL* DEG2RAD) + mag[0] * sinf(DECL* DEG2RAD);
    navUkfData.v0m[2] = mag[2];

    navUkfData.kf = srcdkfInit(SIM_S, SIM_M, SIM_V, SIM_N, navUkfTimeUpdate);
    //navUkfData.kf->

    navUkfData.x = srcdkfGetState(navUkfData.kf);

    Q[0] = UKF_VEL_Q;
    Q[1] = UKF_VEL_Q;
    Q[2] = UKF_VEL_ALT_Q;
    Q[3] = UKF_POS_Q;
    Q[4] = UKF_POS_Q;
    Q[5] = UKF_POS_ALT_Q;
    Q[6] = UKF_ACC_BIAS_Q;
    Q[7] = UKF_ACC_BIAS_Q;
    Q[8] = UKF_ACC_BIAS_Q;
    Q[9] = UKF_GYO_BIAS_Q;
    Q[10] = UKF_GYO_BIAS_Q;
    Q[11] = UKF_GYO_BIAS_Q;
    Q[12] = UKF_QUAT_Q;
    Q[13] = UKF_QUAT_Q;
    Q[14] = UKF_QUAT_Q;
    Q[15] = UKF_QUAT_Q;
    Q[16] = UKF_PRES_ALT_Q;

    V[UKF_V_NOISE_ACC_BIAS_X] = UKF_ACC_BIAS_V;
    V[UKF_V_NOISE_ACC_BIAS_Y] = UKF_ACC_BIAS_V;
    V[UKF_V_NOISE_ACC_BIAS_Z] = UKF_ACC_BIAS_V;
    V[UKF_V_NOISE_GYO_BIAS_X] = UKF_GYO_BIAS_V;
    V[UKF_V_NOISE_GYO_BIAS_Y] = UKF_GYO_BIAS_V;
    V[UKF_V_NOISE_GYO_BIAS_Z] = UKF_GYO_BIAS_V;
    V[UKF_V_NOISE_RATE_X] = UKF_RATE_V;
    V[UKF_V_NOISE_RATE_Y] = UKF_RATE_V;
    V[UKF_V_NOISE_RATE_Z] = UKF_RATE_V;
    V[UKF_V_NOISE_VELN] = UKF_VEL_V;
    V[UKF_V_NOISE_VELE] = UKF_VEL_V;
    V[UKF_V_NOISE_VELD] = UKF_ALT_VEL_V;

    srcdkfSetVariance(navUkfData.kf, Q, V, 0, 0);

    // vel
    UKF_VELN = 0.0f;
    UKF_VELE = 0.0f;
    UKF_VELD = 0.0f;

    // pos
    UKF_POSN = 0.0f;
    UKF_POSE = 0.0f;
    UKF_POSD = Altitude;
    //UKF_POSD = navUkfPresToAlt(Pressure);//pravka

    // acc bias
    UKF_ACC_BIAS_X = 0.0f;
    UKF_ACC_BIAS_Y = 0.0f;
    UKF_ACC_BIAS_Z = 0.0f;

    // gyo bias
    UKF_GYO_BIAS_X = 0.0f;
    UKF_GYO_BIAS_Y = 0.0f;
    UKF_GYO_BIAS_Z = 0.0f;


    // quat
    UKF_Q1 =  1.0f;
    UKF_Q2 =  0.0f;
    UKF_Q3 =  0.0f;
    UKF_Q4 =  0.0f;

    UKF_PRES_ALT = Altitude;
    //UKF_PRES_ALT = navUkfPresToAlt(Pressure);//pravka


    flag_start_state_init = 1;
    //navUkfInitState();

}
void navUkfFinish(void) {
    navUkfNormalizeQuat(&UKF_Q1, &UKF_Q1);


    uint8_t i=0;
    for (i = 0; i < SIM_S; i++)
	{
	    if(isnanf(navUkfData.x[i]))
		{
		    StateNavUKF |= UKF_NAN;
		}
	}
    //taskENTER_CRITICAL();
    xSemaphoreTake(xUKF_PID_Mutex, portMAX_DELAY);
    q0 = UKF_Q1;
    q1 = UKF_Q2;
    q2 = UKF_Q3;
    q3 = UKF_Q4;

    DataUKF.posx = UKF_POSE;
    DataUKF.posy = UKF_POSN;
    DataUKF.posz = UKF_POSD;

    DataUKF.velx = UKF_VELE;
    DataUKF.vely = UKF_VELN;
    DataUKF.velz = -UKF_VELD;
    //taskEXIT_CRITICAL();
    //taskENTER_CRITICAL();
    DataUKF.abiasx = UKF_ACC_BIAS_X;
    DataUKF.abiasy = UKF_ACC_BIAS_Y;
    DataUKF.abiasz = UKF_ACC_BIAS_Z;

    DataUKF.baro_alt = ALT_POS;
    DataUKF.baro_vel = -ALT_VEL;

    DataUKF.tru_pos_z = ALTITUDE_UKF;
    DataUKF.tru_vel_z = -VELOCITYD_UKF;
    DataUKF.pres_alt = UKF_PRES_ALT;

    gxbias = UKF_GYO_BIAS_X;
    gybias = UKF_GYO_BIAS_Y;
    gzbias = UKF_GYO_BIAS_Z;
    xSemaphoreGive(xUKF_PID_Mutex);
    //taskEXIT_CRITICAL();



    navUkfQuatExtractEuler(&UKF_Q1, &navUkfData.yaw, &navUkfData.pitch, &navUkfData.roll);

    //taskENTER_CRITICAL();
    xSemaphoreTake(xUKF_PID_Mutex, portMAX_DELAY);
    axisx = navUkfData.roll;
    axisy = navUkfData.pitch;
    axisz = navUkfData.yaw;

    xSemaphoreGive(xUKF_PID_Mutex);
    //taskEXIT_CRITICAL();


    //////////////////////////////////////////////////////
//    float vloc[3], vglov[3], w[3], wr[3];
//
//    w[0] = gxf+gxbias;
//    w[1] = gyf+gybias;
//    w[2] = gzf+gzbias;
//
//    vloc[0] = GPS_FRAME_POS_X;
//    vloc[1] = GPS_FRAME_POS_Y;
//    vloc[2] = GPS_FRAME_POS_Z;
//
//    crossVector(w, vloc, wr);
//
//    GPSLocGlob(wr, vglov, &UKF_Q1);
//
//    cclinVelX = wr[0];
//    cclinVelY = wr[1];
//    cclinVelZ = wr[2];
//
//    ccGlobLinVelX = vglov[0];
//    ccGlobLinVelY = vglov[1];
//    ccGlobLinVelZ = vglov[2];
//
//
//
//    GPSLocGlob(vloc, vglov, &UKF_Q1);
//
//    ccglobPosX = vglov[0];
//    ccglobPosY = vglov[1];
//    ccglobPosZ = vglov[2];

    //////////////////////////////////////////////////////


//	float cx, sx, cy, sy;
//	//cx = cosf(axisx);
//	sx = sinf(axisx);
//	//cy = cosf(axisy);
//	sy = sinf(axisy);
////
//	g_mix_z = (gzf+UKF_GYO_BIAS_Z)*mq9 + (gxf+UKF_GYO_BIAS_X)*sy + (gyf+UKF_GYO_BIAS_Y)*sx;

    //taskENTER_CRITICAL();
    xSemaphoreTake(xUKF_PID_Mutex, portMAX_DELAY);
    navUkfData.yawCos = cosf(navUkfData.yaw);
    navUkfData.yawSin = sinf(navUkfData.yaw);
    xSemaphoreGive(xUKF_PID_Mutex);
    //taskEXIT_CRITICAL();

    navUkfData.yaw *= RAD2DEG;
    navUkfData.pitch *= RAD2DEG;
    navUkfData.roll *= RAD2DEG;

    //    x' = x cos f - y sin f
    //    y' = y cos f + x sin f

}



void altUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt, int n) {
    float acc;
    int i;

    // assume out == in
    out = in;

    for (i = 0; i < n; i++) {
        acc = u[0] + in[ALT_STATE_BIAS*n + i];

        out[ALT_STATE_BIAS*n + i] = in[ALT_STATE_BIAS*n + i] + (noise[ALT_NOISE_BIAS*n + i] * dt);
        out[ALT_STATE_VEL*n + i] = in[ALT_STATE_VEL*n + i] - (acc * dt) + (noise[ALT_NOISE_VEL*n + i] * dt);
        //out[ALT_STATE_VEL*n + i] = in[ALT_STATE_VEL*n + i] + (acc * dt) + (noise[ALT_NOISE_VEL*n + i] * dt); //pravka
        out[ALT_STATE_POS*n + i] = in[ALT_STATE_POS*n + i] - (in[ALT_STATE_VEL*n + i] * dt) + (acc * dt * dt * 0.5f);
    }
}
void altUkfPresUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[ALT_STATE_POS] + noise[0];     // return altitude
}
static void altDoPresUpdate(float measuredPres) {
    float noise;        // measurement variance
    float y;            // measurment

    noise = ALT_PRES_NOISE;
   // y = Altitude;
    y = measuredPres;
    //y = navUkfPresToAlt(measuredPres);//pravka

    srcdkfMeasurementUpdate(altUkfData.kf, 0, &y, 1, 1, &noise, altUkfPresUpdate);
}
void altUkfProcess(float measuredPres) {
    float accIn[3];
    float acc[3];

    //taskENTER_CRITICAL();
    xSemaphoreTake(xMPU_UKF_Mutex, portMAX_DELAY);
    accIn[0] = axf;// + UKF_ACC_BIAS_X;
    accIn[1] = ayf;// + UKF_ACC_BIAS_Y;
    accIn[2] = azf;// + UKF_ACC_BIAS_Z;
    xSemaphoreGive(xMPU_UKF_Mutex);
    //taskEXIT_CRITICAL();

    // rotate acc to world frame
    navUkfRotateVectorByQuat(acc, accIn, &UKF_Q1);
    acc[2] -= GRAVITY;

    srcdkfTimeUpdate(altUkfData.kf, &acc[2], DeltaTimeUKF);

    altDoPresUpdate(measuredPres);
}
void altUkfInit(void) {
    float Q[ALT_S];		// state variance
    float V[ALT_V];		// process variance

    memset((void *)&altUkfData, 0, sizeof(altUkfData));

    altUkfData.kf = srcdkfInit(ALT_S, ALT_M, ALT_V, ALT_N, altUkfTimeUpdate);

    altUkfData.x = srcdkfGetState(altUkfData.kf);

    Q[ALT_STATE_POS] = 5.0f;
    Q[ALT_STATE_VEL] = 1e-6f;
    Q[ALT_STATE_BIAS] = 0.05f;

    V[ALT_NOISE_BIAS] = ALT_BIAS_NOISE;
    V[ALT_NOISE_VEL] = ALT_VEL_NOISE;

    srcdkfSetVariance(altUkfData.kf, Q, V, 0, 0);

    ALT_POS = Altitude;
    //ALT_POS = navUkfPresToAlt(Pressure);//pravka
    ALT_VEL = 0.0f;
    ALT_BIAS = 0.0f;
}







void quatMultiply(float *qr, float *q1, float *q2) {
    qr[0] = q2[0]*q1[0] - q2[1]*q1[1] - q2[2]*q1[2] - q2[3]*q1[3];
    qr[1] = q2[0]*q1[1] + q2[1]*q1[0] - q2[2]*q1[3] + q2[3]*q1[2];
    qr[2] = q2[0]*q1[2] + q2[1]*q1[3] + q2[2]*q1[0] - q2[3]*q1[1];
    qr[3] = q2[0]*q1[3] - q2[1]*q1[2] + q2[2]*q1[1] + q2[3]*q1[0];
}
void eulerToQuatYPR(float *q, float *angles) {
    float32_t cy, cp, cr;
    float32_t sy, sp, sr;

    angles[0] *= DEG2RAD * 0.5f;
    angles[1] *= DEG2RAD * 0.5f;
    angles[2] *= DEG2RAD * 0.5f;

    cy = cosf(angles[0]);
    cp = cosf(angles[1]);
    cr = cosf(angles[2]);

    sy = sinf(angles[0]);
    sp = sinf(angles[1]);
    sr = sinf(angles[2]);

    q[0] = cy*cp*cr + sy*sp*sr;
    q[1] = cy*cp*sr - sy*sp*cr;
    q[2] = cy*sp*cr + sy*cp*sr;
    q[3] = sy*cp*cr - cy*sp*sr;
}
void eulerToQuatRPY(float *q, float *angles) {
    float32_t cy, cp, cr;
    float32_t sy, sp, sr;

    angles[0] *= DEG2RAD * 0.5f;
    angles[1] *= DEG2RAD * 0.5f;
    angles[2] *= DEG2RAD * 0.5f;

    cy = cosf(angles[0]);
    cp = cosf(angles[1]);
    cr = cosf(angles[2]);

    sy = sinf(angles[0]);
    sp = sinf(angles[1]);
    sr = sinf(angles[2]);

    q[0] = cr*cp*cy - sr*sp*sy;
    q[1] = cr*sp*sy + sr*cp*cy;
    q[2] = cr*sp*cy - sr*cp*sy;
    q[3] = cr*cp*sy + sr*sp*cy;
}


