
#ifndef _IMU_INS_H_
#define _IMU_INS_H_

#define GRAVITY		9.80665f	// m/s^2

#define DECL 		12.39f
#define INCL 		71.0f
#define TG_DECL 	0.2196813f



//������
extern SemaphoreHandle_t xSD_FLAGS_collect_Semaphore;
extern SemaphoreHandle_t xSD_UKF_collect_Semaphore;
extern SemaphoreHandle_t xSD_PID_collect_Semaphore;
//�������

//������ ��� UKF
//������� ��� ����� � ������� ���������
extern SemaphoreHandle_t xUKF_Semaphore;
SemaphoreHandle_t  xMPU_UKF_Mutex, xUKF_PID_Mutex;

extern TaskHandle_t ukf_handle;

extern float axf, ayf, azf, gxf, gyf, gzf;//���������� ������ � �������� � g � ���/���
extern float temp_gyro;//����������� �������
extern uint8_t flag_end_of_gyro_calib;//���� ����� ����������
extern uint32_t cnt_cycle_ins, cnt_ticks_ins;
extern uint32_t cnt_cycle_ukf, cnt_ticks_ukf;//�������� �����

extern float mq1, mq2, mq3, mq4, mq5, mq6, mq7, mq8, mq9;//������� ��������
extern float ref_alt_hold;
extern float alt_offset;
extern int16_t ax, ay, az, gx, gy, gz;//����� ����� � 16 ��������
extern uint8_t flag_altWithoutGPS;

extern float g_mix_z;
extern float x_ref, y_ref;
extern float DeltaTimeUKF;

//������ ��������� ��������� ������ � �������
void prvIMU_INS(void *pvParameters);
void prvIMU_UKF(void *pvParameters);
void runInit(void);
#endif /* _IMU_INS_H_ */
