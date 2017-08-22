
#ifndef _PWM_IN_OUT_H_
#define _PWM_IN_OUT_H_


//***************************************************************************************************//
#define	PPM_R_IRQ	 					TIM4_IRQn
#define	PPM_R		 					TIM4

#define	PPM_R_Port_Group				GPIOD
#define	PPM_R_RCC_PERIPH_PIN()			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE)
#define	PPM_R_MULTI_Pin					GPIO_Pin_12|GPIO_Pin_13
#define	PPM_R_Pin_Source1				GPIO_PinSource12
#define	PPM_R_Pin_Source2				GPIO_PinSource13
#define	PPM_R_GPIO_AF					GPIO_AF_TIM4

#define	PPM_R_RCC_PERIPH()				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
//***************************************************************************************************//
#define	PWM_M_IRQ	 					TIM2_IRQn
#define	PWM_M		 					TIM2

#define	PWM_M_Port_Group1				GPIOA
#define	PWM_M_RCC_PERIPH_PIN1()			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)
#define	PWM_M_MULTI_Pin1				GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3

#define	PWM_M_Port_Group2				GPIOC
#define	PWM_M_RCC_PERIPH_PIN2()			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)
#define	PWM_M_MULTI_Pin2				GPIO_Pin_6


#define	PWM_M_Pin_Source1				GPIO_PinSource0
#define	PWM_M_Pin_Source2				GPIO_PinSource1
#define	PWM_M_Pin_Source3				GPIO_PinSource2
#define	PWM_M_Pin_Source4				GPIO_PinSource3
#define	PWM_M_GPIO_AF					GPIO_AF_TIM2

#define	PWM_M_RCC_PERIPH()				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE)
//***************************************************************************************************//


#define MED_PPM       		 1500
#define DEAD_ZONE_PPM       	 5
#define MIN_PPM       		 990
#define MAX_PPM        		 2014
#define YAW_RATE       		 0.004f
#define MAX_PITCH_ROLL		 M_PI/3.0f
#define PITCH_ROLL_K       	 MAX_PITCH_ROLL/((MAX_PPM - MIN_PPM)/2.0)
#define THROTLE_K      		 1.0/(MAX_PPM-10 - MIN_PPM)

#define TIME_FOR_ARMING 		20*1

#define DEAD_ZONE_ALT_STICK		0.08f
extern volatile uint8_t flagSavePoint;
extern volatile float throtle_ppm, yaw_rate_ppm, pitch_ppm, roll_ppm;
extern volatile float k_vel_auto_ppm;
extern uint16_t option_channel_1, option_channel_2;
extern uint16_t M1_lev_front, M2_prav_back, M3_prav_front, M4_lev_back;
extern uint16_t ppm1_buf[9]; 		/* буфер для значений каналов ППМ */
 //Счетчики циклов
extern uint32_t cnt_cycle_ppm, cnt_ticks_ppm;
extern uint32_t cnt_cycle_pwm, cnt_ticks_pwm;
//импорт
extern SemaphoreHandle_t xSD_PPM_collect_Semaphore;
extern SemaphoreHandle_t xSD_PWM_collect_Semaphore;
//extern data_ukf DataUKF;
//extern SemaphoreHandle_t xSD_FLAGS_collect_Semaphore;
//extern uint8_t flag_start_home;
//extern uint8_t flag_start_home_baro;


void Init_PWM_Reciever1(uint16_t Presc);
void Init_PWM_Motor(uint16_t Period1, uint16_t Dp1);
void prvArming(void *pvParameters);


#endif /* _PWM_IN_OUT_H_ */
