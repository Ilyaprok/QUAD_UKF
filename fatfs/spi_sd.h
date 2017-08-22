//  ******************************************************************************
//  * @file    sd.h 
//  * @author  CIG
//  * @version V1.0.0
//  * @date    
//  * @brief   
//  ******************************************************************************

#ifndef SD_H_
#define SD_H_
// �������  ���  ������  �  SD
#include "stm32f4xx_spi.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "diskio.h"

#include "ff.h"
#include "ffconf.h"

//#include "system.h"
/*
#define GO_IDLE_STATE            0              //����������� ������������
#define SEND_IF_COND             8              //��� SDC V2 - �������� ��������� ����������
#define READ_SINGLE_BLOCK        17             //������ ���������� ����� ������
#define WRITE_SINGLE_BLOCK       24             //������ ���������� ����� ������
#define SD_SEND_OP_COND	         41             //������ �������� �������������
#define APP_CMD			 55                    //������� ������� �� ACMD <n> ������
#define READ_OCR		 58                     //������ �������� OCR
*/
//���������������� ��� ���������� ������� SS
//#define CS_ENABLE         GPIO_ResetBits(GPIOB, GPIO_Pin_12)
//#define CS_DISABLE    	  GPIO_SetBits(GPIOB, GPIO_Pin_12)
/* Port Controls */
#define CS_HIGH()			GPIO_SetBits(GPIOE, GPIO_Pin_15);		/* MMC CS = H */
#define CS_LOW()			GPIO_ResetBits(GPIOE, GPIO_Pin_15);		/* MMC CS = L */


void disk_timerproc(void);

#endif /* SD_H_ */
