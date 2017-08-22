#ifndef _BUZZER_H_
#define _BUZZER_H_





extern volatile uint8_t buzzer_flag;
extern volatile uint32_t counter_buzzer_sys;

void Buzzer_Init(void);
void Buzzer_Function(void);

#endif /* _BUZZER_H_ */
