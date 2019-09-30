/*
	######################################################################################
	######################################################################################
	######################################################################################
	######################################################################################
	##################### EFRISA - EEPIS FLYING ROBOT FOR INDONESIA ######################
	######################################################################################
	######################################################################################
	######################################################################################
	######################################################################################
*/
/* Include core modules */
#include "stm32f4xx.h"
/* Include my libraries here */
#include "defines.h"
#include "tm_stm32f4_usart.h"
#include "stdio.h"
#include <stdlib.h>
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_pwmin.h"
#include "tm_stm32f4_timer_properties.h"
#include "tm_stm32f4_pwm.h"
#include "arm_math.h"
#include "tm_stm32f4_usart_dma.h"
#include "tm_stm32f4_bmp180.h"
#include "tm_stm32f4_adc.h"
#include "tm_stm32f4_servo.h"
#include "string.h"
// GLOBAL VARIABLE
char serial2[50]; 
double input1,input2,input3,input4,input5,input6;
int EDFKA, EDFKI;
void ManualMod(), GetPWM();
TM_PWMIN_t PWMIN_TIM1;
TM_PWMIN_t PWMIN_TIM4;
TM_PWMIN_t PWMIN_TIM3;
TM_PWMIN_t PWMIN_TIM5;
TM_PWMIN_t PWMIN_TIM9;
TM_PWMIN_t PWMIN_TIM8;
TM_PWM_TIM_t TIM2_Data;
int setServo= 90;
TM_SERVO_t Servo1, Servo2, Servo3;
double map (double x, double in_min, double in_max, double out_min, double out_max){
	return (x - in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}
int main(void) {
    SystemInit();
    TM_DELAY_Init();

    /* Initialize USART1 at 57600 baud, TX: PA9, RX: PA10 */
		TM_USART_Init(USART1, TM_USART_PinsPack_1, 57600); //1
/* PWM input */    
    TM_PWMIN_InitTimer(TIM1, &PWMIN_TIM1, TM_PWMIN_Channel_1, TM_PWMIN_PinsPack_2, 50 /* 1 */, TIM1_CC_IRQn);//pe9
		TM_PWMIN_InitTimer(TIM4, &PWMIN_TIM4, TM_PWMIN_Channel_1, TM_PWMIN_PinsPack_1, 50 /* 1 */, TIM4_IRQn);//pb6
		TM_PWMIN_InitTimer(TIM3, &PWMIN_TIM3, TM_PWMIN_Channel_1, TM_PWMIN_PinsPack_1, 50 /* 1 */, TIM3_IRQn);//pa6
		TM_PWMIN_InitTimer(TIM5, &PWMIN_TIM5, TM_PWMIN_Channel_1, TM_PWMIN_PinsPack_1, 50 /* 1 */, TIM5_IRQn);//pa0
		TM_PWMIN_InitTimer(TIM9, &PWMIN_TIM9, TM_PWMIN_Channel_1, TM_PWMIN_PinsPack_2, 50 /* 1 */, TIM1_BRK_TIM9_IRQn);//PE5
 		TM_PWMIN_InitTimer(TIM8, &PWMIN_TIM8, TM_PWMIN_Channel_2, TM_PWMIN_PinsPack_1, 50 /* 1 */, TIM8_CC_IRQn );//pc7

/* PWM input */
    TM_PWM_InitTimer(TIM2, &TIM2_Data, 50);
		
		//Channel 3 and PinsPack 2 = PA5 */
		TM_PWM_InitChannel(&TIM2_Data, TM_PWM_Channel_1, TM_PWM_PinsPack_2);
		//Channel 4 and PinsPack 2 = PA15 */
		TM_PWM_InitChannel(&TIM2_Data, TM_PWM_Channel_1, TM_PWM_PinsPack_3);
	
	/* Initialize servo 1, TIM2, Channel 1, Pinspack 2 = PA1 */
    TM_SERVO_Init(&Servo1, TIM2, TM_PWM_Channel_2, TM_PWM_PinsPack_1);
	/* Initialize servo 1, TIM2, Channel 1, Pinspack 2 = PA2 */
    TM_SERVO_Init(&Servo2, TIM2, TM_PWM_Channel_3, TM_PWM_PinsPack_1);
	/* Initialize servo 1, TIM2, Channel 1, Pinspack 2 = PA3 */
    TM_SERVO_Init(&Servo3, TIM2, TM_PWM_Channel_4, TM_PWM_PinsPack_1);
	
	TM_SERVO_SetDegrees(&Servo1, setServo);
	TM_SERVO_SetDegrees(&Servo2, setServo);
	TM_SERVO_SetDegrees(&Servo3, setServo);
    while (1) {
				GetPWM();
				ManualMod();
    }
}
void GetPWM(){
		if (TM_DELAY_Time() >= 50) {
            TM_DELAY_SetTime(0);
            
        /* Get new data for both input signals */
				TM_PWMIN_Get(&PWMIN_TIM1);
				TM_PWMIN_Get(&PWMIN_TIM4);
				TM_PWMIN_Get(&PWMIN_TIM3);
				TM_PWMIN_Get(&PWMIN_TIM5);
				TM_PWMIN_Get(&PWMIN_TIM9);
				TM_PWMIN_Get(&PWMIN_TIM8);
					
					input1=PWMIN_TIM1.DutyCycle;
					input2=PWMIN_TIM4.DutyCycle;
					input3=PWMIN_TIM3.DutyCycle;
					input4=PWMIN_TIM5.DutyCycle; 
					input5=PWMIN_TIM9.DutyCycle;
					input6=PWMIN_TIM8.DutyCycle;
					
					input1 = map(input1,5.12,10,10,170);
					input2 = map(input2,5,9.96,10,170);
					input3 = map(input3,5.49,10,1000,2200);
					input4 = map(input4,5.46,10,10,170);
					if (input1<10){input1=10;}
					if (input1>170){input1=170;}
					if (input2<10){input2=10;}
					if (input2>170){input2=170;}
					if (input4<10){input4=10;}
					if (input4>170){input4=170;}
					
					if (input3<1000){input3=1000;}
					if (input3>2100){input3=2100;}
        }
}

void ManualMod(){
				TM_SERVO_SetDegrees(&Servo1, input1);
				TM_SERVO_SetDegrees(&Servo3, input2);
				TM_SERVO_SetDegrees(&Servo2, input4);
				EDFKA=input3;
				EDFKI=input3;
				TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, EDFKA); //BRUSHLESS
				TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, EDFKI); //BRUSHLESS
				
}

void TIM1_CC_IRQHandler(void) {
	/* Interrupt request, don't forget! */
	TM_PWMIN_InterruptHandler(&PWMIN_TIM1);
}
void TIM4_IRQHandler(void) {
	/* Interrupt request, don't forget! */
	TM_PWMIN_InterruptHandler(&PWMIN_TIM4);
}
void TIM3_IRQHandler(void) {
	/* Interrupt request, don't forget! */
	TM_PWMIN_InterruptHandler(&PWMIN_TIM3);
}
void TIM5_IRQHandler(void) {
	/* Interrupt request, don't forget! */
	TM_PWMIN_InterruptHandler(&PWMIN_TIM5);
}
void TIM1_BRK_TIM9_IRQHandler(void) {
	/* Interrupt request, don't forget! */
	TM_PWMIN_InterruptHandler(&PWMIN_TIM9);
}
void TIM8_CC_IRQHandler(void) {
	/* Interrupt request, don't forget! */
	TM_PWMIN_InterruptHandler(&PWMIN_TIM8);
}