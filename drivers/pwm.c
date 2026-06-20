#include "pwm.h"

#define SERVO_HOLD_US       1740
#define SERVO_RELEASE_US    1200

/*
PA1 -> TIM2_CH2
*/

PWM_HandleTypeDef ServoPWM=
{
    TIM2
};


void PWM_Init(
                PWM_HandleTypeDef *pwm,
                uint32_t frequency
             )
{

    if(pwm->TIMx==TIM2)
    {
        /*===================
          rcc
        ===================*/

        RCC->APB2ENR |=
            RCC_APB2ENR_IOPAEN;

        RCC->APB1ENR |=
            RCC_APB1ENR_TIM2EN;


        /*===================
          PA1

          Alternate Function
          Push Pull
          50MHz
        ===================*/

        GPIOA->CRL &= ~(0xF<<4);
        GPIOA->CRL |=  (0xB<<4);


        /*===================

          APB1=36MHz

          TIM2=72MHz

          PSC=71

          72MHz/(71+1)

          =1MHz

        ===================*/

        pwm->TIMx->PSC=71;


        /*
          period:

          1MHz/50Hz

          =20000

        */

        pwm->TIMx->ARR=
                1000000/frequency;


        /*===================
          PWM MODE 1
          CH2
        ===================*/

        pwm->TIMx->CCMR1 |=
            TIM_CCMR1_OC2M_1 |
            TIM_CCMR1_OC2M_2;

        pwm->TIMx->CCMR1 |=
            TIM_CCMR1_OC2PE;


        pwm->TIMx->CCER |=
            TIM_CCER_CC2E;


        /* servo center */

        pwm->TIMx->CCR2=1500;


        pwm->TIMx->CR1 |=
            TIM_CR1_ARPE;

        pwm->TIMx->EGR |=
            TIM_EGR_UG;

        pwm->TIMx->CR1 |=
            TIM_CR1_CEN;
    }
}



void PWM_SetUs(
                PWM_HandleTypeDef *pwm,
                uint16_t us
              )
{
    pwm->TIMx->CCR2=us;
}

void PWM_InitServo(void)
{
    PWM_Init(&ServoPWM, 50);
    Servo_Release();
}

void Servo_Hold(void)
{
    PWM_SetUs(&ServoPWM, SERVO_HOLD_US);
}

void Servo_Release(void)
{
    PWM_SetUs(&ServoPWM, SERVO_RELEASE_US);
}
