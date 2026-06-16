#include "servo.h"
#include "pwm.h"

#define SERVO_HOLD_US       1740
#define SERVO_RELEASE_US    1200


static void Servo_SetUs(uint16_t us)
{
    PWM_SetUs(&ServoPWM, us);
}

void Servo_Init(void)
{
    PWM_Init(&ServoPWM, 50);
    Servo_Release();
}

void Servo_Lock(void)
{
    Servo_SetUs(SERVO_HOLD_US);
}

void Servo_Release(void)
{
    Servo_SetUs(SERVO_RELEASE_US);
}

void Servo_Set0(void)
{
    Servo_Lock();
}

void Servo_Set90(void)
{
    Servo_Release();
}
