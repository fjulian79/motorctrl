#include "motor.hpp"

#include <string.h>

#define MOTOR_PWM_BITS      10
#define MOTOR_PWM_MAX       (1<<MOTOR_PWM_BITS)

Motor::Motor() : 
      PinMot_A(0)
    , PinMot_B(0)
    , PinMot_Pwm(0)
    , PinEnc_Clk(0)
    , PinEnc_Dir(0)
{
    /* Nothing to do */
}


void Motor::init(uint32_t pinmot_a, uint32_t pinmot_b, uint32_t pinmot_pwm, uint32_t pinenc_clk, uint32_t pinenc_dir, uint32_t pwm_hz)
{
    PinMot_A = pinmot_a;
    PinMot_B = pinmot_b;
    PinMot_Pwm = pinmot_pwm;
    PinEnc_Clk = pinenc_clk;    
    PinEnc_Dir = pinenc_dir;

    pinMode(PinMot_A, OUTPUT);
    pinMode(PinMot_B, OUTPUT);
    pinMode(PinMot_Pwm, OUTPUT);

    pinMode(PinEnc_Clk, INPUT);
    pinMode(PinEnc_Dir, INPUT);

    analogWrite(PinMot_Pwm, 0);
    analogWriteFrequency(pwm_hz);
    analogWriteResolution(MOTOR_PWM_BITS);
}

void Motor::set(float percent)
{
    percent = constrain(percent, -100.0, 100.0);
    set((int32_t)(MOTOR_PWM_MAX*percent)/100);
}

void Motor::set(int32_t val)
{
    uint32_t pwm = constrain(abs(val), 0, MOTOR_PWM_MAX);

    if (val == 0)
    {
        digitalWrite(PinMot_A, LOW);
        digitalWrite(PinMot_B, LOW);
    }
    else if(val < 0)
    {
        val = -val;
        digitalWrite(PinMot_A, HIGH);
        digitalWrite(PinMot_B, LOW);
    } 
    else
    {
        digitalWrite(PinMot_A, LOW);
        digitalWrite(PinMot_B, HIGH);
    }

    analogWrite(PinMot_Pwm, pwm);
}

void Motor::stop(void)
{
    set((int32_t)0);
}

void Motor::ebreak(uint32_t val)
{
    val = constrain(val, 0, MOTOR_PWM_MAX);

    digitalWrite(PinMot_A, HIGH);
    digitalWrite(PinMot_B, HIGH);
    analogWrite(PinMot_Pwm, val);
}