#include "motor.hpp"

#include <string.h>

#define MOTOR_PWM_BITS      10
#define MOTOR_PWM_MAX       (1<<MOTOR_PWM_BITS)

Motor::Motor()
{
    memset(&Pin, 0, sizeof(Pin));
    memset(&Encoder, 0, sizeof(Encoder));
}

void Motor::init(uint32_t pinmot_a, uint32_t pinmot_b, uint32_t pinmot_pwm, uint32_t pinenc_clk, uint32_t pinenc_dir, uint32_t pwm_hz)
{
    Pin.motA = pinmot_a;
    Pin.motB = pinmot_b;
    Pin.motPwm = pinmot_pwm;
    Pin.encClk = pinenc_clk;    
    Pin.encDir = pinenc_dir;

    pinMode(Pin.motA, OUTPUT);
    pinMode(Pin.motB, OUTPUT);
    pinMode(Pin.motPwm, OUTPUT);

    analogWrite(Pin.motPwm, 0);
    analogWriteFrequency(pwm_hz);
    analogWriteResolution(MOTOR_PWM_BITS);
}

void Motor::initEncoder(callback_function_t callback)
{
    pinMode(Pin.encClk, INPUT);
    pinMode(Pin.encDir, INPUT);
    attachInterrupt(digitalPinToInterrupt(Pin.encClk), callback, RISING);
}

void Motor::encoderIsrCallback(void)
{
    uint32_t now = micros();
    uint32_t dir = digitalRead(Pin.encDir);

    if(dir)
        Encoder.posistion++;
    else
        Encoder.posistion--;

    Encoder.periode = now - Encoder.t_lastIRQ;
    Encoder.t_lastIRQ = now;

    if (dir)
        Encoder.speed = 256081.9462/Encoder.periode;
    else
        Encoder.speed = -256081.9462/Encoder.periode;
}

void Motor::pwm(float percent)
{
    percent = constrain(percent, -100.0, 100.0);
    pwm((int32_t)(MOTOR_PWM_MAX*percent)/100);
}

void Motor::pwm(int32_t val)
{
    uint32_t pwm = constrain(abs(val), 0, MOTOR_PWM_MAX);

    if (val == 0)
    {
        digitalWrite(Pin.motA, LOW);
        digitalWrite(Pin.motB, LOW);
    }
    else if(val < 0)
    {
        val = -val;
        digitalWrite(Pin.motA, HIGH);
        digitalWrite(Pin.motB, LOW);
    } 
    else
    {
        digitalWrite(Pin.motA, LOW);
        digitalWrite(Pin.motB, HIGH);
    }

    analogWrite(Pin.motPwm, pwm);
}

void Motor::stop(void)
{
    pwm((int32_t)0);
}

void Motor::ebreak(uint32_t val)
{
    val = constrain(val, 0, MOTOR_PWM_MAX);

    digitalWrite(Pin.motA, HIGH);
    digitalWrite(Pin.motB, HIGH);
    analogWrite(Pin.motPwm, val);
}

int32_t Motor::getPosition(void)
{
    return Encoder.posistion;
}

float Motor::getSpeed(void)
{
    return Encoder.speed;
}

void Motor::loop(bool print)
{
    static uint32_t t_last = 0;
    uint32_t t_now = micros();

    // Check for stopped motor
    if (t_now - Encoder.t_lastIRQ > 60000)
        Encoder.speed = 0;

    if (t_now - t_last > 1000)
    {
        digitalWrite(DEBUG_A, HIGH);

        //int32_t pos = Enc_posistion;
        //uint32_t deltaMicros = t_now-t_last;
        //float deltaT = ((float) deltaMicros)/1.0e6;
        //int32_t deltaPos = pos - posPrev;
        //v1 = deltaPos/deltaT;
        //v1 = v1*60.0/(21.3*11);
        //posPrev = pos;
        //t_last = t_now;

        //static float v2_last = 0;
        //v3 = (2*v3+v2)/3;
        //v3 = 0.854*v3 + 0.0728*v2 + 0.0728*v2_last;
        //v2_last = v2;

        /* BESSEL */
        // 30 - 1000
        //float b[3] = {0.00761985,  0.0152397 ,  0.00761985};
        //float a[3] = {1.        , -1.69028083,  0.72076023};

        // 25 - 1000
        // float b[3] = {0.06745527, 0.13491055, 0.06745527};  // Numerator coefficients
        // float a[3] = {1.0, -1.1429805, 0.4128016};  // Denominator coefficients
        
        // 15 - 1000
        //float b[3] = {0.0020518,  0.00410359,  0.0020518 };
        //float a[3] = {1.0      , -1.84107589,  0.84928308};

        //static float v_in[2] = {0.0, 0.0};  // vorherige Eingangswerte
        //static float v_out[2] = {0.0, 0.0};  // vorherige Ausgangswerte
        //v3 = b[0] * v2 + b[1] * v_in[0] + b[2] * v_in[1] - a[1] * v_out[0] - a[2] * v_out[1];
        //v_in[1] = v_in[0];
        //v_in[0] = v2;
        //v_out[1] = v_out[0];
        //v_out[0] = v3;

        if(print)
        {
            Serial.print(Encoder.posistion);
            Serial.print("; ");
            Serial.print(Encoder.speed);
            Serial.print("\n");
        }

        digitalWrite(DEBUG_A, LOW);
    }
}
