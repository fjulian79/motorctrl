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

void Motor::initPidSpeed(float kp, float ki, float kd)
{
    PidSpeed.init(kp, ki, kd);
}

void Motor::initPidPos(float kp, float ki, float kd)
{
    PidPos.init(kp, ki, kd);
}

void Motor::encoderIsrCallback(void)
{
    uint32_t now = micros();
    uint32_t dir = digitalRead(Pin.encDir);

    if(dir)
        Encoder.position++;
    else
        Encoder.position--;

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

void Motor::speed(float speed)
{
    PidSpeed.setpoint(speed);
}

void Motor::move(int32_t pos, bool rel)
{

}

void Motor::resetPosition(void)
{
    Encoder.position = 0;
}

void Motor::stop(void)
{
    PidSpeed.enable(false);
    PidPos.enable(false);
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
    return Encoder.position;
}

float Motor::getSpeed(void)
{
    return Encoder.speed;
}


float filter_butterworth(float input) 
{
    const float b[2] = { 0.16020035, 0.16020035 };          // 30Hz - 200
    const float a[2] = { 1.00000000, -0.67959930 };
    //const float b[2] = { 0.13672874, 0.13672874 };          // 25Hz - 200 
    //const float a[2] = { 1.00000000, -0.72654253 };
    //const float b[2] = { 0.11216024, 0.11216024 };          // 20Hz - 200
    //const float a[2] = { 1.00000000, -0.77567951 };
    static float prev_input = 0.0;
    static float prev_output = 0.0;

    float output = b[0] * input + b[1] * prev_input - a[1] * prev_output;
    prev_input = input;
    prev_output = output;

    return output;
}

float filter_bessel(float input) 
{
    const float b[3] = {0.06745527, 0.13491055, 0.06745527};
    const float a[3] = {1.0, -1.1429805, 0.4128016};
    static float lastInput[2] = {0.0, 0.0};
    static float lastOutput[2] = {0.0, 0.0};

    float output = b[0] * input + b[1] * lastInput[0] + b[2] * lastInput[1] 
            - a[1] * lastOutput[0] - a[2] * lastOutput[1];

    lastInput[1] = lastInput[0];
    lastInput[0] = input;
    lastOutput[1] = lastOutput[0];
    lastOutput[0] = output;

    return output;
}

void Motor::loop(bool print)
{
    static uint32_t t_last = 0;

    /**
     * ATTENTION: The Order of those two lines matter!! 
     * Interrupts are vever disabled, so if the micos() call takes place before 
     * evaluating the Encoder.t_lastIRQ value, a interrupt may occure inbetween.
     * In this case the Encoder.t_lastIRQ may be bigger then the value of t_now.
     * As result the caluclulation lead to something near UINT32_MX and this 
     * would be handled as timeout and the speed will be set to zero.
     */
    uint32_t t_lastIRQ = Encoder.t_lastIRQ;
    uint32_t t_now = micros();
    
    if (t_now - t_lastIRQ > 60000)
    {
        Encoder.speed = 0;
    }

    if (t_now - t_last > 2000)
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
        //float v2 = Encoder.speed;
        //v3 = 0.854*v3 + 0.0728*v2 + 0.0728*v2_last;
        //v2_last = v2;

        float speed = Encoder.speed;
        //Encoder.speed_filtered = filter_butterworth(speed);
        Encoder.speed_filtered = speed;

        if(PidSpeed.isEnabled())
        {
            float val = PidSpeed.calc(Encoder.speed_filtered);
            pwm(val);
        }

        if(print)
        {
            Serial.print(PidSpeed.getSetpoint());
            Serial.print("; ");
            Serial.print(speed);
            Serial.print("; ");
            Serial.print(Encoder.speed_filtered);
            Serial.print("\n");
        }

        t_last = t_now;

        digitalWrite(DEBUG_A, LOW);
    }
}

template <typename T>
PidControl<T>::PidControl()
{
    init(0, 0, 0);
}

template <typename T>
void PidControl<T>::init(float kp, float ki, float kd)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
    reset();
}

template <typename T>
float PidControl<T>::getKp(void)
{
    return Kp;
}

template <typename T>
float PidControl<T>::getKi(void)
{
    return Ki;
}

template <typename T>
float PidControl<T>::getKd(void)
{
    return Kd;
}

template <typename T>
void PidControl<T>::setpoint(T sp, bool enable)
{
    Setpoint = sp;
    Enabled = enable;
}

template <typename T>
T PidControl<T>::getSetpoint(void)
{
    return Setpoint;
}

template <typename T>
void PidControl<T>::enable(bool state)
{
    Enabled = state;
}

template <typename T>
bool PidControl<T>::isEnabled(void)
{
    return Enabled;
}

template <typename T>
void PidControl<T>::reset()
{
    E_sum = E_last = Setpoint = 0;
    Enabled = false;
}

template <typename T>
float PidControl<T>::calc(T input)
{
    T e;
    float p, i, d, u;

    e = Setpoint - input;
    E_sum += e;
    E_sum = constrain(E_sum, -10000.0, 10000.0);
    p = e * Kp;
    i = E_sum * Ki;
    d = (e - E_last) * Kd;
    E_last = e;
    u = p + i + d;

    //Serial.print(p);
    //Serial.print(" ");
    //Serial.print(i);
    //Serial.print(" ");
    //Serial.print(d);
    //Serial.print("\n");

    return u;
}