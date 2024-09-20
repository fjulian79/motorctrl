#ifndef MOTOR_HPP_
#define MOTOR_HPP_

#include <Arduino.h>
#include <stdint.h>

#define DEBUG_A                 PC10
#define DEBUG_B                 PC11

template <typename T>
class PidControl
{
    public:

        PidControl();

        void init(float kp, float ki, float kd);

        float getKp(void);

        float getKi(void);

        float getKd(void);

        void setpoint(T sp, bool enable = true);

        T getSetpoint(void);

        void enable(bool state = true);

        bool isEnabled(void);

        void reset(void);

        float calc(T input);

    private:

        float Kp;
        float Ki;
        float Kd;
        T Setpoint;
        T E_sum;
        T E_last;
        bool Enabled;
};

class Motor
{
    public:

        Motor();

        void init(uint32_t pinmot_a, uint32_t pinmot_b, uint32_t pinmot_pwm, uint32_t pinenc_clk, uint32_t pinenc_dir, uint32_t pwm_hz = 32000);

        void initEncoder(callback_function_t callback);

        void initPidSpeed(float Kp, float Ki, float kd);

        void initPidPos(float Kp, float Ki, float kd);

        void printStatus();

        void encoderIsrCallback(void);

        void pwm(float percent);

        void pwm(int32_t val);

        void speed(float speed);

        void move(int32_t pos, bool rel = false);

        void resetPosition(void);

        void stop(void);

        void ebreak(uint32_t val);

        int32_t getPosition(void);

        float getSpeed();

        void loop(bool debug = false);

    private:

        struct 
        {
            uint32_t motA;
            uint32_t motB;
            uint32_t motPwm;
            uint32_t encClk;
            uint32_t encDir;
        } Pin;

        struct
        {
            volatile int32_t position;
            volatile uint32_t t_lastIRQ;
            volatile uint32_t periode;
            volatile float speed;
            float speed_filtered;
        } Encoder;
    
    PidControl<float> PidSpeed;
    PidControl<int32_t> PidPos; 
};

#endif /* DCMOT_HPP_ */