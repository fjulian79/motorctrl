#ifndef MOTOR_HPP_
#define MOTOR_HPP_

#include <Arduino.h>
#include <stdint.h>

class Motor
{
    public:

        Motor();

        void init(uint32_t pinmot_a, uint32_t pinmot_b, uint32_t pinmot_pwm, uint32_t pinenc_clk, uint32_t pinenc_dir, uint32_t pwm_hz = 32000);

        void attachEncoderIsr(callback_function_t callback);

        void procEncoder(void);

        void set(float percent);

        void set(int32_t val);

        void stop(void);

        void ebreak(uint32_t val);

        int32_t getPosition(void);

    
    private:

        uint32_t PinMot_A;
        uint32_t PinMot_B;
        uint32_t PinMot_Pwm;
        uint32_t PinEnc_Clk;
        uint32_t PinEnc_Dir;

        volatile int32_t  Enc_Pos;
};

#endif /* DCMOT_HPP_ */