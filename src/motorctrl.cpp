/*
 * motorctrl, a generic motor control application.
 *
 * Copyright (C) 2021 Julian Friedrich
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>. 
 *
 * You can file issues at https://github.com/fjulian79/motorctrl/issues
 */

#include "motorctrl.hpp"
#include "cmd.hpp"

#define BUZZER_PIN              PB8

#define MOTOR_PWM_HZ            32000

Cli cli;
Buzzer buzzer;
Param<Parameter_t> param;
LiPo lipo(&param.data.Battery);
HardwareTimer timer(TIM3);
DcMot motor[MOTOR_COUNT];

cliCmd_t cmdTable[] =
{
    {"help",    cmd_help},
    {"ver",     cmd_ver},
    {"param",   cmd_param},
    {"bat",     cmd_bat},
    {"cal",     cmd_cal},
    {"temp",    cmd_temp},
    {"mot",     cmd_mot},
    {"stop",    cmd_stop},
    {"beep",    cmd_beep},
    {"reset",   cmd_reset},
};

void setup()
{
    DcMotParams_t motorParams;
    uint32_t a[4] = {PA11, PC12, PB13, PB14};
    uint32_t b[4] = {PA8,  PA12, PB12, PB15};
    uint32_t p[4] = {PC8,  PC9,  PC6,  PC7};

    Serial.begin(115200);
    while (!Serial);   
    Serial.println();

    cmd_ver(0, 0);

    if(param.read() != true)
    {
        param.data.Battery = LIPO_PARAMS;
        param.write();
        Serial.printf("Parameter reset.\n");
    }
    else
    {
        Serial.printf("Parameter loaded.\n");
    }

    pinMode(LED_BUILTIN, OUTPUT);

    buzzer.begin(BUZZER_PIN);

    timer.setOverflow(MOTOR_PWM_HZ, HERTZ_FORMAT);
    timer.resume();
    motorParams.pTim = &timer;
    for (uint8_t n = 0; n < MOTOR_COUNT; n++)
    {
        motorParams.A = a[n];
        motorParams.B = b[n];
        motorParams.Pwm = p[n];
        motor[n].init(&motorParams);
    }

    cli.begin(cmdTable);
}

void loop()
{
    static uint32_t ledTick = 0;
    uint32_t tick = millis();

    if(lipo.getNumCells() != 0 && lipo.lowVoltage())
    {
        buzzer.pulse(250, 250, 1);
    }

    if (tick - ledTick >= 250)
    {
        ledTick = tick;
        digitalToggle(LED_BUILTIN);
    }
    
    lipo.task(tick);
    buzzer.task(tick);
    cli.read();
}