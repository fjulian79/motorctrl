/*
 * motorctrl, a generic motor control application.
 *
 * Copyright (C) 2024 Julian Friedrich
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

#include <Arduino.h>
#include <cli/cli.hpp>
#include <generic/generic.hpp>
#include <generic/task.hpp>

#include "version/version.h"
#include "motor.hpp"

#include <stdio.h>
#include <stdint.h>

#define BUZZER_PIN              PB8
#define DEBUG_A                 PC10
#define DEBUG_B                 PC11

#define MOTOR_NUM               4

Cli cli;
Task ledTask(250);
Task serialTask(10);
Motor motor[MOTOR_NUM];

CLI_COMMAND(ver)
{
    Serial.printf("\n%s %s, Copyright (C) 2024 Julian Friedrich\n", 
            VERSION_PROJECT, VERSION_GIT_SHORT);
    Serial.printf("Build:    %s, %s\n", __DATE__, __TIME__);
    Serial.printf("Git Repo: %s\n", VERSION_GIT_REMOTE_ORIGIN);
    Serial.printf("Revision: %s\n", VERSION_GIT_LONG);
    Serial.printf("\n");
    Serial.printf("This program comes with ABSOLUTELY NO WARRANTY. This is free software, and you\n");
    Serial.printf("are welcome to redistribute it under certain conditions.\n");
    Serial.printf("See GPL v3 licence at https://www.gnu.org/licenses/ for details.\n\n");

    return 0;
}

CLI_COMMAND(m)
{
    if(argc == 2)
    {
        uint8_t idx = strtoul(argv[0], 0, 0);
        float val = atoff(argv[1]);
        
        Serial.printf("Motor %d set to ", idx);
        Serial.print(val);
        Serial.print("\n");

        motor[idx].set(val);

        return 0;
    }

    return -1;
}

CLI_COMMAND(s)
{
    motor[0].stop();
    motor[1].stop();
    motor[2].stop();
    motor[3].stop();

    return 0;
}

CLI_COMMAND(reset)
{
    Serial.printf("Resetting the CPU ...\n");
    delay(100);
    NVIC_SystemReset();
    
    return 0;
}

CLI_COMMAND(help)
{
    Serial.printf("Supported commands:\n");
    Serial.printf("  ver         Used to print version infos.\n");
    Serial.printf("  reset       Used to reset the CPU.\n");
    Serial.printf("  help        Prints this text.\n");

    return 0;
}

void setup()
{
    uint32_t pinmot_a[MOTOR_NUM] =   {PA11, PC12, PB13, PB14};
    uint32_t pinmot_b[MOTOR_NUM] =   {PA8,  PA12, PB12, PB15};
    uint32_t pinmot_pwm[MOTOR_NUM] = {PC8,  PC9,  PC6,  PC7};

    //WARNING: Only motor 0, 1 have valid encoder pins defined
    uint32_t pinenc_clk[MOTOR_NUM] = {PC14, PC2,  PC2,  PC2}; 
    uint32_t pinenc_dir[MOTOR_NUM] = {PC15, PC3,  PC3,  PC3};

    for (uint8_t i=0; i<MOTOR_NUM; i++)
    {
        motor[i].init(pinmot_a[i], pinmot_b[i], pinmot_pwm[i], pinenc_clk[i], pinenc_dir[i]);
    }
    
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    while (!Serial);   
    Serial.println();
    cmd_ver(0, 0);

    cli.begin();
}

void loop()
{
    uint32_t now = millis();

    if(ledTask.isScheduled(now))
    {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }

    cli.loop();
}
