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

#include "git_version.h"
#include "motorctrl.hpp"
#include <stm32yyxx_ll_adc.h>

#ifdef GIT_VERSION_SHORT
#define GIT_VERSION         " " GIT_VERSION_SHORT
#else
#define GIT_VERSION         ""
#endif

int8_t cmd_ver(char *argv[], uint8_t argc)
{
    Serial.printf("\nmotorctrl%s, Copyright (C) 2021 Julian Friedrich\n", 
            GIT_VERSION);
    Serial.printf("Build:    %s, %s\n", __DATE__, __TIME__);
    Serial.printf("Git Repo: %s\n", GIT_REMOTE_ORIGIN_URL);
    Serial.printf("Revision: %s\n", GIT_VERSION_LONG);
    Serial.printf("\n");
    Serial.printf("This program comes with ABSOLUTELY NO WARRANTY. This is free software, and you\n");
    Serial.printf("are welcome to redistribute it under certain conditions.\n");
    Serial.printf("See GPL v3 licence at https://www.gnu.org/licenses/ for details.\n\n");

    return 0;
}

int8_t cmd_help(char *argv[], uint8_t argc)
{
    Serial.printf("Supported commands:\n");
    Serial.printf("  mot n pwm      Sets the pwm of a given motor.\n");
    Serial.printf("                 n    The motor number (0..%d)\n", MOTOR_COUNT-1);
    Serial.printf("                 pwm  The pwm value \n");
    Serial.printf("  stop           Stops all motors at once.\n");
    Serial.printf("  beep           Used to test the buzzer class.\n");
    Serial.printf("  bat            Prints battery infos.\n");
    Serial.printf("  temp           Reads the temperature.\n");
    Serial.printf("  cal cell mV    Calibrate the given cell.\n");
    Serial.printf("  param\n");
    Serial.printf("        save     Writes the data from RAM to EEPROM.\n");
    Serial.printf("        clear    Set the RAM data to zero.\n");
    Serial.printf("        discard  Wipes the EEPROM data.\n");
    Serial.printf("  reset          Resets the CPU.\n");
    Serial.printf("  ver            Prints version infos.\n");
    Serial.printf("  help           Prints this text.\n");

    return 0;
}

int8_t cmd_param(char *argv[], uint8_t argc)
{
    if(argc == 0)
    {
        return -1;
    }

    if(strcmp(argv[0], "clear") == 0)
    {
        param.clear();
        Serial.printf("RAM parameters set to zero.\n");
    }
    else if(strcmp(argv[0], "save") == 0)
    {
        param.write();
        Serial.printf("Parameters saved.\n");
    }
    else if(strcmp(argv[0], "discard") == 0)
    {
        param.discard();
        Serial.printf("Parameters discarded.\n");
    }
    else
    {
        Serial.printf("Invalid parameter.\n");
        return -1;
    }

    return 0;
}

int8_t cmd_temp(char *argv[], uint8_t argc)
{
    uint32_t CALX_TEMP = 25;
    uint32_t V25 = 1430;
    uint32_t AVG_SLOPE = 4300;
    uint32_t temp = 0;
    
    temp = __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(
            AVG_SLOPE, V25, CALX_TEMP, lipo.getVref(), analogRead(ATEMP), 
            LL_ADC_RESOLUTION_12B);

    Serial.printf("CPU: %lu°C\n", temp);

    return 0; 
}

int8_t cmd_bat(char *argv[], uint8_t argc)
{
    int8_t numCells = lipo.getNumCells();
    
    if (numCells < 0)
    {
        Serial.printf("Error: no LiPo pack detected!\n");
        return 0;
    }

    for (uint8_t i = 0; i < numCells; i++)
    {
        uint32_t millis = lipo.getCell(i);
        Serial.printf("%d: %lu,%luV\n", i, millis/1000, millis%1000);
    }
    
    return 0;
}

int8_t cmd_cal(char *argv[], uint8_t argc)
{
    uint8_t cell = 0;
    uint32_t voltage = 0;

    if(argc != 2)
    {
        return -1;    
    }
    
    cell = strtol(argv[0], 0 ,0);
    voltage = strtol(argv[1], 0 ,0);
    lipo.calibrate(cell, voltage);

    return 0;
}

int8_t cmd_mot(char *argv[], uint8_t argc)
{
    uint32_t n = 0;
    int32_t pwm = 0;

    if (argc != 2)
    {
        return -1;        
    }

    n = strtoul(argv[0], 0, 0);
    pwm = strtol(argv[1], 0, 0);

    if(n > 3)
    {
        return -1;
    }

    Serial.printf("Motor %d set to %ld\n", n, pwm);
    motor[n].set(pwm);

    return 0; 
}

int8_t cmd_stop(char *argv[], uint8_t argc)
{
    for (uint8_t n = 0; n < 4; n++)
    {
        motor[n].stop();
    }
    
    return 0;
}

int8_t cmd_beep(char *argv[], uint8_t argc)
{
    if (argc == 1)
    {
        if (strcmp(argv[0], "on") == 0)
            buzzer.enable(true);
        else
            buzzer.enable(false);
    }
    else if (argc == 2)
    {
        uint16_t on = atoi(argv[0]);
        uint16_t off = atoi(argv[1]);

        buzzer.pulse(on, off);
    }
    else if (argc == 3)
    {
        uint16_t on = atoi(argv[0]);
        uint16_t off = atoi(argv[1]);
        uint16_t cnt = atoi(argv[2]);

        buzzer.pulse(on, off, cnt);
    }
    else
    {
        return -1;
    }
    
    return 0;
}

int8_t cmd_reset(char *argv[], uint8_t argc)
{
    Serial.printf("Resetting the CPU...\n");
    delay(100);

    NVIC_SystemReset();

    return 0;
}
