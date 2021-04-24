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

#ifndef CMD_H_
#define CMD_H_

#include <stdio.h>
#include <stdint.h>

/**
 * @brief Prints versions infos.
 * 
 * @param argv unused
 * @param argc unused
 * 
 * @return int8_t zero
 */
int8_t cmd_ver(char *argv[], uint8_t argc);

/**
 * @brief Prints the help text.
 * 
 * @param argv unused
 * @param argc unused
 * @return int8_t zero
 */
int8_t cmd_help(char *argv[], uint8_t argc);

/**
 * @brief Used to work on the parameters in the data flash.
* 
 * argv[0] defines what to do ...
 *      save     Writes the data from RAM to EEPROM.
 *      clear    Set the RAM data to zero.
 *      discard  Wipes the EEPROM data.
 * 
 * @param argv array of arguments
 * @param argc number of arguments
 * @return int8_t 
 */
int8_t cmd_param(char *argv[], uint8_t argc);

/**
 * @brief Prints the current cpu temperature.
 * 
 * @param argv unused
 * @param argc unused
 * @return int8_t zero
 */
int8_t cmd_temp(char *argv[], uint8_t argc);

/**
 * @brief Prints the LIPO cell voltages.
 * 
 * @param argv unused
 * @param argc unused
 * @return int8_t zero
 */
int8_t cmd_bat(char *argv[], uint8_t argc);

/**
 * @brief Used to calibrate the LIPO cell voltage measurement.
 * 
 * argv[0], cell number
 * argv[1], measured cell voltage in mV
 * 
 * @param argv array of arguments
 * @param argc number of arguments
 * @return int8_t zero
 */
int8_t cmd_cal(char *argv[], uint8_t argc);

/**
 * @brief Used to contol the motors
 * 
 * argv[0], motor number
 * argv[1], motor value
 * 
 * @param argv array of arguments
 * @param argc number of arguments
 * @return int8_t zero in case of sucess
 *                -1 in case of invalid arguments.
 */
int8_t cmd_mot(char *argv[], uint8_t argc);

/**
 * @brief Stops all motors
 * 
 * @param argv unused
 * @param argc unused
 * @return int8_t zero
 */
int8_t cmd_stop(char *argv[], uint8_t argc);

/**
 * @brief Controlls the buzzer
 * 
 * argc = 1
 * argv[0] = "on", beeper allways on.
 * 
 * argc = 2
 * argv[0] = on time
 * argv[1] = off time
 * 
 * argc = 3
 * argv[0] = on time
 * argv[1] = off time
 * argv[2] = number of tones
 * 
 * @param argv array of arguments
 * @param argc number of arguments
 * @return int8_t 
 */
int8_t cmd_beep(char *argv[], uint8_t argc);

/**
 * @brief Triggers a CPU reset
 * 
 * @param argv unused
 * @param argc unused
 * @return int8_t zero, but never reached.
 */
int8_t cmd_reset(char *argv[], uint8_t argc);

#endif /* CMD_H_ */