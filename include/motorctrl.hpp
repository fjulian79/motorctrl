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

#ifndef MOTORCTRL_H_
#define MOTORCTRL_H_

#include <Arduino.h>

#include <cli/cli.h>
#include <buzzer/buzzer.hpp>
#include <param/param.hpp>
#include <lipo/lipo.hpp>
#include <dcmot/dcmot.hpp>
#include <generic/generic.hpp>

#include <stdio.h>
#include <stdint.h>

/**
 * @brief Number of supported motors
 */
#define MOTOR_COUNT             4

/**
 * @brief The global parameter structure used in RAM and flash
 */
typedef struct
{
    BatteryParams_t Battery;

}Parameter_t;

/**
 * Global instances of classes 
 */
extern Buzzer buzzer;
extern Param<Parameter_t> param;
extern LiPo lipo;
extern HardwareTimer timer;
extern DcMot motor[MOTOR_COUNT];

#endif /* MOTORCTRL_H_ */