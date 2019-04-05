/*
   datatypes.hpp : Datatype declarations
 
   Copyright (c) 2019 Simon D. Levy, Juan Gallostra Acin, Pep Marti-Saumell

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <cstdint>
#include "filters/eskf_struct.hpp"

typedef struct {

    float throttle;
    float roll;
    float pitch;
    float yaw;
    float setpoint[3];            // X, Y, Z setpoint
} demands_t;

typedef struct {
    hf::eskf_state_t * UAVState;  // Mosquito status 
    bool  armed;
    bool  executingMission;
    float batteryVoltage;         // current Voltage
} state_t;

typedef struct {
    bool _endStage1 = false;
    bool _endStage2 = false;
    uint8_t _rcCalibrationStatus = 0;
    float _center[3] = {0, 0, 0};   // R, P, Y
    float _min[4] = {0, 0, 0, 0};   // T, R, P, Y
    float _max[4] = {0, 0, 0, 0};   // T, R, P, Y
} tx_calibration_t;

