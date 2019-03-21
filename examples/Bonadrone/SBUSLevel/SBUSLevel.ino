/*
   SBUS.ino : Hackflight sketch for Bonadrone flight controller with SBUS receiver

   Additional libraries needed:

       https://github.com/simondlevy/LSM6DSM
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SBUSRX

   Hardware support for Bonadrone flight controller:

       https://github.com/simondlevy/grumpyoldpizza

   Copyright (c) 2018 Simon D. Levy

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

#include <Arduino.h>

#include "hackflight.hpp"
#include "boards/bonadrone.hpp"
#include "receivers/sbus.hpp"
#include "mixers/quadx.hpp"

#include "pidcontrollers/level.hpp"

// Change this as needed
#define SBUS_SERIAL Serial1

static constexpr uint8_t CHANNEL_MAP[6] = {2,0,1,3,5,4};

hf::Hackflight h;

hf::SBUS_Receiver rc = hf::SBUS_Receiver(CHANNEL_MAP, SERIAL_SBUS, &SBUS_SERIAL);

hf::MixerQuadX mixer;

// Rate Constants
const float RATE_ROLL_P = 0.05;
const float RATE_ROLL_I = 0.40;
const float RATE_ROLL_D = 0.0001;
const float RATE_PITCH_P = 0.05;
const float RATE_PITCH_I = 0.55;
const float RATE_PITCH_D = 0.0001;
const float RATE_YAW_P = 0.05;
const float RATE_YAW_I = 0.40;

const float RATE_DEM2RATE = 6.00;

hf::Rate ratePid = hf::Rate(
        RATE_ROLL_P,   // Gyro Roll P
        RATE_ROLL_I,   // Gyro Roll I
        RATE_ROLL_D, // Gyro Roll D
        RATE_PITCH_P,   // Gyro Pitch P
        RATE_PITCH_I,   // Gyro Pitch I
        RATE_PITCH_D, // Gyro Pitch D
        RATE_YAW_P,   // Gyro yaw P
        RATE_YAW_I,   // Gyro yaw I
        RATE_DEM2RATE);  // Demands to rate (6 when using rate)

// Level constants
const float LEVEL_ROLL_P = 1.00;
const float LEVEL_PITCH_P = 1.00;

const float LEVEL_DEM2ANGLE = 45.0;

hf::Level level = hf::Level(LEVEL_ROLL_P,LEVEL_PITCH_P, LEVEL_DEM2ANGLE, RATE_DEM2RATE);  // Pitch Level P

void setup(void)
{
    // begin the serial port for the ESP32
    Serial4.begin(115200);

    // 0 means the controller will always be active, but by changing
    // that number it can be linked to a different aux state
    h.addPidController(&level, 0);

    h.init(new hf::BonadroneBrushed(), &rc, &mixer, &ratePid);
}

void loop(void)
{
    h.update();
}