/*
    barometer.hpp: Altitude estimation using barometer

    Adapted from

    https://github.com/multiwii/baseflight/blob/master/src/sensors.c

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
    along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "filter.hpp"

namespace hf {

    class Barometer {

      private:

          // TODO -> why these specific values?
          const float NOISE_LPF             = 0.5f;
          const float VELOCITY_BOUND        = 300.0f;
          const float VELOCITY_DEADBAND     = 10.f;
          static const uint8_t HISTORY_SIZE = 48;

          float   alt;
          float   groundAltitude;
          float   previousAltitude;
          float   pressuseSum;
          float   history[HISTORY_SIZE];
          uint8_t historyIdx;

          // Pressure in millibars to altitude in centimeters. We assume
          // millibars are the units of the pressure readings from the sensor
          float millibarsToCentimeters(float pa)
          {
              // see: https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
              return (1.0f - powf(pa / 1013.25f, 0.190295f)) * 4433000.0f;
          }

      public:

          void init(void)
          {
              alt = 0;
              groundAltitude = 0;
              previousAltitude = 0;
              pressuseSum = 0;
              historyIdx = 0;

              for (uint8_t k = 0; k < HISTORY_SIZE; ++k) {
                  history[k] = 0;
              }
          }

          // static variables are initialized one time and then stick
          // around maintaining its value until the end of the program.
          // Furthermote, they are initialized to 0 and shared amongst
          // instances of the class.

          void calibrate(void)
          {
              static float groundPressure;
              // TODO -> why divide by 8
              groundPressure -= groundPressure / 8;
              groundPressure += pressuseSum / (HISTORY_SIZE - 1);
              groundAltitude = millibarsToCentimeters(groundPressure/8);
          }

          void update(float pressure)
          {
              // cycle the index throught the history array
              history[historyIdx] = pressure;
              uint8_t nextIndex = (historyIdx + 1) % HISTORY_SIZE;
              pressuseSum += pressure;
              // Remove next reading from sum so that pressureSum is kept in sync
              pressuseSum -= history[nextIndex];
              historyIdx = nextIndex;
          }

          float getAltitude(void)
          {
              float alt_tmp = millibarsToCentimeters(pressuseSum/(HISTORY_SIZE-1)) - groundAltitude;
              alt = Filter::complementary(alt, alt_tmp, NOISE_LPF);
              return alt;
          }

          // Get vertical velocity in centimeters / second
          float getVerticalVelocity(uint32_t currentTime)
          {
              static float previousAltitude;
              static uint32_t previousTime;
              float vel = (alt - previousAltitude) * 1000000.0f / (currentTime-previousTime);
              previousAltitude = alt;
              previousTime = currentTime;
              vel = Filter::constrainAbs(vel, VELOCITY_BOUND);
              return Filter::deadband(vel, VELOCITY_DEADBAND);
          }
    }; // class Barometer

} // namespace hp
