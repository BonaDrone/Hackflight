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
          float   pressureSum;
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
              pressureSum = 0;
              historyIdx = 0;

              for (uint8_t k = 0; k < HISTORY_SIZE; ++k) {
                  history[k] = 0;
              }
          }

          // static variables are initialized one time and then stick
          // around maintaining its value until the end of the program.
          // Furthermote, they are initialized to 0 and shared amongst
          // instances of the class.

          // Calibrate the baro by setting the average measured pressure as the
          // ground pressure and the corresponding altitude as the ground altitude.
          void calibrate(void)
          {
              static float groundPressure;
              // groundPressure will stabilize at 8 times the average measured
              // pressure (when groundPressure/8 equals pressureSum/(HISTORY_SIZE-1))
              // This acts as a low pass filter and helps to reduce noise
              groundPressure -= groundPressure / 8;
              groundPressure += pressureSum / (HISTORY_SIZE - 1);
              groundAltitude = millibarsToCentimeters(groundPressure/8);
          }

          // update the barometer pressure reading
          void update(float pressure)
          {
              history[historyIdx] = pressure;
              pressureSum += pressure;
              // cycle the index throught the history array
              uint8_t nextIndex = (historyIdx + 1) % HISTORY_SIZE;
              // Remove next reading from sum so that pressureSum is kept in sync
              pressureSum -= history[nextIndex];
              historyIdx = nextIndex;
          }

          // get aestimated altitude from barometer in centimeters with respect
          // to the altitude estimated as ground altitude
          float getAltitude(void)
          {
              float alt_tmp = millibarsToCentimeters(pressureSum/(HISTORY_SIZE-1)) - groundAltitude;
              // The complementary filter weights the last estimated altitude and
              // the altitude estimated from the barometer preassure readings
              // It seems the objective here is to run some sort of EMA (exponential
              // moving average) filter, which is a sort of simple low pass filter.
              // see: https://www.norwegiancreations.com/2015/10/tutorial-potentiometers-with-arduino-and-filtering/
              alt = Filter::complementary(alt, alt_tmp, NOISE_LPF);
              return alt;
          }

    }; // class Barometer

} // namespace hp
