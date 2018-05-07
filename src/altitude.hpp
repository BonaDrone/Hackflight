/*
    altitude.hpp: Altitude estimation via barometer/accelerometer fusion

    Adapted from

    https://github.com/multiwii/baseflight/blob/master/src/imu.c

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

# pragma once

#include "datatypes.hpp"
#include "sensors/barometer.hpp"

namespace hf {

  class AltitudeEstimator {

    private:
      // sensor abstractions
      Barometer baro = Barometer();

      // estimated altitude
      float estimatedAltitude;

      // State variables
      bool holding;
      float referenceAltitude;
      float initialThrottle; // [0, 1]
      float pid;

    public:

      AltitudeEstimator()
      {

      }

      void handleAuxSwitch(demands_t & demands)
      {
        if (demands.aux > 0) {
          holding = true;
          referenceAltitude = estimatedAltitude;
          // This is the reference throttle to hover
          // at the current altitude
          initialThrottle = demands.throttle;
        }
        else {
          holding = false;
        }
      }

      void modifyDemands(demands_t & demands)
      {
         if (holding) {
           demands.throttle = initialThrottle + pid;
         }
      }

      void updateBaro(float pressure)
      {
          baro.update(pressure);
      }

  }; // class AltitudeEstimator
} // namespace hf
