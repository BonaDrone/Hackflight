/*
   imu.hpp: Altitude estimation via accelerometer Z-axis integration


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

#pragma once

#include <math.h>
#include "filter.hpp"

namespace hf {

    class IMU {

        private:

            // This variables will store the latest available readings
            // from the gyro and the IMU. Since this class is Hardware
            // independent this values will have to be supplied
            float accel[3];
            float gyro[3];

        public:

          // Update last known acceleration values and time of the last measure
          void updateAcceleration(float _accel[3])
          {
              memcpy(accel, _accel, 3*sizeof(float));
          }

          // Update last known gyro values and time of the last measure
          void updateGyro(float _gyro[3])
          {
              memcpy(gyro, _gyro, 3*sizeof(float));
          }

    }; // class IMU

} // namespace hf
