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

            const ACCEL_LPF_CUTOFF = 5.0f;

            float fc_accel;

            bool ready;

            uint32_t accelTimeSum;
            int accelSumCount;

            float accelSumZ;
            float accelZoffset;

            void reset(void)
            {
              accelSumZ = 0;
              accelSumCount = 0;
              accelTimeSum = 0;
            }

        public:

          void init(void)
          {
              accelZoffset = 0;
              // calculate RC time constant used in the accelZ lpf
              fc_accel = 0.5f / (M_PI * ACCEL_LPF_CUTOFF);
              reset();
              ready = false;
          }

    } // class IMU

} // namespace hf
