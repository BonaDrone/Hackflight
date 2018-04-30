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

            // This variables will store the latest available readings
            // from the gyro and the IMU. Since this class is Hardware
            // independent this values will have to be supplied
            float accel[3];
            float gyro[3];

            float fc_accel;

            bool ready;

            uint32_t accelTimeSum;
            int accelSumCount;

            float accelZ_tmp;
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

          // Estimate vertical velocity as:
          // average acceleration since last query * g * time since last query
          float getVerticalVelocity(void)
          {
              accelZ_tmp = accelSumZ / accelSumCount;
              // Skip startup transient
              float velocityFromAcc = ready ? accelZ_tmp * 9.80665e-4 * accelTimeSum : 0;
              reset();
              ready = true;
              return velocityFromAcc;
          }

          float getVerticalAcceleration(void)
          {
              return accelZ_tmp;
          }

          // Update last known acceleration values
          void updateAcceleration(float _accel[3], uint32_t currentTime)
          {
            memcpy(accel, _accel, 3*sizeof(float));
            update(currentTime);
          }

          // Update last known gyro values
          void updateGyro(float _gyro[3], uint32_t currentTime)
          {
            memcpy(gyro, _gyro, 3*sizeof(float));
            update(currentTime);
          }

    } // class IMU

} // namespace hf
