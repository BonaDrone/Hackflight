/*
   filter.hpp: Static filtering methods

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

#include <cmath>

#include "debug.hpp"

namespace hf {

    class Filter {

        public:

            static float max(float a, float b)
            {
                return a > b ? a : b;
            }

            static float deadband(float value, float deadband)
            {
                if (fabs(value) < deadband) {
                    value = 0;
                } else if (value > 0) {
                    value -= deadband;
                } else if (value < 0) {
                    value += deadband;
                }
                return value;
            }

            static float complementary(float a, float b, float c)
            {
                return a * c + b * (1 - c);
            }

            static float constrainMinMax(float val, float min, float max)
            {
                return (val<min) ? min : ((val>max) ? max : val);
            }

            static float constrainAbs(float val, float max)
            {
                return constrainMinMax(val, -max, +max);
            }

    }; // class Filter

    class KalmanFilter {
      private:

      public:

    }; // Class KalmanFilter

    class ComplementaryFilter {

      private:
        
        // filter gain
        float gain[2];
        // Zero-velocity update
        float accel_threshold;
        static const uint8_t ZUPT_SIZE = 12;
        uint8_t ZUPTIdx;
        float   ZUPT[ZUPT_SIZE];

        float ApplyZUPT(float accel, float vel)
        {
            // first update ZUPT array with latest estimation
            ZUPT[ZUPTIdx] = accel;
            // and move index to next slot
            uint8_t nextIndex = (ZUPTIdx + 1) % ZUPT_SIZE;
            ZUPTIdx = nextIndex;
            // Apply Zero-velocity update
            for (uint8_t k = 0; k < ZUPT_SIZE; ++k) {
                if (ZUPT[k] > accel_threshold) return 0.0;
            }
            return vel;
        }


      public:

        ComplementaryFilter(float sigma_accel, float sigma_baro, float accel_threshold)
        {
            gain[0] = sqrt(2 * sigma_accel / sigma_baro);
            gain[1] = sigma_accel / sigma_baro;
            accel_threshold = accel_threshold;
            // initialize zero-velocity update
            ZUPTIdx = 0;
            for (uint8_t k = 0; k < ZUPT_SIZE; ++k) {
                ZUPT[k] = 0;
            }
        }

        void estimate(float * velocity, float * altitude, float baro_altitude, float accel, float deltat)
        {
            // Apply complementary filter
            *altitude = *altitude + deltat*((*velocity) + (gain[0] + gain[1]*deltat/2)*(baro_altitude-(*altitude)))+
             accel*pow(deltat, 2)/2;
            *velocity = *velocity + deltat*(gain[1]*(baro_altitude-(*altitude)) + accel);
            // Compute zero-velocity update
            *velocity = ApplyZUPT(accel, *velocity);
        }

    }; // Class ComplementaryFilter

} // namespace hf
