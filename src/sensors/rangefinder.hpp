/*
   rangefinder.hpp : Support for rangefinder sensors (sonar, time-of-flight)

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

#include <cmath>
#include <math.h>

#include "debug.hpp"
#include "peripheral.hpp"
#include "filters.hpp"

namespace hf {

    class Rangefinder : public PeripheralSensor {

        private:

            static constexpr float UPDATE_HZ = 75.0; // XXX should be using interrupt!

            static constexpr float UPDATE_PERIOD = 1.0/UPDATE_HZ;

            float _distance;
            
            // Range finder calibration parameters
            float _rx = 0;
            float _ry = 0;
            float _rz = 0;
            
        public:

            Rangefinder(void) : PeripheralSensor(false, true)
            {
            }

            virtual bool getJacobianObservation(float * H, float * x, float * q, uint32_t estimationTime) override
            {
                float aux1 = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
                // 1 column
                H[0] =  0;
                // 2 column
                H[1] =  0;
                // 3 column
                H[2] =  1/aux1;
                // 4 column
                H[3] =  0;
                // 5 column
                H[4] =  0;
                // 6 column
                H[5] =  0;

                return true;
            }

            virtual bool getInnovation(float * z, float * x, float * q, uint32_t estimationTime) override
            {
                // innovation = measured - predicted
                // predicted is p_w_r(3)/R*R_r_i(3,3), where R = rotation matrix
                float predicted = x[2]/( q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);
                z[0] = _distance - predicted; 
                
                return true;               
            }
            
            virtual void getCovarianceCorrection(float * R) override
            {
                R[0] = 0.1f;
            }

            virtual bool Zinverse(float * Z, float * invZ) override
            {
                if (Z[0] == 0)
                {
                    return 1;
                }
                invZ[0] = 1.0/Z[0];
                return 0;
            }
            
            void setCalibration(float rx, float ry, float rz)
            {
                _rx = rx;
                _ry = ry;
                _rz = rz;
            }

        protected:

            virtual bool shouldUpdateESKF(float time, state_t & state) override
            {
                static float _time;
                float newDistance;
                if (time - _time > UPDATE_PERIOD && distanceAvailable(newDistance)) 
                {
                    _time = time;
                    if (_distance < 0.0 || _distance > 4.5)
                    {
                        return false;
                    }
                    _distance = newDistance;
                    return true; 
                }
                return false;
            }

            virtual bool distanceAvailable(float & distance) = 0;

    };  // class Rangefinder

} // namespace
