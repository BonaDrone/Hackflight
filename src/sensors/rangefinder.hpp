/*
   rangefinder.hpp : Support for rangefinder sensors (sonar, time-of-flight)

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

#pragma once

#include <cmath>
#include <math.h>

#include "debug.hpp"
#include "peripheral.hpp"
#include "filters.hpp"

namespace hf {

    class Rangefinder : public PeripheralSensor {

        private:

            static constexpr float UPDATE_HZ = 50.0; // XXX should be using interrupt!

            static constexpr float UPDATE_PERIOD = 1.0/UPDATE_HZ;

            float _distance;
            
            // Range finder calibration parameters
            float rx = 0;
            float ry = 0;
            float rz = 0;
            
        public:

            Rangefinder(void) : PeripheralSensor(false, true)
            {
            }

            virtual void getJacobianObservation(float * H, float * x) override
            {
              // 1 column
              H[0] =  1/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]);
              // 2 column
              H[1] =  0;
              // 3 column
              H[2] =  (x[2]*((2*x[2]*ry - 2*x[3]*rz + 2*x[5]*rx)/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) + (2*x[3]*(x[0] + rz*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - rx*(2*x[2]*x[4] - 2*x[3]*x[5]) + ry*(2*x[2]*x[3] + 2*x[4]*x[5])))/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])))/2 - (x[3]*((2*x[2]*rz + 2*x[3]*ry - 2*x[4]*rx)/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - (2*x[2]*(x[0] + rz*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - rx*(2*x[2]*x[4] - 2*x[3]*x[5]) + ry*(2*x[2]*x[3] + 2*x[4]*x[5])))/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])))/2 - (x[5]*((2*x[2]*rx + 2*x[4]*rz - 2*x[5]*ry)/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - (2*x[4]*(x[0] + rz*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - rx*(2*x[2]*x[4] - 2*x[3]*x[5]) + ry*(2*x[2]*x[3] + 2*x[4]*x[5])))/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])))/2 - (x[4]*((2*x[3]*rx + 2*x[4]*ry + 2*x[5]*rz)/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - (2*x[5]*(x[0] + rz*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - rx*(2*x[2]*x[4] - 2*x[3]*x[5]) + ry*(2*x[2]*x[3] + 2*x[4]*x[5])))/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])))/2;
              // 4 column
              H[3] =  (x[3]*((2*x[3]*rx + 2*x[4]*ry + 2*x[5]*rz)/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - (2*x[5]*(x[0] + rz*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - rx*(2*x[2]*x[4] - 2*x[3]*x[5]) + ry*(2*x[2]*x[3] + 2*x[4]*x[5])))/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])))/2 - (x[2]*((2*x[2]*rx + 2*x[4]*rz - 2*x[5]*ry)/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - (2*x[4]*(x[0] + rz*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - rx*(2*x[2]*x[4] - 2*x[3]*x[5]) + ry*(2*x[2]*x[3] + 2*x[4]*x[5])))/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])))/2 - (x[5]*((2*x[2]*ry - 2*x[3]*rz + 2*x[5]*rx)/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) + (2*x[3]*(x[0] + rz*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - rx*(2*x[2]*x[4] - 2*x[3]*x[5]) + ry*(2*x[2]*x[3] + 2*x[4]*x[5])))/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])))/2 - (x[4]*((2*x[2]*rz + 2*x[3]*ry - 2*x[4]*rx)/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - (2*x[2]*(x[0] + rz*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - rx*(2*x[2]*x[4] - 2*x[3]*x[5]) + ry*(2*x[2]*x[3] + 2*x[4]*x[5])))/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])))/2;
              // 5 column
              H[4] =  (x[4]*((2*x[2]*ry - 2*x[3]*rz + 2*x[5]*rx)/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) + (2*x[3]*(x[0] + rz*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - rx*(2*x[2]*x[4] - 2*x[3]*x[5]) + ry*(2*x[2]*x[3] + 2*x[4]*x[5])))/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])))/2 - (x[5]*((2*x[2]*rz + 2*x[3]*ry - 2*x[4]*rx)/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - (2*x[2]*(x[0] + rz*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - rx*(2*x[2]*x[4] - 2*x[3]*x[5]) + ry*(2*x[2]*x[3] + 2*x[4]*x[5])))/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])))/2 + (x[3]*((2*x[2]*rx + 2*x[4]*rz - 2*x[5]*ry)/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - (2*x[4]*(x[0] + rz*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - rx*(2*x[2]*x[4] - 2*x[3]*x[5]) + ry*(2*x[2]*x[3] + 2*x[4]*x[5])))/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])))/2 + (x[2]*((2*x[3]*rx + 2*x[4]*ry + 2*x[5]*rz)/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - (2*x[5]*(x[0] + rz*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - rx*(2*x[2]*x[4] - 2*x[3]*x[5]) + ry*(2*x[2]*x[3] + 2*x[4]*x[5])))/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5])))/2;
              // 6 column
              H[5] =  0;
              // 7 column
              H[6] =  0;
              // 8 column
              H[7] =  0;
            }

            virtual void getInnovation(float * z, float * x) override
            {
                // innovation = measured - predicted
                // predicted is p_w_r(3)/R*R_r_i(3,3), where R = rotation matrix
                float predicted = (x[0] + rz*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - rx*(2*x[2]*x[4] - 2*x[3]*x[5]) + ry*(2*x[2]*x[3] + 2*x[4]*x[5]))/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]);;
                z[0] = _distance - predicted;
            }
            
            virtual void getCovarianceCorrection(float * R) override
            {
                R[0] = 0.0001f;
            }

            virtual int Zinverse(float * Z, float * invZ) override
            {
                if (Z[0] == 0)
                {
                  return 1;
                }
                invZ[0] = 1.0/Z[0];
                return 0;
            }  

        protected:

            virtual void modifyState(eskf_state_t & state, float time) override
            {
                (void)state;
                (void)time;
            }

            virtual bool ready(float time) override
            {
                float newDistance;

                if (distanceAvailable(newDistance)) {
                        _distance = newDistance;
                        return true;
                }
                return false; 
            }

            virtual bool shouldUpdateESKF(float time) override
            {
                static float _time;

                if (time - _time > UPDATE_PERIOD) {
                    _time = time;
                    return true; 
                }
                return false;
            }

            virtual bool distanceAvailable(float & distance) = 0;

    };  // class Rangefinder

} // namespace
