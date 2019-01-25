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

            static constexpr float UPDATE_HZ = 50.0; // XXX should be using interrupt!

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

            virtual void getJacobianObservation(float * H, float * x) override
            {
              // auxiliary variables to avoid duplicate calculations
              float aux1 = x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9];
              float aux2 = (2*x[6]*_rx + 2*x[8]*_rz - 2*x[9]*_ry)/aux1;
              float aux3 = 2*x[6]*x[7] + 2*x[8]*x[9];
              float aux4 = 2*x[6]*x[8] - 2*x[7]*x[9];
              float aux5 = (2*x[6]*_ry - 2*x[7]*_rz + 2*x[9]*_rx)/aux1;
              float aux6 = (2*x[7]*_rx + 2*x[8]*_ry + 2*x[9]*_rz)/aux1;
              float aux7 = (2*x[6]*_rz + 2*x[7]*_ry - 2*x[8]*_rx)/aux1;
              float aux8 = x[0] + _rz*aux1 - _rx*aux4 + _ry*aux3;
              float aux9 = aux5 + (2*x[7]*aux8)/(aux1*aux1);
              float aux10 = aux7 - (2*x[6]*aux8)/(aux1*aux1);
              float aux11 = aux2 - (2*x[8]*aux8)/(aux1*aux1);
              float aux12 = aux6 - (2*x[9]*aux8)/(aux1*aux1);
              // 1 column
              H[0] =  1/aux1;
              // 2 column
              H[1] =  0;
              // 3 column
              H[2] =  0;
              // 4 column
              H[3] =  0;
              // 5 column
              H[4] =  0;
              // 6 column
              H[5] =  0;
              // 7 column
              H[6] =  (x[6]*aux9)/2 - (x[7]*aux10)/2 - (x[9]*aux11)/2 - (x[8]*aux12)/2;
              // 8 column
              H[7] =  (x[7]*aux12)/2 - (x[6]*aux11)/2 - (x[9]*aux9)/2 - (x[8]*aux10)/2;
              // 9 column
              H[8] =  (x[8]*aux9)/2 - (x[9]*aux10)/2 + (x[7]*aux11)/2 + (x[6]*aux12)/2;
              // 10 column
              H[9] =  0;
              // 11 column
              H[10] =  0;
              // 12 column
              H[11] =  0;
              // 13 column
              H[12] =  0;
              // 14 column
              H[13] =  0;
              // 15 column
              H[14] =  0;
            }

            virtual void getInnovation(float * z, float * x) override
            {
                // innovation = measured - predicted
                // predicted is p_w_r(3)/R*R_r_i(3,3), where R = rotation matrix
                float predicted = (x[0] + _rz*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]) - _rx*(2*x[6]*x[8] - 2*x[7]*x[9]) + _ry*(2*x[6]*x[7] + 2*x[8]*x[9]))/(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]);
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
            
            void setCalibration(float rx, float ry, float rz)
            {
              _rx = rx;
              _ry = ry;
              _rz = rz;
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
