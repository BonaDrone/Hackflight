/*
   imu.hpp : Support for imu (a.k.a. gyroscope + accelerometer) 

   Copyright (c) 2019 BonaDrone (www.bonadrone.com)
   Developed by: Pep Marti-Saumell (jmarti<at>bonadrone.com>) & Juan Gallostra Acin (jgallostra<at>bonadrone.com)

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
#include "sensor.hpp"
#include "surfacemount.hpp"
#include "board.hpp"

namespace hf {

    class IMU : public SurfaceMountSensor {

        friend class Hackflight;

        public:

            // IMU will be used to estimate but not to correct
            IMU(void) : SurfaceMountSensor(true, false)
            {
                memset(_rates, 0, 3*sizeof(float));
                memset(_accels, 0, 3*sizeof(float));
            }

            virtual void integrateNominalState(float * fx, float * x, double dt) override
            {
                // 1 column
                fx[0] = x[0] + dt*x[1];
                // 2 column
                fx[1] =  x[1] - dt*(9.80665 - _accels[2]*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) + _accels[0]*(2*x[2]*x[4] - 2*x[3]*x[5]) - _accels[1]*(2*x[2]*x[3] + 2*x[4]*x[5]));
                // 3 column
                fx[2] =  x[2] + (dt*x[3]*(x[6] - _rates[0]))/2 + (dt*x[4]*(x[7] - _rates[1]))/2 + (dt*x[5]*(x[8] - _rates[2]))/2;
                // 4 column
                fx[3] =  x[3] - (dt*x[2]*(x[6] - _rates[0]))/2 - (dt*x[4]*(x[8] - _rates[2]))/2 + (dt*x[5]*(x[7] - _rates[1]))/2;
                // 5 column
                fx[4] =  x[4] - (dt*x[2]*(x[7] - _rates[1]))/2 + (dt*x[3]*(x[8] - _rates[2]))/2 - (dt*x[5]*(x[6] - _rates[0]))/2;
                // 6 column
                fx[5] =  x[5] - (dt*x[2]*(x[8] - _rates[2]))/2 - (dt*x[3]*(x[7] - _rates[1]))/2 + (dt*x[4]*(x[6] - _rates[0]))/2;
                // 7 column
                fx[6] =  x[6];
                // 8 column
                fx[7] =  x[7];
                // 9 column
                fx[8] =  x[8];
            }
            
            virtual void getJacobianErrors(float * Fdx, float * x, double dt) override
            {
                // 1 column
                Fdx[0] =  1;
                Fdx[8] =  0;
                Fdx[16] =  0;
                Fdx[24] =  0;
                Fdx[32] =  0;
                Fdx[40] =  0;
                Fdx[48] =  0;
                Fdx[56] =  0;
                // 2 column
                Fdx[1] =  dt;
                Fdx[9] =   1;
                Fdx[17] =   0;
                Fdx[25] =   0;
                Fdx[33] =   0;
                Fdx[41] =   0;
                Fdx[49] =   0;
                Fdx[57] =   0;
                // 3 column
                Fdx[2] =                                                               0;
                Fdx[10] =  dt*(_accels[1]*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) - _accels[2]*(2*x[2]*x[3] + 2*x[4]*x[5]));
                Fdx[18] =                                                               1;
                Fdx[26] =                                                  dt*(x[8] - _rates[2]);
                Fdx[34] =                                                 -dt*(x[7] - _rates[1]);
                Fdx[42] =                                                               0;
                Fdx[50] =                                                               0;
                Fdx[58] =                                                               0;
                // 4 column
                Fdx[3] =                                                                0;
                Fdx[11] =  -dt*(_accels[0]*(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]) + _accels[2]*(2*x[2]*x[4] - 2*x[3]*x[5]));
                Fdx[19] =                                                  -dt*(x[8] - _rates[2]);
                Fdx[27] =                                                                1;
                Fdx[35] =                                                   dt*(x[6] - _rates[0]);
                Fdx[43] =                                                                0;
                Fdx[51] =                                                                0;
                Fdx[59] =                                                                0;
                // 5 column
                Fdx[4] =                                                       0;
                Fdx[12] =  dt*(_accels[0]*(2*x[2]*x[3] + 2*x[4]*x[5]) + _accels[1]*(2*x[2]*x[4] - 2*x[3]*x[5]));
                Fdx[20] =                                          dt*(x[7] - _rates[1]);
                Fdx[28] =                                         -dt*(x[6] - _rates[0]);
                Fdx[36] =                                                       1;
                Fdx[44] =                                                       0;
                Fdx[52] =                                                       0;
                Fdx[60] =                                                       0;
                // 6 column
                Fdx[5] =    0;
                Fdx[13] =    0;
                Fdx[21] =  -dt;
                Fdx[29] =    0;
                Fdx[37] =    0;
                Fdx[45] =    1;
                Fdx[53] =    0;
                Fdx[61] =    0;
                // 7 column
                Fdx[6] =    0;
                Fdx[14] =    0;
                Fdx[22] =    0;
                Fdx[30] =  -dt;
                Fdx[38] =    0;
                Fdx[46] =    0;
                Fdx[54] =    1;
                Fdx[62] =    0;
                // 8 column
                Fdx[7] =    0;
                Fdx[15] =    0;
                Fdx[23] =    0;
                Fdx[31] =    0;
                Fdx[39] =  -dt;
                Fdx[47] =    0;
                Fdx[55] =    0;
                Fdx[63] =    1;
            }
            
            virtual void getCovarianceEstimation(float * Q) override
            {
                // 1 column
                Q[0] =  0;
                Q[8] =  0;
                Q[16] =  0;
                Q[24] =  0;
                Q[32] =  0;
                Q[40] =  0;
                Q[48] =  0;
                Q[56] =  0;
                // 2 column
                Q[1] =  0;
                Q[9] =  0.0001;
                Q[17] =  0;
                Q[25] =  0;
                Q[33] =  0;
                Q[41] =  0;
                Q[49] =  0;
                Q[57] =  0;
                // 3 column
                Q[2] = 0;
                Q[10] = 0;
                Q[18] = 0.0001;
                Q[26] = 0;
                Q[34] = 0;
                Q[42] = 0;
                Q[50] = 0;
                Q[58] = 0;
                // 4 column
                Q[3] = 0;
                Q[11] = 0;
                Q[19] = 0;
                Q[27] = 0.0001;
                Q[35] = 0;
                Q[43] = 0;
                Q[51] = 0;
                Q[59] = 0;
                // 5 column
                Q[4] = 0;
                Q[12] = 0;
                Q[20] = 0;
                Q[28] = 0;
                Q[36] = 0.0001;
                Q[44] = 0;
                Q[52] = 0;
                Q[60] = 0;
                // 6 column
                Q[5] =  0;
                Q[13] =  0;
                Q[21] =  0;
                Q[29] =  0;
                Q[37] =  0;
                Q[45] =  0.0001;
                Q[53] =  0;
                Q[61] =  0;
                // 7 column
                Q[6] =  0;
                Q[14] =  0;
                Q[22] =  0;
                Q[30] =  0;
                Q[38] =  0;
                Q[46] =  0;
                Q[54] =  0.0001;
                Q[62] =  0;
                // 8 column
                Q[7] =  0;
                Q[15] =  0;
                Q[23] =  0;
                Q[31] =  0;
                Q[39] =  0;
                Q[47] =  0;
                Q[55] =  0;
                Q[63] =  0.0001;
            }

        protected:

            virtual void modifyState(eskf_state_t & state, float time) override
            {
                (void)time;

                // NB: We negate X, Y rates to simplify PID controller
                state.angularVelocities[0] =  _rates[0];
                state.angularVelocities[1] = -_rates[1];
                state.angularVelocities[2] = -_rates[2];
            }

            virtual bool ready(float time) override
            {
                (void)time;

                bool result = board->getIMU(_rates, _accels);

                return result;
            }
            
            virtual bool shouldUpdateESKF(float time) override
            {
                return true;
            }

        private:

            float _rates[3];
            float _accels[3];

    };  // class IMU

} // namespace
