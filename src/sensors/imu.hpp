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
                fx[0] = x[0] + dt*x[3];
                // 2 column
                fx[1] =  x[1] + dt*x[4];
                // 3 column
                fx[2] =  x[2] + dt*x[5];
                // 4 column
                fx[3] =  x[3] - dt*((- _accels[0])*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) - (- _accels[1])*(2*x[6]*x[9] - 2*x[7]*x[8]) + (- _accels[2])*(2*x[6]*x[8] + 2*x[7]*x[9]));
                // 5 column
                fx[4] =  x[4] - dt*((- _accels[1])*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) + (- _accels[0])*(2*x[6]*x[9] + 2*x[7]*x[8]) - (- _accels[2])*(2*x[6]*x[7] - 2*x[8]*x[9]));
                // 6 column
                fx[5] =  x[5] - dt*(9.80665 + (- _accels[2])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]) - (- _accels[0])*(2*x[6]*x[8] - 2*x[7]*x[9]) + (- _accels[1])*(2*x[6]*x[7] + 2*x[8]*x[9]));
                // 7 column
                fx[6] =  x[6] + (dt*x[7]*(- _rates[0]))/2 + (dt*x[8]*(- _rates[1]))/2 + (dt*x[9]*(- _rates[2]))/2;
                // 8 column
                fx[7] =  x[7] - (dt*x[6]*(- _rates[0]))/2 - (dt*x[8]*(- _rates[2]))/2 + (dt*x[9]*(- _rates[1]))/2;
                // 9 column
                fx[8] =  x[8] - (dt*x[6]*(- _rates[1]))/2 + (dt*x[7]*(- _rates[2]))/2 - (dt*x[9]*(- _rates[0]))/2;
                // 10 column
                fx[9] =  x[9] - (dt*x[6]*(- _rates[2]))/2 - (dt*x[7]*(- _rates[1]))/2 + (dt*x[8]*(- _rates[0]))/2;
            }
            
            virtual void getJacobianErrors(float * Fdx, float * x, double dt) override
            {
                // 1 column
                Fdx[0] =  1;
                Fdx[9] =  0;
                Fdx[18] =  0;
                Fdx[27] =  0;
                Fdx[36] =  0;
                Fdx[45] =  0;
                Fdx[54] =  0;
                Fdx[63] =  0;
                Fdx[72] =  0;

                // 2 column
                Fdx[1] =  0;
                Fdx[10] =  1;
                Fdx[19] =  0;
                Fdx[28] =  0;
                Fdx[37] =  0;
                Fdx[46] =  0;
                Fdx[55] =  0;
                Fdx[64] =  0;
                Fdx[73] =  0;

                // 3 column
                Fdx[2] =  0;
                Fdx[11] =  0;
                Fdx[20] =  1;
                Fdx[29] =  0;
                Fdx[38] =  0;
                Fdx[47] =  0;
                Fdx[56] =  0;
                Fdx[65] =  0;
                Fdx[74] =  0;

                // 4 column
                Fdx[3] =  dt;
                Fdx[12] =   0;
                Fdx[21] =   0;
                Fdx[30] =   1;
                Fdx[39] =   0;
                Fdx[48] =   0;
                Fdx[57] =   0;
                Fdx[66] =   0;
                Fdx[75] =   0;

                // 5 column
                Fdx[4] =   0;
                Fdx[13] =  dt;
                Fdx[22] =   0;
                Fdx[31] =   0;
                Fdx[40] =   1;
                Fdx[49] =   0;
                Fdx[58] =   0;
                Fdx[67] =   0;
                Fdx[76] =   0;

                // 6 column
                Fdx[5] =   0;
                Fdx[14] =   0;
                Fdx[23] =  dt;
                Fdx[32] =   0;
                Fdx[41] =   0;
                Fdx[50] =   1;
                Fdx[59] =   0;
                Fdx[68] =   0;
                Fdx[77] =   0;

                // 7 column
                Fdx[6] =                                                                                0;
                Fdx[15] =                                                                                0;
                Fdx[24] =                                                                                0;
                Fdx[33] =          -dt*((- _accels[1])*(2*x[6]*x[8] + 2*x[7]*x[9]) + (- _accels[2])*(2*x[6]*x[9] - 2*x[7]*x[8]));
                Fdx[42] =   dt*((- _accels[2])*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) + (- _accels[1])*(2*x[6]*x[7] - 2*x[8]*x[9]));
                Fdx[51] =  -dt*((- _accels[1])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]) - (- _accels[2])*(2*x[6]*x[7] + 2*x[8]*x[9]));
                Fdx[60] =                                                                                1;
                Fdx[69] =                                                                   dt*(- _rates[2]);
                Fdx[78] =                                                                  -dt*(- _rates[1]);

              // 8 column
                Fdx[7] =                                                                                0;
                Fdx[16] =                                                                                0;
                Fdx[25] =                                                                                0;
                Fdx[34] =  -dt*((- _accels[2])*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) - (- _accels[0])*(2*x[6]*x[8] + 2*x[7]*x[9]));
                Fdx[43] =          -dt*((- _accels[0])*(2*x[6]*x[7] - 2*x[8]*x[9]) + (- _accels[2])*(2*x[6]*x[9] + 2*x[7]*x[8]));
                Fdx[52] =   dt*((- _accels[0])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]) + (- _accels[2])*(2*x[6]*x[8] - 2*x[7]*x[9]));
                Fdx[61] =                                                                  -dt*(- _rates[2]);
                Fdx[70] =                                                                                1;
                Fdx[79] =                                                                   dt*(- _rates[0]);

                // 9 column
                Fdx[8] =                                                                                0;
                Fdx[17] =                                                                                0;
                Fdx[26] =                                                                                0;
                Fdx[35] =   dt*((- _accels[1])*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + (- _accels[0])*(2*x[6]*x[9] - 2*x[7]*x[8]));
                Fdx[44] =  -dt*((- _accels[0])*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - (- _accels[1])*(2*x[6]*x[9] + 2*x[7]*x[8]));
                Fdx[52] =          -dt*((- _accels[0])*(2*x[6]*x[7] + 2*x[8]*x[9]) + (- _accels[1])*(2*x[6]*x[8] - 2*x[7]*x[9]));
                Fdx[62] =                                                                   dt*(- _rates[1]);
                Fdx[71] =                                                                  -dt*(- _rates[0]);
                Fdx[80] =                                                                                1;

            }
            
            virtual void getCovarianceEstimation(float * Q) override
            {                
                // 1 column
                Q[0] =  0;
                Q[9] =  0;
                Q[18] =  0;
                Q[27] =  0;
                Q[36] =  0;
                Q[45] =  0;
                Q[54] =  0;
                Q[63] =  0;
                Q[72] =  0;

                // 2 column
                Q[1] =  0;
                Q[10] =  0;
                Q[19] =  0;
                Q[28] =  0;
                Q[37] =  0;
                Q[46] =  0;
                Q[55] =  0;
                Q[64] =  0;
                Q[73] =  0;

                // 3 column
                Q[2] =  0;
                Q[11] =  0;
                Q[20] =  0;
                Q[29] =  0;
                Q[38] =  0;
                Q[47] =  0;
                Q[56] =  0;
                Q[65] =  0;
                Q[74] =  0;

                // 4 column
                Q[3] =  0;
                Q[12] =   0;
                Q[21] =   0;
                Q[30] =   20.0;
                Q[39] =   0;
                Q[48] =   0;
                Q[57] =   0;
                Q[66] =   0;
                Q[75] =   0;

                // 5 column
                Q[4] =   0;
                Q[13] =  0;
                Q[22] =   0;
                Q[31] =   0;
                Q[40] =   20.0;
                Q[49] =   0;
                Q[58] =   0;
                Q[67] =   0;
                Q[76] =   0;

                // 6 column
                Q[5] =   0;
                Q[14] =   0;
                Q[23] =  0;
                Q[32] =   0;
                Q[41] =   0;
                Q[50] =   1.0;
                Q[59] =   0;
                Q[68] =   0;
                Q[77] =   0;

                // 7 column
                Q[6] = 0;
                Q[15] = 0;
                Q[24] = 0;
                Q[33] = 0;
                Q[42] =  0;
                Q[51] =  0;
                Q[60] =  0.001;
                Q[69] =  0;
                Q[78] =  0;

              // 8 column
                Q[7] =   0;
                Q[16] =  0;
                Q[25] =  0;
                Q[34] =  0;
                Q[43] =  0;
                Q[52] =  0;
                Q[61] =  0;
                Q[70] =  0.001;
                Q[79] =  0;

                // 9 column
                Q[8] =   0;
                Q[17] =  0;
                Q[26] =  0;
                Q[35] =  0;
                Q[44] =  0;
                Q[52] =  0;
                Q[62] =  0;
                Q[71] =  0;
                Q[80] =  0.001;

            }

        protected:

            virtual void modifyState(eskf_state_t & state, float time) override
            {
                (void)time;
            }

            virtual bool shouldUpdateESKF(float time, state_t & state) override
            {
                (void)time;

                board->getIMU(_rates, _accels);
                // Serial.println(_rates[0]);
                state.UAVState->angularVelocities[0] = _rates[0];
                state.UAVState->angularVelocities[1] = _rates[1];
                state.UAVState->angularVelocities[2] = _rates[2];                

                return true;
            }

        private:

            float _rates[3];
            float _accels[3];

    };  // class IMU

} // namespace
