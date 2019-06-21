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

            virtual void integrateNominalState(float * fx, float * x, float * q, double dt) override
            {
                // 1 column
                fx[0] = x[0] + dt*x[3];
                // 2 column
                fx[1] =  x[1] + dt*x[4];
                // 3 column
                fx[2] =  x[2] + dt*x[5];
                // 4 column
                fx[3] =  x[3] - dt*((- _accels[0])*(q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]) - (- _accels[1])*(2*q[0]*q[3] - 2*q[1]*q[2]) + (- _accels[2])*(2*q[0]*q[2] + 2*q[1]*q[3]));
                // 5 column
                fx[4] =  x[4] - dt*((- _accels[1])*(q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3]) + (- _accels[0])*(2*q[0]*q[3] + 2*q[1]*q[2]) - (- _accels[2])*(2*q[0]*q[1] - 2*q[2]*q[3]));
                // 6 column
                fx[5] =  x[5] - dt*(9.80665 + (- _accels[2])*(q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]) - (- _accels[0])*(2*q[0]*q[2] - 2*q[1]*q[3]) + (- _accels[1])*(2*q[0]*q[1] + 2*q[2]*q[3]));
            }
            
            virtual void getJacobianErrors(float * Fdx, float * x, float * q, double dt) override
            {
                // 1 column
                Fdx[0] =  1;
                Fdx[6] =  0;
                Fdx[12] =  0;
                Fdx[18] =  0;
                Fdx[24] =  0;
                Fdx[30] =  0;

                // 2 column
                Fdx[1] =  0;
                Fdx[7] =  1;
                Fdx[13] =  0;
                Fdx[19] =  0;
                Fdx[25] =  0;
                Fdx[31] =  0;

                // 3 column
                Fdx[2] =  0;
                Fdx[8] =  0;
                Fdx[14] =  1;
                Fdx[20] =  0;
                Fdx[26] =  0;
                Fdx[32] =  0;

                // 4 column
                Fdx[3] =  dt;
                Fdx[9] =   0;
                Fdx[15] =   0;
                Fdx[21] =   1;
                Fdx[27] =   0;
                Fdx[33] =   0;

                // 5 column
                Fdx[4] =   0;
                Fdx[10] =  dt;
                Fdx[16] =   0;
                Fdx[22] =   0;
                Fdx[28] =   1;
                Fdx[34] =   0;

                // 6 column
                Fdx[5] =   0;
                Fdx[11] =   0;
                Fdx[17] =  dt;
                Fdx[23] =   0;
                Fdx[29] =   0;
                Fdx[35] =   1;

            }
            
            virtual void getCovarianceEstimation(float * Q) override
            {                
                // 1 column
                Q[0] =  0;
                Q[6] =  0;
                Q[12] =  0;
                Q[18] =  0;
                Q[24] =  0;
                Q[30] =  0;

                // 2 column
                Q[1] =  0;
                Q[7] =  0;
                Q[13] =  0;
                Q[19] =  0;
                Q[25] =  0;
                Q[31] =  0;

                // 3 column
                Q[2] =  0;
                Q[8] =  0;
                Q[14] =  0;
                Q[20] =  0;
                Q[26] =  0;
                Q[32] =  0;

                // 4 column
                Q[3] =  0;
                Q[9] =   0;
                Q[15] =   0;
                Q[21] =   0.08;
                Q[27] =   0;
                Q[33] =   0;

                // 5 column
                Q[4] =   0;
                Q[10] =  0;
                Q[16] =   0;
                Q[22] =   0;
                Q[28] =   0.08;
                Q[34] =   0;

                // 6 column
                Q[5] =   0;
                Q[11] =   0;
                Q[17] =  0;
                Q[23] =   0;
                Q[29] =   0;
                Q[35] =   0.08;
            }
            
            void setSensorData(float accels[3])
            {                
                _accels[0] = accels[0];
                _accels[1] = accels[1];
                _accels[2] = accels[2];
            }

        protected:

            virtual void modifyState(eskf_state_t & state, float time) override
            {
                (void)time;
            }

            virtual bool shouldUpdateESKF(float time, state_t & state) override
            {
                (void)time;

                // board->getIMU(_rates, _accels);

                // state.UAVState->angularVelocities[0] = _rates[0];
                // state.UAVState->angularVelocities[1] = _rates[1];
                // state.UAVState->angularVelocities[2] = _rates[2];                

                return true;
            }

        private:

            float _rates[3];
            float _accels[3];

    };  // class IMU

} // namespace
