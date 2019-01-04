/*
   gyrometer.hpp : Support for gyrometer (a.k.a. gyroscope) 

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
#include "sensor.hpp"
#include "surfacemount.hpp"
#include "board.hpp"

namespace hf {

    class Gyrometer : public SurfaceMountSensor {

        friend class Hackflight;

        public:

            // Gyrometer will be used to estimate but not to correct
            Gyrometer(void) : SurfaceMountSensor(true, false)
            {
                memset(_rates, 0, 3*sizeof(float));
            }

            virtual void getJacobianModel(float * Fx, float * x, double dt) override
            {
                // First Column
                Fx[0]  = 1.0;
                Fx[7]  = _rates[0]*dt/2.0;
                Fx[14] = _rates[1]*dt/2.0;
                Fx[21] = _rates[2]*dt/2.0;
                Fx[28] = 0.0;
                Fx[35] = 0.0;
                Fx[42] = 0.0;

                // Second Column
                Fx[1]  = -_rates[0]*dt/2.0;
                Fx[8]  =  1.0;
                Fx[15] = -_rates[2]*dt/2.0;
                Fx[22] = _rates[1]*dt/2.0;
                Fx[29] = 0.0;
                Fx[36] = 0.0;
                Fx[43] = 0.0;
                
                // Third Column
                Fx[2]  = -_rates[1]*dt/2.0;
                Fx[9]  =  _rates[2]*dt/2.0;
                Fx[16] =  1.0;
                Fx[23] = -_rates[0]*dt/2.0;
                Fx[30] = 0.0;
                Fx[37] = 0.0;
                Fx[44] = 0.0;

                // Fourth Column
                Fx[3]  = -_rates[2]*dt/2.0;
                Fx[10] = -_rates[1]*dt/2.0;
                Fx[17] = _rates[0]*dt/2.0;
                Fx[24] = 1.0;
                Fx[31] = 0.0;
                Fx[38] = 0.0;
                Fx[45] = 0.0;
                
                // Fifth Column
                Fx[4]  = -x[1]*dt/-2.0;
                Fx[11] = x[0]*dt/-2.0;
                Fx[18] = x[3]*dt/-2.0;
                Fx[25] = -x[2]*dt/-2.0;
                Fx[32] = 1.0;
                Fx[39] = 0.0;
                Fx[46] = 0.0;

                // Sixth Column
                Fx[5]  = -x[2]*dt/-2.0;
                Fx[12] = -x[3]*dt/-2.0;
                Fx[19] = x[0]*dt/-2.0;
                Fx[26] = x[1]*dt/-2.0;
                Fx[33] = 0.0;
                Fx[40] = 1.0;
                Fx[47] = 0.0;

                // Seventh Column
                Fx[6]  = -x[3]*dt/-2.0;
                Fx[13] = x[2]*dt/-2.0;
                Fx[20] = -x[1]*dt/-2.0;
                Fx[27] = x[0]*dt/-2.0;
                Fx[34] = 0.0;
                Fx[41] = 0.0;
                Fx[48] = 1.0;

            }
            
            virtual void getJacobianErrors(float * Fdx, double dt) override
            {
                // First Column
                Fdx[0]  =  1.0;
                Fdx[6]  =  _rates[2]*dt;
                Fdx[12] = -_rates[1]*dt;
                Fdx[18] = 0.0;
                Fdx[24] = 0.0;
                Fdx[30] = 0.0;
                
                // Second Column
                Fdx[1]  = -_rates[2]*dt;
                Fdx[7]  =  1.0;
                Fdx[13] =  _rates[0]*dt;
                Fdx[19] = 0.0;
                Fdx[25] = 0.0;
                Fdx[31] = 0.0;

                // Third Column
                Fdx[2]  =  _rates[1]*dt;
                Fdx[8]  = -_rates[0]*dt;
                Fdx[14] = 1.0;
                Fdx[20] = 0.0;
                Fdx[26] = 0.0;
                Fdx[32] = 0.0;

                // Fourth Column
                Fdx[3]  = -1.0;
                Fdx[9]  = 0.0;
                Fdx[15] = 0.0;
                Fdx[21] = 0.0;
                Fdx[27] = 0.0;
                Fdx[33] = 0.0;
                
                // Fifth Column
                Fdx[4]  = 0.0;
                Fdx[10] = -1.0;
                Fdx[16] = 0.0;
                Fdx[22] = 0.0;
                Fdx[28] = 0.0;
                Fdx[34] = 0.0;
                
                // Sixth Column
                Fdx[5]  = 0.0;
                Fdx[11] = 0.0;
                Fdx[17] = -1.0;
                Fdx[23] = 0.0;
                Fdx[29] = 0.0;
                Fdx[35] = 0.0;

            }
            
            virtual void getCovarianceEstimation(float * Q) override
            {
                Q[0] = 0.0001;
                Q[4] = 0.0001;
            }

        protected:

            virtual void modifyState(eskf_state_t & state, float time) override
            {
                (void)time;

                // NB: We negate gyro X, Y to simplify PID controller
                state.angularVelocities[0] =  _rates[0];
                state.angularVelocities[1] = -_rates[1];
                state.angularVelocities[2] = -_rates[2];
            }

            virtual bool ready(float time) override
            {
                (void)time;

                bool result = board->getGyrometer(_rates);

                return result;
            }

        private:

            float _rates[3];

    };  // class Gyrometer

} // namespace
