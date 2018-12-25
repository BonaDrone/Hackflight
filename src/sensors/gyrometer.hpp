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

            Gyrometer(void)
            {
                memset(_rates, 0, 3*sizeof(float));
            }


            virtual void getJacobianModel(double * Fx, double dt) override
            {
              // First Column
              Fx[0]  = 1.0;
              Fx[4]  = (double)_rates[0]*dt/2.0;
              Fx[8]  = (double)_rates[1]*dt/2.0;
              Fx[12]  = (double)_rates[2]*dt/2.0;
              // Second Column
              Fx[1]  = -(double)_rates[0]*dt/2.0;
              Fx[5]  =  1.0;
              Fx[9]  = -(double)_rates[2]*dt/2.0;
              Fx[13]  =  (double)_rates[1]*dt/2.0;
              // Third Column
              Fx[2]  = -(double)_rates[1]*dt/2.0;
              Fx[6]  =  (double)_rates[2]*dt/2.0;
              Fx[10] =  1.0;
              Fx[14] = -(double)_rates[0]*dt/2.0;
              // Fourth Column
              Fx[3] = -(double)_rates[2]*dt/2.0;
              Fx[7] = -(double)_rates[1]*dt/2.0;
              Fx[11] =  (double)_rates[0]*dt/2.0;
              Fx[15] =  1.0;
            }
            
            virtual void getJacobianErrors(double * Fdx, double dt) override
            {
              // First Column
              Fdx[0] =  1.0;
              Fdx[3] =  (double)_rates[2]*dt;
              Fdx[6] = -(double)_rates[1]*dt;
              // Second Column
              Fdx[1] = -(double)_rates[2]*dt;
              Fdx[4] =  1.0;
              Fdx[7] =  (double)_rates[0]*dt;
              // Third Column
              Fdx[2] =  (double)_rates[1]*dt;
              Fdx[5] = -(double)_rates[0]*dt;
              Fdx[8] = 1.0;
            }
            
            virtual void getCovarianceEstimation(double * Q) override
            {
              Q[0] = (double)0.0001;
              Q[4] = (double)0.0001;
            }

        protected:

            virtual void modifyState(state_t & state, float time) override
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
