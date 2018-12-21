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
#include "filters/linalg.hpp"

namespace hf {

    class Gyrometer : public SurfaceMountSensor {

        friend class Hackflight;

        public:

            Gyrometer(void)
            {
                memset(_rates, 0, 3*sizeof(float));
            }


            virtual void getJacobianModel(Matrix * Fx, double dt) override
            {
                // First Column
                Fx->set(0, 0, 1.0);
                Fx->set(1, 0, _rates[0]*dt/2.0);
                Fx->set(2, 0, _rates[1]*dt/2.0);
                Fx->set(3, 0, _rates[2]*dt/2.0);
                // Second Column
                Fx->set(0, 1, -_rates[0]*dt/2.0);
                Fx->set(1, 1, 1.0);
                Fx->set(2, 1, -_rates[2]*dt/2.0);
                Fx->set(3, 1, _rates[1]*dt/2.0);
                // Third Column
                Fx->set(0, 2, -_rates[1]*dt/2.0);
                Fx->set(1, 2, _rates[2]*dt/2.0);
                Fx->set(2, 2, 1.0);
                Fx->set(3, 2, -_rates[0]*dt/2.0);
                // Fourth Column
                Fx->set(0, 3, -_rates[2]*dt/2.0);
                Fx->set(1, 3, -_rates[1]*dt/2.0);
                Fx->set(2, 3, _rates[0]*dt/2.0);
                Fx->set(3, 3, 1.0);
            }
            
            virtual void getJacobianErrors(Matrix * Fdx, double dt) override
            {
                // First Column
                Fdx->set(0, 0, 1.0);
                Fdx->set(1, 0, _rates[2]*dt);
                Fdx->set(2, 0, -_rates[1]*dt);
                // Second Column
                Fdx->set(0, 1, -_rates[2]*dt);
                Fdx->set(1, 1, 1.0);
                Fdx->set(2, 1, _rates[0]*dt);
                // Third Column
                Fdx->set(0, 2, _rates[1]*dt);
                Fdx->set(1, 2, -_rates[0]*dt);
                Fdx->set(2, 2, 1.0);
            }
            
            virtual void getCovarianceEstimation(Matrix * Q, uint8_t errorStates) override
            {
                Q->setDimensions(errorStates, errorStates);
                Q->set(0, 0, 0.0001);
                Q->set(1, 1, 0.0001);
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
