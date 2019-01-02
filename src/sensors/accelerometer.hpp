/*
   accelerometer.hpp : Support for accelerometer

   Hackflight requires your Board implementation to provide the
   quaternion directly, but access to accelerometer could be useful
   for other kinds of sensor fusion (altitude hold).

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

    class Accelerometer : public SurfaceMountSensor {

        friend class Hackflight;

        public:

            Accelerometer()
            {
                memset(_gs, 0, 3*sizeof(float));
            }

        protected:

            virtual void modifyState(state_t & state, float time) override
            {
                // Here is where you'd do sensor fusion
                (void)state;
                (void)time;
            }

            virtual bool ready(float time) override
            {
                (void)time;
                return board->getAccelerometer(_gs);
            }
            
            virtual void getJacobianObservation(float * H, float * x) override
            {
              // First Column
              H[0]  =  0.0;
              H[3]  =  x[0]*x[0] - x[1]*x[1] - x[2]*x[2] + x[3]*x[3];
              H[6]  = - 2*x[0]*x[1] - 2*x[2]*x[3];
              // Second Column
              H[1]  = -x[0]*x[0] + x[1]*x[1] + x[2]*x[2] - x[3]*x[3];
              H[4]  =  0.0;
              H[7]  = 2*x[1]*x[3] - 2*x[0]*x[2];
              // Third Column
              H[2]  = 2*x[0]*x[1] + 2*x[2]*x[3];
              H[5]  = 2*x[0]*x[2] - 2*x[1]*x[3];
              H[8]  = 0.0;
            }

            virtual void getInnovation(float * z, float * x) override
            {
                float meas[3];
                float tmp1[3];
                float tmp2[3];
                meas[0] = _gs[0]*9.80665;
                meas[1] = _gs[1]*9.80665;
                meas[2] = _gs[2]*9.80665;
                // We might have to normalize these two vectors (y and h)
                // Predicted Observations
                _predictedObservation[0] = ((2*x[0]*x[2] - 
                                            2*x[1]*x[3])*-9.80665);
                _predictedObservation[1] = ((-2*x[0]*x[1] - 
                                             2*x[2]*x[3])*-9.80665);
                _predictedObservation[2] = ((-x[0]*x[0] + 
                                             x[1]*x[1] +
                                             x[2]*x[2] -
                                             x[3]*x[3])*-9.80665);
                                             
                norvec(_predictedObservation, tmp1, 3);
                norvec(meas, tmp2, 3);
                // innovation = measured - predicted
                z[0] = tmp2[0] - tmp1[0];
                z[1] = tmp2[1] - tmp1[1];
                z[2] = tmp2[2] - tmp1[2];
            }
            
            virtual void getCovarianceCorrection(float * R) override
            {
                // Approximate the process noise using a small constant
                R[0] = 1.00f;
                R[4] = 1.00f;
                R[8] = 1.00f;
            }

        private:

            float _gs[3];
            float _predictedObservation[3];

    };  // class Accelerometer

} // namespace
