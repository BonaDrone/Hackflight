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
            
            // Accelerometer will be used to correct
            Accelerometer() : SurfaceMountSensor(false, true)
            {
                memset(_accels, 0, 3*sizeof(float));
            }
            
            virtual void getJacobianObservation(float * H, float * x) override
            {
                // 1 column
                H[0] =  0;
                H[8] =  0;
                H[16] =  0;
                // 2 column
                H[1] =  0;
                H[9] =  0;
                H[17] =  0;
                // 3 column
                H[2] =                                  0;
                H[10] =  x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5];
                H[18] =            - 2*x[2]*x[3] - 2*x[4]*x[5];
                // 4 column
                H[3] =  - x[2]*x[2] + x[3]*x[3] + x[4]*x[4] - x[5]*x[5];
                H[11] =                                    0;
                H[19] =                2*x[3]*x[5] - 2*x[2]*x[4];
                // 5 column
                H[4] =  2*x[2]*x[3] + 2*x[4]*x[5];
                H[12] =  2*x[2]*x[4] - 2*x[3]*x[5];
                H[20] =                      0;
                // 6 column
                H[5] =  0;
                H[13] =  0;
                H[21] =  0;
                // 7 column
                H[6] =  0;
                H[14] =  0;
                H[22] =  0;
                // 8 column
                H[7] =  0;
                H[15] =  0;
                H[23] =  0;
            }

            virtual void getInnovation(float * z, float * x) override
            {
                float tmp1[3];
                float tmp2[3];
                // We might have to normalize these two vectors (y and h)
                // Predicted Observations
                _predictedObservation[0] = ((2*x[2]*x[4] - 
                                            2*x[3]*x[5])*-9.80665);
                _predictedObservation[1] = ((-2*x[2]*x[3] - 
                                             2*x[4]*x[5])*-9.80665);
                _predictedObservation[2] = ((-x[2]*x[2] + 
                                             x[3]*x[3] +
                                             x[4]*x[4] -
                                             x[5]*x[5])*-9.80665);
                                             
                norvec(_predictedObservation, tmp1, 3);
                norvec(_accels, tmp2, 3);
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

        protected:

            virtual void modifyState(eskf_state_t & state, float time) override
            {
                // Here is where you'd do sensor fusion
                (void)state;
                (void)time;
            }

            virtual bool ready(float time) override
            {
                (void)time;
                return board->getAccelerometer(_accels);
            }
            
            virtual bool shouldUpdateESKF(float time) override
            {
                return true;
            }

        private:

            float _accels[3];
            float _predictedObservation[3];

    };  // class Accelerometer

} // namespace
