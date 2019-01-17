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
            
            // Accel. will be used to estimate and to correct
            Accelerometer() : SurfaceMountSensor(true, true)
            {
                memset(_gs, 0, 3*sizeof(float));
                // Define which actions does the ESKF should do with this sensor
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
                return board->getAccelerometer(_gs);
            }
            
            virtual void getJacobianObservation(float * H, float * x) override
            {
              // 1 column
              H[0] =  0;
              H[8] =  0;
              H[16] =  0;
              H[24] =  0;
              H[32] =  0;
              H[40] =  0;
              H[48] =  0;
              H[56] =  0;
              // 2 column
              H[1] =  0;
              H[9] =  0;
              H[17] =  0;
              H[25] =  0;
              H[33] =  0;
              H[41] =  0;
              H[49] =  0;
              H[57] =  0;
              // 3 column
              H[2] =                                  0;
              H[10] =                                  0;
              H[18] =                                  0;
              H[26] =  x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5];
              H[34] =            - 2*x[2]*x[3] - 2*x[4]*x[5];
              H[42] =                                  0;
              H[50] =                                  0;
              H[58] =                                  0;
              // 4 column
              H[3] =                                    0;
              H[11] =                                    0;
              H[19] =  - x[2]*x[2] + x[3]*x[3] + x[4]*x[4] - x[5]*x[5];
              H[27] =                                    0;
              H[35] =                2*x[3]*x[5] - 2*x[2]*x[4];
              H[43] =                                    0;
              H[51] =                                    0;
              H[59] =                                    0;
              // 5 column
              H[4] =                      0;
              H[12] =                      0;
              H[20] =  2*x[2]*x[3] + 2*x[4]*x[5];
              H[28] =  2*x[2]*x[4] - 2*x[3]*x[5];
              H[36] =                      0;
              H[44] =                      0;
              H[52] =                      0;
              H[60] =                      0;
              // 6 column
              H[5] =  0;
              H[13] =  0;
              H[21] =  0;
              H[29] =  0;
              H[37] =  0;
              H[45] =  0;
              H[53] =  0;
              H[61] =  0;
              // 7 column
              H[6] =  0;
              H[14] =  0;
              H[22] =  0;
              H[30] =  0;
              H[38] =  0;
              H[46] =  0;
              H[54] =  0;
              H[62] =  0;
              // 8 column
              H[7] =  0;
              H[15] =  0;
              H[23] =  0;
              H[31] =  0;
              H[39] =  0;
              H[47] =  0;
              H[55] =  0;
              H[63] =  0;
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
