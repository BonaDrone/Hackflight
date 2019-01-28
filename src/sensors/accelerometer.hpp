/*
   accelerometer.hpp : Support for accelerometer

   Hackflight requires your Board implementation to provide the
   quaternion directly, but access to accelerometer could be useful
   for other kinds of sensor fusion (altitude hold).

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
              H[15] =  0;
              H[30] =  0;
              // 2 column
              H[1] =  0;
              H[16] =  0;
              H[31] =  0;
              // 3 column
              H[2] =  0;
              H[17] =  0;
              H[32] =  0;
              // 4 column
              H[3] =  0;
              H[18] =  0;
              H[33] =  0;
              // 5 column
              H[4] =  0;
              H[19] =  0;
              H[34] =  0;
              // 6 column
              H[5] =  0;
              H[20] =  0;
              H[35] =  0;
              // 7 column
              H[6] =                                  0;
              H[21] =  x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9];
              H[36] =            - 2*x[6]*x[7] - 2*x[8]*x[9];
              // 8 column
              H[7] =  - x[6]*x[6] + x[7]*x[7] + x[8]*x[8] - x[9]*x[9];
              H[22] =                                    0;
              H[37] =                2*x[7]*x[9] - 2*x[6]*x[8];
              // 9 column
              H[8] =  2*x[6]*x[7] + 2*x[8]*x[9];
              H[23] =  2*x[6]*x[8] - 2*x[7]*x[9];
              H[38] =                      0;
              // 10 column
              H[9] =  0;
              H[24] =  0;
              H[39] =  0;
              // 11 column
              H[10] =  0;
              H[25] =  0;
              H[40] =  0;
              // 12 column
              H[11] =  0;
              H[26] =  0;
              H[41] =  0;
              // 13 column
              H[12] =  0;
              H[27] =  0;
              H[42] =  0;
              // 14 column
              H[13] =  0;
              H[28] =  0;
              H[43] =  0;
              // 15 column
              H[14] =  0;
              H[29] =  0;
              H[44] =  0;
            }

            virtual void getInnovation(float * z, float * x) override
            {
                float tmp1[3];
                float tmp2[3];
                // We might have to normalize these two vectors (y and h)
                // Predicted Observations
                _predictedObservation[0] = ((2*x[6]*x[8] - 2*x[7]*x[9])*-9.80665);
                _predictedObservation[1] = ((-2*x[6]*x[7] - 2*x[8]*x[9])*-9.80665);
                _predictedObservation[2] = ((-x[6]*x[6] + x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) * -9.80665);
                                             
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
