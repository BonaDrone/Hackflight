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
            
            virtual bool getJacobianObservation(float * H, float * x) override
            {
              
              // Serial.println("Accel Correct");
              
              // 1 column
              H[0] =  0;
              H[9] =  0;
              H[18] =  0;
              // 2 column
              H[1] =  0;
              H[10] =  0;
              H[19] =  0;
              // 3 column
              H[2] =  0;
              H[11] =  0;
              H[20] =  0;
              // 4 column
              H[3] =  0;
              H[12] =  0;
              H[21] =  0;
              // 5 column
              H[4] =  0;
              H[13] =  0;
              H[22] =  0;
              // 6 column
              H[5] =  0;
              H[14] =  0;
              H[23] =  0;
              // 7 column
              H[6] =                                  0;
              H[15] =  x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9];
              H[24] =            - 2*x[6]*x[7] - 2*x[8]*x[9];
              // 8 column
              H[7] =  - x[6]*x[6] + x[7]*x[7] + x[8]*x[8] - x[9]*x[9];
              H[16] =                                    0;
              H[25] =                2*x[7]*x[9] - 2*x[6]*x[8];
              // 9 column
              H[8] =  2*x[6]*x[7] + 2*x[8]*x[9];
              H[17] =  2*x[6]*x[8] - 2*x[7]*x[9];
              H[26] =                      0;
              
              return true;
            }

            virtual bool getInnovation(float * z, float * x) override
            {
                float tmp1[3];
                float tmp2[3];
                // We might have to normalize these two vectors (y and h)
                // Predicted Observations
                _predictedObservation[0] = ((2*x[6]*x[8] - 2*x[7]*x[9]) * -9.80665);
                _predictedObservation[1] = ((-2*x[6]*x[7] - 2*x[8]*x[9]) * -9.80665);
                _predictedObservation[2] = ((-x[6]*x[6] + x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) * -9.80665);
                                             
                norvec(_predictedObservation, tmp1, 3);
                norvec(_accels, tmp2, 3);
                // innovation = measured - predicted
                z[0] = tmp2[0] - tmp1[0];
                z[1] = tmp2[1] - tmp1[1];
                z[2] = tmp2[2] - tmp1[2];
                
                return true;
            }
            
            virtual void getCovarianceCorrection(float * R) override
            {
                // Approximate the process noise using a small constant
                // R[0] = 0.70f;
                // R[4] = 0.70f;
                // R[8] = 0.70f;
                R[0] = 2.00f;
                R[4] = 2.00f;
                R[8] = 2.00f;
            }            

        protected:
            
            virtual bool shouldUpdateESKF(float time, state_t & state) override
            {
                static float _time;

                if (time - _time > UPDATE_PERIOD) {
                    board->getAccelerometer(_accels);
                    _time = time;
                    return true; 
                }
                return false;
            }

        private:

            static constexpr float UPDATE_HZ = 200.0; // XXX should be using interrupt!

            static constexpr float UPDATE_PERIOD = 1.0/UPDATE_HZ;

            float _accels[3];
            float _predictedObservation[3];

    };  // class Accelerometer

} // namespace
