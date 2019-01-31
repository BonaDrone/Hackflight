/*
   opticalflow.hpp : Support for PMW3901 optical-flow sensor

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

#include <PMW3901.h>

#include "debug.hpp"
#include "peripheral.hpp"

namespace hf {

    class OpticalFlow : public PeripheralSensor {

        private:

            static constexpr float UPDATE_HZ = 100.0; // XXX should be using interrupt!
            static constexpr float UPDATE_PERIOD = 1.0/UPDATE_HZ;

            // Use digital pin 12 for chip select and SPI1 port for comms
            PMW3901 _flowSensor = PMW3901(12, &SPI1);
            // flow measures
            float _deltaX = 0;
            float _deltaY = 0;
            // focal distances XXX should be estimated
            float _fx = 30;
            float _fy = 30;
            // angular velocities
            float _rates[3];

        protected:
            
            virtual bool shouldUpdateESKF(float time) override
            {
                static float _time;

                if (time - _time > UPDATE_PERIOD) {
                    _time = time;
                    int16_t deltaX=0, deltaY=0;
                    _flowSensor.readMotionCount(&deltaX, &deltaY);
                    _deltaX = (float)deltaX;
                    _deltaY = (float)deltaY;

                    return true; 
                }
                return false;
            }

        public:

            OpticalFlow(void) : PeripheralSensor(false, true) {}

            void begin(void)
            {
                if (!_flowSensor.begin()) {
                    while (true) {
                        Serial.println("Initialization of the flow sensor failed");
                        delay(500);
                    }
                }

            }
            
            virtual void getJacobianObservation(float * H, float * x) override
            {
                // 1 column
                H[0] =  0;
                H[15] =  0;
                // 2 column
                H[1] =  0;
                H[16] =  0;
                // 3 column
                H[2] =   (_fx*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9]))*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]*x[2];
                H[17] =  -(_fy*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9]))*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]*x[2];
                // 4 column
                H[3] =          (_fx*(2*x[6]*x[9] - 2*x[7]*x[8])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2];
                H[18] =  (_fy*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9])*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]))/x[2];
                // 5 column
                H[4] =  -(_fx*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9])*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]))/x[2];
                H[19] =           (_fy*(2*x[6]*x[9] + 2*x[7]*x[8])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2];
                // 6 column
                H[5] =  -(_fx*(2*x[6]*x[7] + 2*x[8]*x[9])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2];
                H[20] =  -(_fy*(2*x[6]*x[8] - 2*x[7]*x[9])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2];
                // 7 column
                H[6] =    (x[6]*((2*_fx*x[7]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/x[2] - (_fx*(2*x[6]*x[5] - 2*x[7]*x[4] + 2*x[8]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2 + (x[7]*((2*_fx*x[6]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/x[2] + (_fx*(2*x[6]*x[4] + 2*x[7]*x[5] - 2*x[9]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2 + (x[8]*((2*_fx*x[9]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/x[2] - (_fx*(2*x[6]*x[3] - 2*x[8]*x[5] + 2*x[9]*x[4])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2 + (x[9]*((2*_fx*x[8]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/x[2] - (_fx*(2*x[7]*x[3] + 2*x[8]*x[4] + 2*x[9]*x[5])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2;
                H[21] =  - (x[7]*((2*_fy*x[6]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/x[2] + (_fy*(2*x[6]*x[3] - 2*x[8]*x[5] + 2*x[9]*x[4])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2 - (x[6]*((2*_fy*x[7]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/x[2] - (_fy*(2*x[7]*x[3] + 2*x[8]*x[4] + 2*x[9]*x[5])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2 - (x[9]*((2*_fy*x[8]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/x[2] + (_fy*(2*x[6]*x[5] - 2*x[7]*x[4] + 2*x[8]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2 - (x[8]*((2*_fy*x[9]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/x[2] + (_fy*(2*x[6]*x[4] + 2*x[7]*x[5] - 2*x[9]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2;
                // 8 column
                H[7] =  (x[8]*((2*_fx*x[6]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/x[2] + (_fx*(2*x[6]*x[4] + 2*x[7]*x[5] - 2*x[9]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2 - (x[9]*((2*_fx*x[7]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/x[2] - (_fx*(2*x[6]*x[5] - 2*x[7]*x[4] + 2*x[8]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2 + (x[6]*((2*_fx*x[8]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/x[2] - (_fx*(2*x[7]*x[3] + 2*x[8]*x[4] + 2*x[9]*x[5])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2 - (x[7]*((2*_fx*x[9]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/x[2] - (_fx*(2*x[6]*x[3] - 2*x[8]*x[5] + 2*x[9]*x[4])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2;
                H[22] =  (x[7]*((2*_fy*x[9]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/x[2] + (_fy*(2*x[6]*x[4] + 2*x[7]*x[5] - 2*x[9]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2 - (x[8]*((2*_fy*x[6]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/x[2] + (_fy*(2*x[6]*x[3] - 2*x[8]*x[5] + 2*x[9]*x[4])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2 - (x[6]*((2*_fy*x[8]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/x[2] + (_fy*(2*x[6]*x[5] - 2*x[7]*x[4] + 2*x[8]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2 + (x[9]*((2*_fy*x[7]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/x[2] - (_fy*(2*x[7]*x[3] + 2*x[8]*x[4] + 2*x[9]*x[5])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2;
                // 9 column
                H[8] =  (x[8]*((2*_fx*x[7]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/x[2] - (_fx*(2*x[6]*x[5] - 2*x[7]*x[4] + 2*x[8]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2 + (x[9]*((2*_fx*x[6]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/x[2] + (_fx*(2*x[6]*x[4] + 2*x[7]*x[5] - 2*x[9]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2 - (x[6]*((2*_fx*x[9]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/x[2] - (_fx*(2*x[6]*x[3] - 2*x[8]*x[5] + 2*x[9]*x[4])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2 - (x[7]*((2*_fx*x[8]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/x[2] - (_fx*(2*x[7]*x[3] + 2*x[8]*x[4] + 2*x[9]*x[5])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2;
                H[23] =  (x[7]*((2*_fy*x[8]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/x[2] + (_fy*(2*x[6]*x[5] - 2*x[7]*x[4] + 2*x[8]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2 + (x[6]*((2*_fy*x[9]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/x[2] + (_fy*(2*x[6]*x[4] + 2*x[7]*x[5] - 2*x[9]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2 - (x[9]*((2*_fy*x[6]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/x[2] + (_fy*(2*x[6]*x[3] - 2*x[8]*x[5] + 2*x[9]*x[4])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2 - (x[8]*((2*_fy*x[7]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/x[2] - (_fy*(2*x[7]*x[3] + 2*x[8]*x[4] + 2*x[9]*x[5])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2]))/2;
                // 10 column
                H[9] =  0;
                H[24] =  0;
                // 11 column
                H[10] =  0;
                H[25] =  0;
                // 12 column
                H[11] =  0;
                H[26] =  0;
                // 13 column
                H[12] =  _fx;
                H[27] =   0;
                // 14 column
                H[13] =   0;
                H[28] =  _fy;
                // 15 column
                H[14] =  0;
                H[16] =  0;
            }

            virtual void getInnovation(float * z, float * x) override
            {
                float _predictedObservation[2];
                // predicted values
                _predictedObservation[0] = _fx*(x[13] - _rates[0]) - (_fx*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9]))*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2];
                _predictedObservation[1] =  _fy*(x[14] - _rates[1]) + (_fy*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9]))*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/x[2];
                // innovation = measured - predicted
                z[0] = _deltaX - _predictedObservation[0];
                z[1] = _deltaY - _predictedObservation[1];
            }
            
            virtual void getCovarianceCorrection(float * R) override
            {
              R[0] = 10.0;
              R[3] = 10.0;
            }
            
            virtual void getMeasures(eskf_state_t & state) override
            {
              _rates[0] = state.angularVelocities[0];
              _rates[1] = state.angularVelocities[1];
              _rates[2] = state.angularVelocities[2];
            }
            
            virtual bool Zinverse(float * Z, float * invZ) override
            {
              float tmp[2];
              if (cholsl(Z, invZ, tmp, 2))
              { 
                return 1;
              }
              return 0;
            }

    };  // class OpticalFlow 

} // namespace hf
