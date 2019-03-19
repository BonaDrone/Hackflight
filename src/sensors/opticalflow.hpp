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

            static constexpr float UPDATE_HZ = 50.0; // XXX should be using interrupt!
            static constexpr float UPDATE_PERIOD = 1.0/UPDATE_HZ;
            static const uint8_t HISTORY = 20;

            // Use digital pin 12 for chip select and SPI1 port for comms
            PMW3901 _flowSensor = PMW3901(12, &SPI1);
            // flow measures
            hf::LowPassFilter _lpDeltaX = hf::LowPassFilter(HISTORY);
            hf::LowPassFilter _lpDeltaY = hf::LowPassFilter(HISTORY);
            float _filteredDeltaX = 0;
            float _filteredDeltaY = 0;                 
            float _deltaX = 0;
            float _deltaY = 0;
            // Time elapsed between corrections
            float deltat = 0.0;
            // Focal distance 
            float f = 350.0;
            // angular velocities
            float _rates[3];

        protected:
            
            virtual bool shouldUpdateESKF(float time) override
            {
                static float _time;

                if (time - _time > UPDATE_PERIOD) {
                    int16_t deltaX=0, deltaY=0;
                    _flowSensor.readMotionCount(&deltaX, &deltaY);
                    // To match camera frame
                    _filteredDeltaX = _lpDeltaX.update((float)deltaX);
                    _filteredDeltaY = _lpDeltaY.update((float)deltaY);                 
                    deltat = time -_time;
                    _time = time;
                    _deltaX = -(float)_filteredDeltaY / deltat;
                    _deltaY = (float)_filteredDeltaX / deltat;
                    // Serial.print(_deltaX, 8);
                    // Serial.print(",");
                    // Serial.print(_deltaY, 8);
                    // Serial.print(",");
                    return true; 
                }
                return false;
            }

        public:

            OpticalFlow(void) : PeripheralSensor(false, true) {}

            bool begin(void)
            {
                _lpDeltaX.init();
                _lpDeltaY.init();
                bool connected = true;
                if (!_flowSensor.begin()) {
                  connected = false;
                }
                return connected;
            }
            virtual bool getJacobianObservation(float * H, float * x) override
            {
                float z;
                // Saturate z estimation to avoid singularities
                if (x[2] < 0.1)
                {
                  z = 0.1;
                } else {
                  z = x[2];
                }
                // 1 column
                H[0] =  0;
                H[15] =  0;
                // 2 column
                H[1] =  0;
                H[16] =  0;
                // 3 column
                H[2] =  -(f*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9]))*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/(z*z);
                H[17] =   (f*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9]))*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/(z*z);
                // 4 column
                H[3] =  (f*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9])*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]))/z;
                H[18] =          (f*(2*x[6]*x[9] - 2*x[7]*x[8])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z;
                // 5 column
                H[4] =           (f*(2*x[6]*x[9] + 2*x[7]*x[8])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z;
                H[19] =  -(f*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9])*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]))/z;
                // 6 column
                H[5] =  -(f*(2*x[6]*x[8] - 2*x[7]*x[9])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z;
                H[20] =  -(f*(2*x[6]*x[7] + 2*x[8]*x[9])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z;
                // 7 column
                H[6] =  - (x[7]*((2*f*x[6]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/z + (f*(2*x[6]*x[3] - 2*x[8]*x[5] + 2*x[9]*x[4])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2 - (x[6]*((2*f*x[7]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/z - (f*(2*x[7]*x[3] + 2*x[8]*x[4] + 2*x[9]*x[5])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2 - (x[9]*((2*f*x[8]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/z + (f*(2*x[6]*x[5] - 2*x[7]*x[4] + 2*x[8]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2 - (x[8]*((2*f*x[9]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/z + (f*(2*x[6]*x[4] + 2*x[7]*x[5] - 2*x[9]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2;
                H[21] =    (x[6]*((2*f*x[7]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/z - (f*(2*x[6]*x[5] - 2*x[7]*x[4] + 2*x[8]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2 + (x[7]*((2*f*x[6]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/z + (f*(2*x[6]*x[4] + 2*x[7]*x[5] - 2*x[9]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2 + (x[8]*((2*f*x[9]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/z - (f*(2*x[6]*x[3] - 2*x[8]*x[5] + 2*x[9]*x[4])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2 + (x[9]*((2*f*x[8]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/z - (f*(2*x[7]*x[3] + 2*x[8]*x[4] + 2*x[9]*x[5])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2;
                // 8 column
                H[7] =  (x[7]*((2*f*x[9]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/z + (f*(2*x[6]*x[4] + 2*x[7]*x[5] - 2*x[9]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2 - (x[8]*((2*f*x[6]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/z + (f*(2*x[6]*x[3] - 2*x[8]*x[5] + 2*x[9]*x[4])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2 - (x[6]*((2*f*x[8]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/z + (f*(2*x[6]*x[5] - 2*x[7]*x[4] + 2*x[8]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2 + (x[9]*((2*f*x[7]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/z - (f*(2*x[7]*x[3] + 2*x[8]*x[4] + 2*x[9]*x[5])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2;
                H[22] =  (x[8]*((2*f*x[6]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/z + (f*(2*x[6]*x[4] + 2*x[7]*x[5] - 2*x[9]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2 - (x[9]*((2*f*x[7]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/z - (f*(2*x[6]*x[5] - 2*x[7]*x[4] + 2*x[8]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2 + (x[6]*((2*f*x[8]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/z - (f*(2*x[7]*x[3] + 2*x[8]*x[4] + 2*x[9]*x[5])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2 - (x[7]*((2*f*x[9]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/z - (f*(2*x[6]*x[3] - 2*x[8]*x[5] + 2*x[9]*x[4])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2;
                // 9 column
                H[8] =  (x[7]*((2*f*x[8]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/z + (f*(2*x[6]*x[5] - 2*x[7]*x[4] + 2*x[8]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2 + (x[6]*((2*f*x[9]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/z + (f*(2*x[6]*x[4] + 2*x[7]*x[5] - 2*x[9]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2 - (x[9]*((2*f*x[6]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/z + (f*(2*x[6]*x[3] - 2*x[8]*x[5] + 2*x[9]*x[4])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2 - (x[8]*((2*f*x[7]*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9])))/z - (f*(2*x[7]*x[3] + 2*x[8]*x[4] + 2*x[9]*x[5])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2;
                H[23] =  (x[8]*((2*f*x[7]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/z - (f*(2*x[6]*x[5] - 2*x[7]*x[4] + 2*x[8]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2 + (x[9]*((2*f*x[6]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/z + (f*(2*x[6]*x[4] + 2*x[7]*x[5] - 2*x[9]*x[3])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2 - (x[6]*((2*f*x[9]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/z - (f*(2*x[6]*x[3] - 2*x[8]*x[5] + 2*x[9]*x[4])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2 - (x[7]*((2*f*x[8]*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9])))/z - (f*(2*x[7]*x[3] + 2*x[8]*x[4] + 2*x[9]*x[5])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z))/2;
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
                H[12] =   0;
                H[27] =  -f;
                // 14 column
                H[13] =  -f;
                H[28] =   0;
                // 15 column
                H[14] =  0;
                H[29] =  0;
                // // 1 column
                // H[0] =  0;
                // H[15] =  0;
                // // 2 column
                // H[1] =  0;
                // H[16] =  0;
                // // 3 column
                // H[2] =  -(f*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9]))*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/(z*z);
                // H[17] =   (f*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9]))*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/(z*z);
                // // 4 column
                // H[3] =  (f*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9])*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]))/z;
                // H[18] =          (f*(2*x[6]*x[9] - 2*x[7]*x[8])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z;
                // // 5 column
                // H[4] =           (f*(2*x[6]*x[9] + 2*x[7]*x[8])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z;
                // H[19] =  -(f*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9])*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]))/z;
                // // 6 column
                // H[5] =  -(f*(2*x[6]*x[8] - 2*x[7]*x[9])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z;
                // H[20] =  -(f*(2*x[6]*x[7] + 2*x[8]*x[9])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z;
                // // 7 column
                // H[6] =  0;
                // H[21] =  0;
                // // 8 column
                // H[7] =  0;
                // H[22] =  0;
                // // 9 column
                // H[8] =  0;
                // H[23] =  0;
                // // 10 column
                // H[9] =  0;
                // H[24] =  0;
                // // 11 column
                // H[10] =  0;
                // H[25] =  0;
                // // 12 column
                // H[11] =  0;
                // H[26] =  0;
                // // 13 column
                // H[12] =  0;
                // H[27] =  0;
                // // 14 column
                // H[13] =  0;
                // H[28] =  0;
                // // 15 column
                // H[14] =  0;
                // H[29] =  0;

                return true;
            }

            virtual bool getInnovation(float * z, float * x) override
            {
                
                // Saturate z estimation to avoid singularities
                float z_est;
                if (x[2] < 0.1)
                {
                    z_est = 0.1;
                } else {
                    z_est = x[2];
                }
                
                float _predictedObservation[2];
                // predicted values
                _predictedObservation[0] = (f*(x[3]*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + x[4]*(2*x[6]*x[9] + 2*x[7]*x[8]) - x[5]*(2*x[6]*x[8] - 2*x[7]*x[9]))*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z_est - f*(x[14] - _rates[1]);
                _predictedObservation[1] =  - f*(x[13] - _rates[0]) - (f*(x[4]*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - x[3]*(2*x[6]*x[9] - 2*x[7]*x[8]) + x[5]*(2*x[6]*x[7] + 2*x[8]*x[9]))*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]))/z_est;
                z[0] = _deltaX - _predictedObservation[0];
                z[1] = _deltaY - _predictedObservation[1];
                
                return true;
            }
            
            virtual void getCovarianceCorrection(float * R) override
            {
                R[0] = 1.0;
                R[4] = 1.0;
            }
            
            virtual void getMeasures(eskf_state_t & state) override
            {
                _rates[0] = state.angularVelocities[0];
                _rates[1] = state.angularVelocities[1];
                _rates[2] = state.angularVelocities[2];
            }

            virtual bool Zinverse(float * Z, float * invZ) override
            {
                // Since Z by default is a 3x3 matrix, temporarily copy its
                // values in a 2x2 matrix to avoid messing up with the indexes
                float Ztmp[4];
                Ztmp[0] = Z[0];
                Ztmp[1] = Z[1];
                Ztmp[2] = Z[3];
                Ztmp[3] = Z[4];
                float invZtmp[4];
                float tmp[2];
                if (cholsl(Ztmp, invZtmp, tmp, 2))  return 1;
                invZ[0] = invZtmp[0];
                invZ[1] = invZtmp[1];
                invZ[3] = invZtmp[2];
                invZ[4] = invZtmp[3];
                return 0;
            }
            
            virtual bool isOpticalFlow(void) override { return true; }

    };  // class OpticalFlow 

} // namespace hf
