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


        protected:

            virtual void modifyState(eskf_state_t & state, float time) override
            {
                (void)time;
                (void)state;
            }

            virtual bool ready(float time) override
            {
                int16_t deltaX=0, deltaY=0;
                _flowSensor.readMotionCount(&deltaX, &deltaY);
                _deltaX = (float)deltaX;
                _deltaY = (float)deltaY;
            }
            
            virtual bool shouldUpdateESKF(float time) override
            {
                static float _time;

                if (time - _time > UPDATE_PERIOD) {
                    _time = time;
                    return true; 
                }
                return false;
            }

        public:

            OpticalFlow(void) : PeripheralSensor(false, true) {}

            bool begin(void)
            {
                bool connected = true;
                if (!_flowSensor.begin()) {
                  connected = false;
                }
                return connected;
            }

    };  // class OpticalFlow 

} // namespace hf
