/*
   pmw3901.hpp : Support for PMW3901 optical flow

   Copyright (c) 2018 Juan Gallostra Acin & Pep Marti Saumell

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

#include <PMW3901.h>
#include "sensors/opticalflow.hpp"

namespace hf {

    class PMW3901_Flow : public OpticalFlow {

        private:
          // Use digital pin 12 for chip select and SPI1 port for comms
          PMW3901 * _flow = new PMW3901(12, &SPI1);

        protected:

            virtual bool getAccumulatedMotion(float & deltaX, float & deltaY) override
            {
                int16_t _deltaX, _deltaY;
                _flow->readMotionCount(&_deltaX, &_deltaY);
                deltaX = (float)_deltaX;
                deltaY = (float)_deltaY;
                return true;
            }

        public:

            bool begin(void)
            {
                bool connected = true;
                if (!_flow->begin()) {
                  connected = false;
                }
                return connected;
            }

    }; // class VL53L1X_Rangefinder 

} // namespace hf
