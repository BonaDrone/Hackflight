/*
   pidcontroller.hpp : Abstract class for PID controllers

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

#include "datatypes.hpp"

namespace hf {

    class PID_Controller {

        friend class Hackflight;

        protected:

          virtual bool modifyDemands(state_t & state, demands_t & demands, float currentTime) = 0;
        
          virtual void resetErrors(void) {}

          virtual bool shouldFlashLed(void) { return false; }

          uint8_t auxState;
          
          ControllerType_t _PIDType;
        
          // To enable activation and deactivation of controllers at runtime
          void modifyAuxState(uint8_t aux) 
          {
              auxState = aux;
          }
          
        public:
          
          void setPIDType(ControllerType_t pidType)
          {
              _PIDType = pidType;
          }
          
          ControllerType_t getPIDType(void)
          {
              return _PIDType;
          }

    };  // class PID_Controller

} // namespace hf
