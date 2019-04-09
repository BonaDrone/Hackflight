/*
   althold.hpp : Altitude hold PID controller

   Copyright (c) 2019 Simon D. Levy, Juan Gallostra Acin, Pep Marti-Saumell

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

#include "receiver.hpp"
#include "debug.hpp"
#include "datatypes.hpp"
#include "pidcontroller.hpp"
#include "pidcontrollers/setpoint.hpp"

namespace hf {

    class AltitudeHold : public PID_Controller {

        friend class Hackflight;

        private: 

              // Arbitrary constants
              const float WINDUP_MAX             = 2.00f;
              const float HOVER_THROTTLE         = -0.12f;
              const float Vz_MAX                 = 1.00f; 
              const float Vz_MIN                 = 0.00f;

              // Setpoint class for PID control
              Setpoint setpoint;

              // Minimum altitude, set by constructor
              float _minAltitude;

              bool setpointIsActive(state_t & state)
              {
                  return state.executingMission;
              }

          protected:
            
              bool modifyDemands(state_t & state, demands_t & demands, float currentTime)
              {
                  // XXX Comment it now, rethink later
                  // Don't do anything till we've reached sufficient altitude
                  // if (state.UAVState->position[2] < _minAltitude) return false;

                  float correction = 0;
                  if (setpointIsActive(state))
                  {
                    // Correct based on setpoint
                    if(setpoint.gotSetpointCorrection(demands.setpoint[2],
                          state.UAVState->position[2], 
                          state.UAVState->linearVelocities[2], 
                          currentTime, correction)){
                      demands.throttle = correction + HOVER_THROTTLE;
                      return true;                      
                    }
                  } else {
                    // Correct based on throttle
                    if (setpoint.gotManualCorrection(demands.throttle, 
                            state.UAVState->position[2], 
                            state.UAVState->linearVelocities[2], 
                            currentTime, correction)) {
                        demands.throttle = correction + HOVER_THROTTLE;
                        return true;
                    }
                  }
                  return false;

              }

              virtual bool shouldFlashLed(void) override 
              {
                  return true;
              }

        public:

            AltitudeHold(float altHoldP, float altHoldVelP, float altHoldVelI, float altHoldVelD, float minAltitude=0.1) : _minAltitude(minAltitude)
            {
                // Initialize PID controller
                setpoint.init(altHoldP, altHoldVelP, altHoldVelI, altHoldVelD, WINDUP_MAX, Vz_MAX, Vz_MIN);
            }

    };  // class AltitudeHold

} // namespace hf
