/*
   planner.hpp : Flight planner class

   Copyright (c) 2018 Juan Gallostra

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

#include <EEPROM.h>

namespace hf {

    // XXX It can be a good idea to create an Action class that is in charge 
    // of validating himself (and maybe executing?) so that validation and 
    // creation code can be delegated there
    typedef struct {

      uint8_t action;
      uint8_t position[3];
      uint8_t rotationDegrees;
      uint8_t duration;
      bool    clockwise;

    } action_t;

    class Planner {
        
        private:
                      
            virtual void endOfAction(state_t & state) = 0;

        protected:
            // Set of possible actions
            static const uint8_t WP_ARM             = 1;
            static const uint8_t WP_DISARM          = 2;
            static const uint8_t WP_LAND            = 3;
            static const uint8_t WP_TAKE_OFF        = 4;
            static const uint8_t WP_GO_FORWARD      = 5;
            static const uint8_t WP_GO_BACKWARD     = 6;
            static const uint8_t WP_GO_LEFT         = 7;
            static const uint8_t WP_GO_RIGHT        = 8;
            static const uint8_t WP_CHANGE_ALTITUDE = 9;
            static const uint8_t WP_CHANGE_SPEED    = 10; // XXX To be implemented
            static const uint8_t WP_HOVER           = 11;
            static const uint8_t WP_TURN_CW         = 12;
            static const uint8_t WP_TURN_CCW        = 13;
            
            static const uint16_t STACK_LENGTH = 256;
            
            // Required constants and values
            const float DISTANCE_THRESHOLD = 0.2;
            // To achieve desired movement (F/B, L/R and rotations)
            // XXX Must be tuned
            const float TARGET_ROLL        = 5;
            const float TARGET_PITCH       = 5; // angle in degrees
            const float TARGET_YAW_RATE    = 2; // angular velocity in degrees per second
          
            // for actions that involve time
            uint32_t _startActionTime = micros();
            float _startActionYaw;

            uint8_t _currentActionIndex = 0;
            action_t _currentAction;

            bool isActionComplete(action_t action, state_t & state, demands_t & demands)
            {
                // Action validators
                bool actionComplete = false;
                
                switch (action.action) {
                  case WP_ARM:
                  {
                      actionComplete = state.armed;
                      break;
                  }
                  case WP_DISARM:
                  {
                      actionComplete = !state.armed;
                      break;
                  }
                  case WP_TAKE_OFF:
                  case WP_CHANGE_ALTITUDE:
                  {
                      if (fabs(action.position[2] - state.UAVState->position[2]) < DISTANCE_THRESHOLD)
                      {
                          actionComplete = true;
                      }
                      break;
                  }
                  case WP_LAND:
                  {
                      actionComplete = (state.UAVState->position[2] < DISTANCE_THRESHOLD);
                      break;
                  }
                  case WP_HOVER:
                  {
                      float elapsedTime = (micros() - _startActionTime) / 1000000.0f;
                      actionComplete =  (elapsedTime > action.duration);
                      break;
                  }
                  // We are not coupling validators here because when changing
                  // to distance based programing we will need different validators
                  case WP_GO_FORWARD: // For the moment, movement is time based
                  {
                    float elapsedTime = (micros() - _startActionTime) / 1000000.0f;
                    actionComplete =  (elapsedTime > action.duration);
                    break;
                  }
                  case WP_GO_BACKWARD:
                  {
                    float elapsedTime = (micros() - _startActionTime) / 1000000.0f;
                    actionComplete =  (elapsedTime > action.duration);
                    break;                    
                  }
                  case WP_GO_LEFT:
                  {
                    float elapsedTime = (micros() - _startActionTime) / 1000000.0f;
                    actionComplete =  (elapsedTime > action.duration);
                    break;                  
                  }
                  case WP_GO_RIGHT:
                  {
                    float elapsedTime = (micros() - _startActionTime) / 1000000.0f;
                    actionComplete =  (elapsedTime > action.duration);
                    break;
                  }
                  case WP_TURN_CW: // End yaw smaller than starting yaw 
                  { 
                    actionComplete = (state.UAVState->eulerAngles[2] < _startActionYaw - action.rotationDegrees);
                    break;                                
                  }
                  case WP_TURN_CCW: // End yaw bigger than starting yaw
                  {
                    actionComplete = (state.UAVState->eulerAngles[2] > _startActionYaw + action.rotationDegrees);
                    break;                                                
                  }

                }
                return actionComplete;
            }

        public:
            
            void executeAction(state_t & state, demands_t & demands)
            {
                switch (_currentAction.action) {
                  case WP_ARM:
                  {
                    state.armed = true;
                    break;                    
                  }
                  case WP_DISARM:
                  {
                    state.armed = false;
                    break;
                  }
                  case WP_TAKE_OFF:
                  case WP_LAND:
                  {
                    // Update setpoint
                    for (int i=0; i<3; i++)
                    {
                      demands.setpoint[i] = _currentAction.position[i];
                    }
                    break;
                  }
                  case WP_CHANGE_ALTITUDE:
                  {
                      demands.setpoint[2] = _currentAction.position[2];
                      break;                    
                  }
                  case WP_HOVER:
                  {
                      break;
                  }
                  // We are ot coupling validators here because when changing
                  // to distance based programing we will need different validators
                  case WP_GO_FORWARD: // For the moment, movement is time based
                  {
                      demands.setpointAngle[0] = -TARGET_PITCH;
                      break;
                  }
                  case WP_GO_BACKWARD:
                  {
                      demands.setpointAngle[0] = TARGET_PITCH;
                      break;                    
                  }
                  case WP_GO_LEFT:
                  {
                      demands.setpointAngle[1] = -TARGET_ROLL;
                      break;                  
                  }
                  case WP_GO_RIGHT:
                  {
                      demands.setpointAngle[1] = TARGET_ROLL;
                      break;
                  }
                  case WP_TURN_CW: // End yaw smaller than starting yaw 
                  { 
                      demands.setpointRate[2] = -TARGET_YAW_RATE;
                      break;                                
                  }
                  case WP_TURN_CCW: // End yaw bigger than starting yaw
                  {
                      demands.setpointRate[2] = TARGET_YAW_RATE;
                      break;                                                
                  }
                  
                }
              
                // If the action is complete, load the next one and reset angle
                // and rate targets
                if (isActionComplete(_currentAction, state, demands))
                {
                  // update starting time and yaw
                  _startActionTime  = micros();
                  _startActionYaw = state.UAVState->eulerAngles[2];
                  // reset target angles and rates
                  for(int i=0; i<3; ++i)
                  {
                    demands.setpointAngle[i] = 0;
                    demands.setpointRate[i] = 0;
                  }
                  
                  // This enables us to define planner specific behaviour that
                  // gets executed when an action is finished
                  endOfAction(state);
                  
                }
            }

    };  // class Planner

} // namespace hf
