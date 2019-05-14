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
#include "datatypes.hpp"

namespace hf {

    // XXX It can be a good idea to create an Action class that is in charge 
    // of validating himself (and maybe executing?) so that validation and 
    // creation code can be delegated there
    typedef struct {

      uint8_t action;
      float position[3];
      float rotationDegrees;
      float duration;
      bool    clockwise;

    } action_t;

    class Planner {
        
        private:
                      
            virtual void endOfAction(state_t & state, demands_t & demands) = 0;

        protected:
            
            static const uint16_t STACK_LENGTH = 256;
            
            // Required constants and values
            const float DISTANCE_THRESHOLD = 0.1;
            // To achieve desired movement (F/B, L/R and rotations)
            // XXX Must be tuned
            const float TARGET_ROLL        = 5;
            const float TARGET_PITCH       = 5;     // angle in degrees
            const float TARGET_YAW_RATE    = 2;     // angular velocity in degrees per second
            const float TAKEOF_DURATION    = 1.00;  // takeof first stage duration in s XXX to be tuned
            
            const float MAX_HEIGHT         = 3;
          
            // for actions that involve time
            bool _actionStart = true;
            uint32_t _startActionTime;
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
                      // Serial.println("Arm");
                      actionComplete = state.armed;
                      break;
                  }
                  case WP_DISARM:
                  {
                      // Serial.println("Disarm");
                      actionComplete = !state.armed;
                      break;
                  }
                  case WP_TAKE_OFF:
                  case WP_CHANGE_ALTITUDE:
                  {
                      // Serial.println("Takeoff or change alt");
                      if (fabs(action.position[2] - state.UAVState->position[2]) < DISTANCE_THRESHOLD)
                      {
                          actionComplete = true;
                      }
                      break;
                  }
                  case WP_LAND:
                  {
                      // Serial.println("Land");
                      actionComplete = (state.UAVState->position[2] < DISTANCE_THRESHOLD);
                      break;
                  }
                  case WP_HOVER:
                  {
                      // Serial.println("Hover");
                      float elapsedTime = (micros() - _startActionTime) / 1000000.0f;
                      actionComplete =  (elapsedTime > action.duration);
                      break;
                  }
                  // We are not coupling validators here because when changing
                  // to distance based programing we will need different validators
                  case WP_GO_FORWARD: // For the moment, movement is time based
                  {
                    // Serial.println("Forward");
                    float elapsedTime = (micros() - _startActionTime) / 1000000.0f;
                    actionComplete =  (elapsedTime > action.duration);
                    break;
                  }
                  case WP_GO_BACKWARD:
                  {
                    // Serial.println("Back");
                    float elapsedTime = (micros() - _startActionTime) / 1000000.0f;
                    actionComplete =  (elapsedTime > action.duration);
                    break;                    
                  }
                  case WP_GO_LEFT:
                  {
                    // Serial.println("Left");
                    float elapsedTime = (micros() - _startActionTime) / 1000000.0f;
                    actionComplete =  (elapsedTime > action.duration);
                    break;                  
                  }
                  case WP_GO_RIGHT:
                  {
                    // Serial.println("Right");
                    float elapsedTime = (micros() - _startActionTime) / 1000000.0f;
                    actionComplete =  (elapsedTime > action.duration);
                    break;
                  }
                  case WP_TURN_CW: // End yaw smaller than starting yaw 
                  { 
                    // Serial.println("Turn CW");
                    actionComplete = (state.UAVState->eulerAngles[2] < _startActionYaw - action.rotationDegrees);
                    break;                                
                  }
                  case WP_TURN_CCW: // End yaw bigger than starting yaw
                  {
                    // Serial.println("Turn CCW");
                    actionComplete = (state.UAVState->eulerAngles[2] > _startActionYaw + action.rotationDegrees);
                    break;                                                
                  }

                }
                return actionComplete;
            }

        public:
            
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

            void executeAction(state_t & state, demands_t & demands, bool safeToArm,
               PID_Controller * controllers[256], uint8_t pidCount)
            {
                if (_actionStart)
                {
                    _actionStart = false; 
                    _startActionTime = micros();
                    _startActionYaw = state.UAVState->eulerAngles[2];
                    // Limit maximum height of the actions
                    if (_currentAction.position[2] > MAX_HEIGHT)
                        _currentAction.position[2] = MAX_HEIGHT;
                } 
                switch (_currentAction.action) {
                  case WP_ARM:
                  {
                      if (safeToArm)
                          state.armed = true;
                      break;                    
                  }
                  case WP_DISARM:
                  {
                      state.armed = false;
                      break;
                  }
                  case WP_TAKE_OFF:
                  {
                      // while taking off deactivate position hold
                      if ((micros() - _startActionTime)/1000000.0 < TAKEOF_DURATION)
                      {
                          for (int k=0; k < pidCount; k++)
                          {
                              if (controllers[k]->getPIDType() == POSHOLD) controllers[k]->deactivate();
                          }
                      }
                      else {
                          for (int k=0; k < pidCount; k++)
                          {
                              if (controllers[k]->getPIDType() == POSHOLD) controllers[k]->activate();
                          }
                      }
                      // Update setpoint
                      for (int i=0; i<3; i++)
                      {
                          demands.setpoint[i] = _currentAction.position[i];
                      }
                      break;                    
                  }
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

                if (isActionComplete(_currentAction, state, demands))
                {   
                    // reset target angles and rates
                    for(int i=0; i<3; ++i)
                    {
                        demands.setpointAngle[i] = 0;
                        demands.setpointRate[i] = 0;
                    }
                    // This enables us to define planner specific behaviour that
                    // gets executed when an action is finished
                    endOfAction(state, demands);
                    _actionStart  = true;
                }
            }

    };  // class Planner

} // namespace hf
