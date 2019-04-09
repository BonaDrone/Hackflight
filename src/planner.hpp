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
          
          bool _hasMission;
          
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
          
          // Required constants and values
          const float DISTANCE_THRESHOLD = 0.2;
          // To achieve desired movement (F/B, L/R and rotations)
          // XXX Must be tuned
          const float TARGET_ROLL        = 5;
          const float TARGET_PITCH       = 5; // angle in degrees
          const float TARGET_YAW_RATE    = 2; // angular velocity in degrees per second
          
          // Variables used to store the programmed mission
          action_t _mission[256];
          uint8_t _currentActionIndex = 0;
          action_t _currentAction;
          // mission length
          uint8_t _missionLength = 0;
          
          // for actions that involve time
          uint32_t _startActionTime = micros();
          float _startActionYaw;
          
          // Each action will be linked to a position with respect to the starting
          // point. This position marks where the drone should be at the end of
          // the action and takes in to account the position of the previous action 
          float _integralPosition[3] = {0, 0, 0};
                        
          void loadMission(action_t mission[256], int startingAddress)
          {
              int address = startingAddress;
              int actionIndex = 0;
              int EEPROMlength = EEPROM.length();
              // Although we iterate through the whole EEPROM length the index
              // that tells which address to read is the value of the variable 
              // address. Some actions, such as hover, have a value (duration 
              // in this case) linked to them and when this happens we read both
              // action ( EEPROM index i) and value (EEPROM index i+1) in the
              // same iteration. We loop through the whole EEPROM length because
              // it is the maximum number of iterations in the worst case scenario.
              for (int ii = 0; ii < EEPROMlength; ii++)
              {
                // read a byte from the current address of the EEPROM
                uint8_t value = EEPROM.read(address);
                // There is no action with code 0, so if a 0 is read we can
                // safely say that the mission is already loaded and stop reading 
                if (value == 0 || value == 255) {
                  break;
                }
                int newAddress = addActionToMission(value, mission, actionIndex, address);
                actionIndex += 1;
                address = newAddress + 1;
                // If we reach this point is because at least one instruction
                // has been loaded
                _hasMission = true;
                // update mission length
                _missionLength += 1;
              }
          } // loadMission
          
          int addActionToMission(uint8_t value, action_t mission[256],
                                 int actionIndex, int address)
          {
            action_t action = {};
            switch (value) {
              
              case WP_ARM:
              {
                  action.action = WP_ARM;
                  break;
              }
              case WP_DISARM:
              {
                  action.action = WP_DISARM;
                  break;
              }
              case WP_TAKE_OFF:
              {
                  action.action = WP_TAKE_OFF;
                  action.position[0] = _integralPosition[0];
                  action.position[1] = _integralPosition[1];
                  _integralPosition[2] = EEPROM.read(++address);
                  action.position[2] = _integralPosition[2];
                  break;
              }
              case WP_LAND:
              {
                  action.action = WP_LAND;
                  action.position[0] = _integralPosition[0];
                  action.position[1] = _integralPosition[1];
                  action.position[2] = 0;
                  break;
              }
              case WP_CHANGE_ALTITUDE:
              {
                  action.action = WP_CHANGE_ALTITUDE;
                  action.position[0] = _integralPosition[0];
                  action.position[1] = _integralPosition[1];
                  _integralPosition[2] = EEPROM.read(++address); 
                  action.position[2] = _integralPosition[2];
                  break;
              }
              case WP_HOVER:
              {
                  action.action = WP_HOVER;
                  action.position[0] = _integralPosition[0];
                  action.position[1] = _integralPosition[1];
                  action.position[2] = _integralPosition[2];
                  action.duration = EEPROM.read(++address);
                  break;
              }
              case WP_GO_FORWARD: // For the moment, movement is time based
              {
                action.action = WP_GO_FORWARD;
                action.position[0] = _integralPosition[0];
                action.position[1] = _integralPosition[1];
                action.position[2] = _integralPosition[2];
                action.duration = EEPROM.read(++address);
                break;
              }
              case WP_GO_BACKWARD:
              {
                action.action = WP_GO_BACKWARD;
                action.position[0] = _integralPosition[0];
                action.position[1] = _integralPosition[1];
                action.position[2] = _integralPosition[2];
                action.duration = EEPROM.read(++address);
                break;                
              }
              case WP_GO_LEFT:
              {
                action.action = WP_GO_LEFT;
                action.position[0] = _integralPosition[0];
                action.position[1] = _integralPosition[1];
                action.position[2] = _integralPosition[2];
                action.duration = EEPROM.read(++address);
                break;                
              }
              case WP_GO_RIGHT:
              {
                action.action = WP_GO_RIGHT;
                action.position[0] = _integralPosition[0];
                action.position[1] = _integralPosition[1];
                action.position[2] = _integralPosition[2];
                action.duration = EEPROM.read(++address);
                break;                
              }
              case WP_TURN_CW:
              {
                action.action = WP_TURN_CW;
                action.position[0] = _integralPosition[0];
                action.position[1] = _integralPosition[1];
                action.position[2] = _integralPosition[2];
                action.rotationDegrees = EEPROM.read(++address);
                action.clockwise = true;
                break;                                
              }
              case WP_TURN_CCW:
              {
                action.action = WP_TURN_CCW;
                action.position[0] = _integralPosition[0];
                action.position[1] = _integralPosition[1];
                action.position[2] = _integralPosition[2];
                action.rotationDegrees = EEPROM.read(++address);
                action.clockwise = false;
                break;                                                
              }
            }
            mission[actionIndex] = action;
            return address;
          } // buildAction

        protected:

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

            void init(int startingAddress)
            {
                _hasMission = false;
                loadMission(_mission, startingAddress);
                _currentActionIndex = 0;
                _currentAction = _mission[_currentActionIndex];
            }
            
            void executeAction(state_t & state, demands_t & demands)
            {
                Serial.println("Executing action:");
                switch (_currentAction.action) {
                  case WP_ARM:
                  {
                    Serial.println("Arm");
                    state.armed = true;
                    break;                    
                  }
                  case WP_DISARM:
                  {
                    Serial.println("Disarm");
                    state.armed = false;
                    break;
                  }
                  case WP_TAKE_OFF:
                  case WP_LAND:
                  {
                    _currentAction.action == WP_TAKE_OFF ? Serial.println("Take off") : Serial.println("Land");
                    // Update setpoint
                    for (int i=0; i<3; i++)
                    {
                      demands.setpoint[i] = _currentAction.position[i];
                    }
                    break;
                  }
                  case WP_CHANGE_ALTITUDE:
                  {
                      Serial.println("Change altitude");
                      demands.setpoint[2] = _currentAction.position[2];
                      break;                    
                  }
                  case WP_HOVER:
                  {
                      Serial.println("Hover");
                      break;
                  }
                  // We are ot coupling validators here because when changing
                  // to distance based programing we will need different validators
                  case WP_GO_FORWARD: // For the moment, movement is time based
                  {
                      Serial.println("Go forward");
                      demands.setpointAngle[0] = -TARGET_PITCH;
                      break;
                  }
                  case WP_GO_BACKWARD:
                  {
                      Serial.println("Go backward");
                      demands.setpointAngle[0] = TARGET_PITCH;
                      break;                    
                  }
                  case WP_GO_LEFT:
                  {
                      Serial.println("Go Left");
                      demands.setpointAngle[1] = -TARGET_ROLL;
                      break;                  
                  }
                  case WP_GO_RIGHT:
                  {
                      Serial.println("Go right");
                      demands.setpointAngle[1] = TARGET_ROLL;
                      break;
                  }
                  case WP_TURN_CW: // End yaw smaller than starting yaw 
                  { 
                      Serial.println("Turn CW");
                      demads.setpointRate[2] = -TARGET_YAW_RATE;
                      break;                                
                  }
                  case WP_TURN_CCW: // End yaw bigger than starting yaw
                  {
                      Serial.println("Turn CCW");
                      demads.setpointRate[2] = TARGET_YAW_RATE;
                      break;                                                
                  }
                  
                }
              
                // If the action is complete, load the next one
                if (isActionComplete(_currentAction, state, demands))
                {
                  // We've reached the end of the mission, reset executing flag
                  // and return
                  if (_currentActionIndex == _missionLength - 1)
                  {
                      Serial.println("Mission END");
                      state.executingMission = false;
                      return;
                  }
                  // Load next action
                  _currentActionIndex += 1;
                  _currentAction = _mission[_currentActionIndex];
                  // update starting time and yaw
                  _startActionTime  = micros();
                  _startActionYaw = state.UAVState->eulerAngles[2];
                  // reset target angles and rates
                  for(int i=0; i<3; ++i)
                  {
                    demands.setpointAngle[i] = 0;
                    demands.setpointRate[i] = 0;
                  }
                }
                
                
            }

            // XXX only for debuging purposes
            void printMission(void)
            {
                if (!_hasMission)
                {
                    return;
                }
                for (int i = 0; i<256; i++)
                {
                    action_t action = _mission[i];
                    Serial.print("Action ");
                    Serial.println(i);
                    Serial.print("Action Code: ");
                    Serial.println(action.action);
                    Serial.print("Position: ");
                    Serial.print(action.position[0]);
                    Serial.print(",");
                    Serial.print(action.position[1]);
                    Serial.print(",");
                    Serial.println(action.position[2]);
                    Serial.print("Rotation: ");
                    Serial.println(action.rotationDegrees);
                    Serial.print("Clockwise: ");
                    Serial.println(action.clockwise);
                    Serial.print("Duration: ");
                    Serial.println(action.duration);
                } 
            }

    };  // class Planner

} // namespace hf
