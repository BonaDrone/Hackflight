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
          static const uint8_t WP_CHANGE_ALTITUDE = 9;
          static const uint8_t WP_HOVER           = 11;
          
          // Variables used to store the programmed mission
          action_t _mission[256];
          int _currentActionIndex = 0;
          action_t _currentAction;
          
          // for actions that involve time
          uint32_t _startActionTime = micros();
          
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
              }
          } // loadMission
          
          int addActionToMission(uint8_t value, action_t mission[256],
                                 int actionIndex, int address)
          {
            action_t action;
            switch (value) {
              
              case WP_ARM:
                  action.action = WP_ARM;
                  break;
              
              case WP_DISARM:
                  action.action = WP_DISARM;
                  break;
              
              case WP_TAKE_OFF:
                  action.action = WP_TAKE_OFF;
                  action.position[0] = _integralPosition[0];
                  action.position[1] = _integralPosition[1];
                  action.position[2] = EEPROM.read(++address);
                  break;
              
              case WP_LAND:
                  action.action = WP_LAND;
                  action.position[0] = _integralPosition[0];
                  action.position[1] = _integralPosition[1];
                  action.position[2] = 0;
                  break;
              
              case WP_CHANGE_ALTITUDE:
                  action.action = WP_CHANGE_ALTITUDE;
                  action.position[0] = _integralPosition[0];
                  action.position[1] = _integralPosition[1];
                  action.position[2] = EEPROM.read(++address);
                  break;
              
              case WP_HOVER:
                  action.action = WP_HOVER;
                  action.position[0] = _integralPosition[0];
                  action.position[1] = _integralPosition[1];
                  action.position[2] = _integralPosition[2];
                  action.duration = EEPROM.read(++address);
                  break;
            }
            mission[actionIndex] = action;
            return address;
          } // buildAction

        protected:

            bool isActionComplete(state_t & state, demands_t & demands)
            {
              // XXX each action with its own validator ?
              bool actionComplete = true;
              if (abs(demands.altitude - state.altitude) > 0.05)
              {
                actionComplete = false;
              }
              if (((micros() - _startActionTime) / 1000000.0f) < _currentAction.duration)
              {
                actionComplete = false;
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
                // XXX Handle arm and disarm actions, also land
                if (isActionComplete(state, demands))
                {
                  // Load next action
                  _currentActionIndex += 1;
                  _currentAction = _mission[_currentActionIndex];
                  // update setpoint
                  demands.altitude = _currentAction.position[2];
                  // update starting time
                  _startActionTime  = micros(); 
                  return;
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
