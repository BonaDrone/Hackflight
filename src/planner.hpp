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

        friend class Hackflight;
        
        private:
          
          static const uint8_t WP_ARM             = 1;
          static const uint8_t WP_DISARM          = 2;
          static const uint8_t WP_LAND            = 3;
          static const uint8_t WP_TAKE_OFF        = 4;
          static const uint8_t WP_CHANGE_ALTITUDE = 9;
          static const uint8_t WP_HOVER           = 11;
          
          action_t _mission[256];
          int _currentActionIndex = 0;
          action_t _currentAction;
          
          // for actions that involve time
          uint32_t _startActionTime = micros();
          
          float _integralPosition[3] = {0, 0, 0};
                        
          void loadMission(action_t mission[256])
          {
              int address = 0;
              int actionIndex = 0;
              while (true)
              {
                // read a byte from the current address of the EEPROM
                uint8_t value = EEPROM.read(address);
                if (value == 0 || address == EEPROM.length()) {
                  break;
                }
                int newAddress = addActionToMission(value, mission, actionIndex, address);
                actionIndex += 1;
                address = newAddress + 1;
              }
          } // loadMission
          
          int addActionToMission(uint8_t value, action_t mission[256],
                                 int actionIndex, int address)
          {
            action_t action;
            switch (value) {
              
              case WP_ARM:
              {
                  action.action = WP_ARM;
              } break;
              
              case WP_DISARM:
              {
                  action.action = WP_DISARM;
              } break;
              
              case WP_TAKE_OFF:
              {
                  action.action = WP_TAKE_OFF;
                  action.position[0] = _integralPosition[0];
                  action.position[1] = _integralPosition[1];
                  action.position[2] = EEPROM.read(++address);
              } break;
              
              case WP_LAND:
              {
                  action.action = WP_LAND;
                  action.position[0] = _integralPosition[0];
                  action.position[1] = _integralPosition[1];
                  action.position[2] = 0;
              } break;
              
              case WP_CHANGE_ALTITUDE:
              {
                  action.action = WP_CHANGE_ALTITUDE;
                  action.position[0] = _integralPosition[0];
                  action.position[1] = _integralPosition[1];
                  action.position[2] = EEPROM.read(++address);
              } break;
              
              case WP_HOVER:
              {
                action.action = WP_HOVER;
                action.position[0] = _integralPosition[0];
                action.position[1] = _integralPosition[1];
                action.position[2] = _integralPosition[2];
                action.duration = EEPROM.read(++address);
              } break;
            }
            mission[actionIndex] = action;
            return address;
          } // buildAction

        protected:

            bool isActionComplete(state_t & state, demands_t & demands)
            {
              // XXX each action with its own validator ?
              if (abs(demands.z - state.altitude) > 0.05)
              {
                return false;
              }
              if (((micros() - _startActionTime) / 1000000.0f) < _currentAction.duration)
              {
                return false;
              }
              return true;
            }

        public:
          
            void init(void)
            {
                loadMission(_mission);
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
                  demands.z = _currentAction.position[2];
                  // update starting time
                  _startActionTime  = micros(); 
                  return;
                }
                
            }
            
            // XXX only for debuging purposes
            void printMission(void)
            {
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
