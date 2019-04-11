/*
   mission.hpp : Flight mission planner class

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
 #include "planner.hpp"
 
 namespace hf {
 
    class MissionPlanner : public Planner {
      
    private:

        bool _hasMission;
        // Variables used to store the programmed mission
        action_t _mission[STACK_LENGTH];
        // mission length
        uint8_t _missionLength = 0;
        // Each action will be linked to a position with respect to the starting
        // point. This position marks where the drone should be at the end of
        // the action and takes in to account the position of the previous action 
        float _integralPosition[3] = {0, 0, 0};

        void loadMission(action_t mission[STACK_LENGTH], int startingAddress)
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
        
        int addActionToMission(uint8_t value, action_t mission[STACK_LENGTH],
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
                // action.position[0] = _integralPosition[0];
                // action.position[1] = _integralPosition[1];
                _integralPosition[2] = EEPROM.read(++address);
                action.position[2] = _integralPosition[2];
                break;
            }
            case WP_LAND:
            {
                action.action = WP_LAND;
                // action.position[0] = _integralPosition[0];
                // action.position[1] = _integralPosition[1];
                action.position[2] = 0;
                break;
            }
            case WP_CHANGE_ALTITUDE:
            {
                action.action = WP_CHANGE_ALTITUDE;
                // action.position[0] = _integralPosition[0];
                // action.position[1] = _integralPosition[1];
                _integralPosition[2] = EEPROM.read(++address); 
                action.position[2] = _integralPosition[2];
                break;
            }
            case WP_HOVER:
            {
                action.action = WP_HOVER;
                // action.position[0] = _integralPosition[0];
                // action.position[1] = _integralPosition[1];
                action.position[2] = _integralPosition[2];
                action.duration = EEPROM.read(++address);
                break;
            }
            case WP_GO_FORWARD: // For the moment, movement is time based
            {
              action.action = WP_GO_FORWARD;
              // action.position[0] = _integralPosition[0];
              // action.position[1] = _integralPosition[1];
              action.position[2] = _integralPosition[2];
              action.duration = EEPROM.read(++address);
              break;
            }
            case WP_GO_BACKWARD:
            {
              action.action = WP_GO_BACKWARD;
              // action.position[0] = _integralPosition[0];
              // action.position[1] = _integralPosition[1];
              action.position[2] = _integralPosition[2];
              action.duration = EEPROM.read(++address);
              break;                
            }
            case WP_GO_LEFT:
            {
              action.action = WP_GO_LEFT;
              // action.position[0] = _integralPosition[0];
              // action.position[1] = _integralPosition[1];
              action.position[2] = _integralPosition[2];
              action.duration = EEPROM.read(++address);
              break;                
            }
            case WP_GO_RIGHT:
            {
              action.action = WP_GO_RIGHT;
              // action.position[0] = _integralPosition[0];
              // action.position[1] = _integralPosition[1];
              action.position[2] = _integralPosition[2];
              action.duration = EEPROM.read(++address);
              break;                
            }
            case WP_TURN_CW:
            {
              action.action = WP_TURN_CW;
              // action.position[0] = _integralPosition[0];
              // action.position[1] = _integralPosition[1];
              action.position[2] = _integralPosition[2];
              action.rotationDegrees = EEPROM.read(++address);
              action.clockwise = true;
              break;                                
            }
            case WP_TURN_CCW:
            {
              action.action = WP_TURN_CCW;
              // action.position[0] = _integralPosition[0];
              // action.position[1] = _integralPosition[1];
              action.position[2] = _integralPosition[2];
              action.rotationDegrees = EEPROM.read(++address);
              action.clockwise = false;
              break;                                                
            }
          }
          mission[actionIndex] = action;
          return address;
        } // buildAction

        virtual void endOfAction(state_t & state, demands_t & demands) override
        {
          // Load next action
          _currentActionIndex += 1;
          _currentAction = _mission[_currentActionIndex];
          _startActionYaw = state.UAVState->eulerAngles[2];
          // If we have reached the end of the mission, reset executing flag
          // and return
          if (_currentActionIndex == _missionLength - 1)
          {
              state.executingMission = false;
              // Reset action pointer to enable mission execution again
              _currentActionIndex = 0;
              _currentAction = _mission[0];
          }
        }
    
    public:
      
        void init(int startingAddress)
        {
              _hasMission = false;
              loadMission(_mission, startingAddress);
              reset();
        }
        
        void reset()
        {
          _currentActionIndex = 0;
          _currentAction = _mission[_currentActionIndex];
        }
        
        // XXX only for debuging purposes
        void printMission(void)
        {
            if (!_hasMission)
            {
                return;
            }
            for (int i = 0; i<STACK_LENGTH; i++)
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
      
    }; // class Mission
 
} // namespace hf