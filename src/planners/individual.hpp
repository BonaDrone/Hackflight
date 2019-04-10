/*
   stack.hpp : Stack planner class that enables inmediate action execution

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
 
 #include "planner.hpp"
 
 namespace hf {
 
    class IndividualPlanner : public Planner {
      
    private:

        // Variables used to store the actions to be executed
        action_t _stack[STACK_LENGTH];
        // stack length
        uint8_t _actionsInStack = 0; // Number of actions
        uint8_t _stackIndex = 0;  // Where to add the next action
        // // Each action will be linked to a position with respect to the starting
        // // point. This position marks where the drone should be at the end of
        // // the action and takes in to account the position of the previous action 
        // float _integralPosition[3] = {0, 0, 0};

        virtual void endOfAction(state_t & state, demands_t & demands) override
        {
            _actionsInStack -= 1;
            // Update index from where to grab the next action
            _currentActionIndex = (_currentActionIndex + 1) % STACK_LENGTH;
            // If there are still actions to be performed load the next one
            if (_actionsInStack > 0)
            {
                _currentAction = _stack[_currentActionIndex];
            } else {
                state.executingStack = false;
            }
        } // endOfAction
    
    public:
      
        void init()
        {
            reset();
        }

        void reset()
        {
          _currentActionIndex = 0;
          _stackIndex = 0;
          _actionsInStack = 0;
          _actionStart = true;
        }
        
        void addActionToStack(state_t & state, uint8_t actionCode, int data)
        {
          action_t action = {};
          switch (actionCode) {
            
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
                action.position[0] = state.UAVState->position[0];
                action.position[1] = state.UAVState->position[1];
                action.position[2] = data;
                break;
            }
            case WP_LAND:
            {
                action.action = WP_LAND;
                action.position[0] = state.UAVState->position[0];
                action.position[1] = state.UAVState->position[1];
                action.position[2] = 0;
                break;
            }
            case WP_CHANGE_ALTITUDE:
            {
                action.action = WP_CHANGE_ALTITUDE;
                action.position[0] = state.UAVState->position[0];
                action.position[1] = state.UAVState->position[1];
                action.position[2] = data;
                break;
            }
            case WP_HOVER:
            {
                action.action = WP_HOVER;
                action.position[0] = state.UAVState->position[0];
                action.position[1] = state.UAVState->position[1];
                action.position[2] = state.UAVState->position[2];
                action.duration = data;
                break;
            }
            case WP_GO_FORWARD: // For the moment, movement is time based
            {
              action.action = WP_GO_FORWARD;
              action.position[0] = state.UAVState->position[0];
              action.position[1] = state.UAVState->position[1];
              action.position[2] = state.UAVState->position[2];
              action.duration = data;
              break;
            }
            case WP_GO_BACKWARD:
            {
              action.action = WP_GO_BACKWARD;
              action.position[0] = state.UAVState->position[0];
              action.position[1] = state.UAVState->position[1];
              action.position[2] = state.UAVState->position[2];
              action.duration = data;
              break;                
            }
            case WP_GO_LEFT:
            {
              action.action = WP_GO_LEFT;
              action.position[0] = state.UAVState->position[0];
              action.position[1] = state.UAVState->position[1];
              action.position[2] = state.UAVState->position[2];
              action.duration = data;
              break;                
            }
            case WP_GO_RIGHT:
            {
              action.action = WP_GO_RIGHT;
              action.position[0] = state.UAVState->position[0];
              action.position[1] = state.UAVState->position[1];
              action.position[2] = state.UAVState->position[2];
              action.duration = data;
              break;                
            }
            case WP_TURN_CW:
            {
              action.action = WP_TURN_CW;
              action.position[0] = state.UAVState->position[0];
              action.position[1] = state.UAVState->position[1];
              action.position[2] = state.UAVState->position[2];
              action.rotationDegrees = data;
              action.clockwise = true;
              break;                                
            }
            case WP_TURN_CCW:
            {
              action.action = WP_TURN_CCW;
              action.position[0] = state.UAVState->position[0];
              action.position[1] = state.UAVState->position[1];
              action.position[2] = state.UAVState->position[2];
              action.rotationDegrees = data;
              action.clockwise = false;
              break;                                                
            }
          }
          _stack[_stackIndex] = action;
          _stackIndex = (_stackIndex + 1) % STACK_LENGTH;
          _actionsInStack += 1;
          _currentAction = _stack[_currentActionIndex];
        } // addActionToStack
        
      
    }; // class IndividualPlanner

} // namespace hf