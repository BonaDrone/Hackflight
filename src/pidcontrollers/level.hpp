/*
   level.hpp : PID controller for Level mode

   Copyright (c) 2019 Simon D. Levy, Juan Gallostra Acin, Pep Marti-Saumell

   Author: Juan Gallostra

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

#include <cstdint>
#include <cstring>
#include <algorithm>
#include <limits>
#include <cmath>

#include "debug.hpp"
#include "datatypes.hpp"
#include "pidcontroller.hpp"

namespace hf {

    class Level : public PID_Controller {

        friend class Hackflight;

        private:
          
            const float FEED_FORWARD = 1.0;
            
            float PTerms[2];
            
            float _demandsToAngle;
            float _demandsToRate;
            
            void computeReferenceDemands(float _demands[2], state_t &  state, demands_t & demands)
            {
                if (state.executingMission || state.executingStack)
                {
                    _demands[0] = demands.setpointAngle[0];
                    _demands[1] = demands.setpointAngle[1];
                } else {
                    _demands[0] = demands.roll * _demandsToAngle;
                    _demands[1] = demands.pitch * _demandsToAngle;
                }
            }

        public:

            Level(float rollLevelP, float pitchLevelP, float maxAngle = 45, float demandsToRate = 6.0)
            {
                PTerms[0] = rollLevelP;
                PTerms[1] = pitchLevelP;
                // roll and pitch demands go between [-0.5, 0.5] so, for a
                // given max angle, the following relation must hold true:
                // 0.5 * _demandsToAngle = maxAngle
                // Since we work in radians:
                // _demandsToAngle = (maxAngle*PI/180) * 2
                _demandsToAngle = maxAngle * 2 * M_PI / 180.0f;
                _demandsToRate = demandsToRate;
            }

            Level(float rollPitchLevelP) : Level(rollPitchLevelP, rollPitchLevelP)
            {
            }

            bool modifyDemands(state_t & state, demands_t & demands, float currentTime)
            {
                (void)currentTime;

                float _demands[2];
                computeReferenceDemands(_demands, state, demands);
                for (int axis=0; axis<2; ++axis)
                {
                  float error = _demands[axis] - state.UAVState->eulerAngles[axis];
                  _demands[axis] = error * PTerms[axis] + FEED_FORWARD * _demands[axis];
                }
                
                demands.roll = _demands[0]/_demandsToRate;
                demands.pitch = _demands[1]/_demandsToRate;

                // We've always gotta do this!
                return true;
            }

    };  // class Level

} // namespace
