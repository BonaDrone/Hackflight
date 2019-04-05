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
          
            // const float FEED_FORWARD = 0.5;
            const float FEED_FORWARD = 0.6;
            const float FF_THRESHOLD = 0.1;
            // const float WINDUP_MAX = 1.0;
            
            float PTerms[2];
            
            float _demandsToAngle;
            float _demandsToRate;


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

            bool modifyDemands(eskf_state_t & state, demands_t & demands, float currentTime)
            {
                (void)currentTime;

                float _demands[2] = {demands.roll, demands.pitch};
                for (int axis=0; axis<2; ++axis)
                {
                  float error = _demands[axis] * _demandsToAngle - state.eulerAngles[axis];
                  // 0.0175 rad ~ 1 degree
                  error = fabs(error) > 0.0175 ? error : 0.0;
                  
                  // XXX Debug 
                  // Serial.println(error, 8);
                  if (fabs(error) > 0.0175) {
                    pinMode(25, OUTPUT);
                    digitalWrite(25, LOW);
                  } else {
                    digitalWrite(25, HIGH);
                  }
                  
                  float FF = 0;
                  if (fabs(_demands[axis]) > FF_THRESHOLD) 
                      FF = FEED_FORWARD * _demands[axis];
                  _demands[axis] = error * PTerms[axis] + FF;

                }
                // Output of Level controller should be the desired rates
                demands.roll = _demands[0]/_demandsToRate;
                demands.pitch = _demands[1]/_demandsToRate;

                // We've always gotta do this!
                return true;
            }

    };  // class Level

} // namespace
