/*
   quaternion.hpp : Support for treating quaternion as a sensor
   
   Supports IMUs like EM7180 SENtral Sensor Fusion solution, where 
   quaternion is computed in hardware, and simulation platforms like
   UE4, where quaternion is provided by physics engine. For other IMUs 
   and simulators, you can use quaternion-filter classes in filters.hpp.

   Copyright (c) 2018 Simon D. Levy

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

#include <cmath>
#include <math.h>

#include "sensor.hpp"
#include "surfacemount.hpp"
#include "board.hpp"

namespace hf {

    class Quaternion : public SurfaceMountSensor {

        friend class Hackflight;

        public:

            // We make this public so we can use it in different sketches
            static void computeEulerAngles(float q[4], float euler[3])
            {
                // Rotate quaternion from IMU frame to euler convention frame
                // This rotation is pi radians around x-axis
                // See: https://en.m.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft)
                float qt[4];
                qt[0] = q[0];
                qt[1] = q[1];
                qt[2] = -q[2];
                qt[3] = -q[3];
              
                euler[0] = atan2(2.0f*(qt[0]*qt[1]+qt[2]*qt[3]),qt[0]*qt[0]-qt[1]*qt[1]-qt[2]*qt[2]+qt[3]*qt[3]);
                euler[1] =  asin(2.0f*(qt[1]*qt[3]-qt[0]*qt[2]));
                euler[2] = atan2(2.0f*(qt[1]*qt[2]+qt[0]*qt[3]),qt[0]*qt[0]+qt[1]*qt[1]-qt[2]*qt[2]-qt[3]*qt[3]);
                
                euler[0] = int(euler[0]*1000)/1000.0;
                euler[1] = int(euler[1]*1000)/1000.0;
                euler[2] = int(euler[2]*1000)/1000.0;
            }
            
            static void computeqL(float * qL, float * q)
            {        
                qL[0] =  q[0];
                qL[4] =  q[1];
                qL[8] =  q[2];
                qL[12] =  q[3];
                      
                qL[1] = -q[1];
                qL[5] =  q[0];
                qL[9] =  q[3];
                qL[13] = -q[2];
                      
                qL[2]  = -q[2];
                qL[6]  = -q[3];
                qL[10] =  q[0];
                qL[14] =  q[1];
                      
                qL[3] = -q[3];
                qL[7] =  q[2];
                qL[11] = -q[1];
                qL[15] =  q[0];
            }

        protected:

            Quaternion(void):SurfaceMountSensor(false, false)
            {
                memset(_quat, 0, 4*sizeof(float));
            }

            virtual void modifyState(eskf_state_t & state, float time) override
            {
                (void)time;
            }

            virtual bool ready(float time) override
            {
                (void)time;

                return board->getQuaternion(_quat);
            }

        private:

            float _quat[4];

    };  // class Quaternion

} // namespace
