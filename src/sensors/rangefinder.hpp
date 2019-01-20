/*
   rangefinder.hpp : Support for rangefinder sensors (sonar, time-of-flight)

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

#include "debug.hpp"
#include "peripheral.hpp"
#include "filters.hpp"

namespace hf {

    class Rangefinder : public PeripheralSensor {

        public:

            Rangefinder(void) : PeripheralSensor(false, true)
            {
            }

            virtual void getJacobianObservation(float * H, float * x) override
            {
                zeros(H, Mobs, NEsta);
                // auxiliary variable
                float aux1 = (x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]);

                H[0] = -1/(x[2]*x[2] - x[3]*x[3] - x[4]*x[4] + x[5]*x[5]);
                H[1] = 0;
                H[2] = -(2*x[0]*x[2]*x[3])/(aux1*aux1) - (2*x[0]*x[4]*x[5])/(aux1*aux1);
                H[3] = (2*x[0]*x[3]*x[5])/(aux1*aux1) - (2*x[0]*x[2]*x[4])/(aux1*aux1);
                H[4] = 0;
                H[5] = 0;
                H[6] = 0;
                H[7] = 0;
            }

            virtual void getInnovation(float * z, float * x) override
            {
              zeros(z, Mobs, 1);
              // innovation = measured - predicted
              // predicted is pz/R*R_r_i(3,3), where R = rotation matrix
              float predicted = 0;
              z[0] = _distance - predicted;
            }
            
            virtual void getCovarianceCorrection(float * R) override
            {
              zeros(R, Mobs, Mobs);
              R[0] = 1.00f;
            }

        protected:

            virtual void modifyState(eskf_state_t & state, float time) override
            {
              (void)state;
              (void)time;
            }

            virtual bool ready(float time) override
            {
                float newDistance;

                if (distanceAvailable(newDistance)) {

                    static float _time;

                    if (time-_time > UPDATE_PERIOD) {

                        _distance = newDistance;

                        _time = time; 

                        return true;
                    }
                }

                return false; 
            }

            virtual bool shouldUpdateESKF(float time) override
            {
              return true;
            }

            virtual bool distanceAvailable(float & distance) = 0;

        private:

            static constexpr float UPDATE_HZ = 25; // XXX should be using interrupt!

            static constexpr float UPDATE_PERIOD = 1/UPDATE_HZ;

            float _distance;

    };  // class Rangefinder

} // namespace
