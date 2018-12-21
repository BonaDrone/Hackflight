/*
   accelerometer.hpp : Support for accelerometer

   Hackflight requires your Board implementation to provide the
   quaternion directly, but access to accelerometer could be useful
   for other kinds of sensor fusion (altitude hold).

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
#include "sensor.hpp"
#include "surfacemount.hpp"
#include "board.hpp"
#include "filters/linalg.hpp"

namespace hf {

    class Accelerometer : public SurfaceMountSensor {

        friend class Hackflight;

        public:

            Accelerometer(int observationRows)
            {
                memset(_gs, 0, 3*sizeof(float));
                setObservationRows(observationRows);
            }

        protected:

            virtual void modifyState(state_t & state, float time) override
            {
                // Here is where you'd do sensor fusion
                (void)state;
                (void)time;
            }

            virtual bool ready(float time) override
            {
                (void)time;

                return board->getAccelerometer(_gs);
            }
            
            virtual void getJacobianObservation(Matrix * H, matrix * x, int errorStates) override
            {
                // Set Jacobian Dimensions
                H.setDimensions(getObservationRows(), errorStates);
                // First Column
                H._vals[0][0]  =  0.0;
                H._vals[1][0]  =  x._vals[0][0]*x._vals[0][0] - 
                                  x._vals[1][0]*x._vals[1][0] - 
                                  x._vals[2][0]*x._vals[2][0] +
                                  x._vals[3][0]*x._vals[3][0];
                H._vals[2][0]  = - 2*x._vals[0][0]*x._vals[1][0] - 
                                   2*x._vals[2][0]*x._vals[3][0];
                // Second Column
                H._vals[0][1]  = -x._vals[0][0]*x._vals[0][0] + 
                                  x._vals[1][0]*x._vals[1][0] +
                                  x._vals[2][0]*x._vals[2][0] - 
                                  x._vals[3][0]*x._vals[3][0];
                H._vals[1][1]  =  0.0;
                H._vals[2][1]  = 2*x._vals[1][0]*x._vals[3][0] -
                                 2*x._vals[0][0]*x._vals[2][0];
                // Third Column
                H._vals[0][2]  = 2*x._vals[0][0]*x._vals[1][0] + 
                                 2*x._vals[2][0]*x._vals[3][0];
                H._vals[1][2]  = 2*x._vals[0][0]*x._vals[2][0] -
                                 2*x._vals[1][0]*x._vals[3][0];
                H._vals[2][2]  = 0.0;              
            }

            virtual void getInnovation(Matrix * z, Matrix * x) override
            {
                // We might have to normalize these two vectors (y and h)
                z.setDimensions(getObservationRows(), 1);
                // Predicted Observations
                _predictedObservation[0] = (2*x._vals[0][0]*x._vals[2][0] - 
                                            2*x._vals[1][0]*x._vals[3][0])*-9.80665;
                _predictedObservation[1] = (-2*x._vals[0][0]*x._vals[1][0] - 
                                             2*x._vals[2][0]*x._vals[3][0])*-9.80665;
                _predictedObservation[2] = (-x._vals[0][0]*x._vals[0][0] + 
                                             x._vals[1][0]*x._vals[1][0] +
                                             x._vals[2][0]*x._vals[2][0] -
                                             x._vals[3][0]*x._vals[3][0])*-9.80665;
                // innovation = measured - predicted
                z._vals[0][0] = _gs[0]*9.80665 - _predictedObservation[0];
                z._vals[1][0] = _gs[1]*9.80665 - _predictedObservation[1];
                z._vals[2][0] = _gs[2]*9.80665 - _predictedObservation[2];
            }
            
            virtual void getCovarianceCorrection(Matrix * R) override
            {
                R.setDimensions(getObservationRows(), _observationRows);
                // Approximate the process noise using a small constant
                R._vals[0][0] = 0.0001f;
                R._vals[1][1] = 0.0001f;
                R._vals[2][2] = 0.0001f;
            }

        private:

            float _gs[3];
            float _predictedObservation[3];

    };  // class Accelerometer

} // namespace
