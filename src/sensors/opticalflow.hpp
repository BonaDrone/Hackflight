/*
   opticalflow.hpp : Support for PMW3901 optical-flow sensor

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

namespace hf {

    class OpticalFlow : public PeripheralSensor {

        private:

            static constexpr float UPDATE_HZ = 50.0; // XXX should be using interrupt!
            static constexpr float UPDATE_PERIOD = 1.0 / UPDATE_HZ;
            // sensor parameters
            // Specific to the sensor
            float _Npix = 30.0; // [pixels] (same in x and y)
            float _thetapix = (M_PI/180) * 4.2; // [rad] (same in x and y)
            float _omegaFactor = 1.25;
            // flow measures
            float _deltaX = 0;
            float _deltaY = 0;
            // Time elapsed between corrections
            float deltat = 0.0;
            // angular velocities
            float _rates[3];

        protected:
            
            virtual bool shouldUpdateESKF(float time, state_t & state) override
            {
                static float _time;

                if (time - _time > UPDATE_PERIOD) {
                    float deltaX=0, deltaY=0;
                    getAccumulatedMotion(deltaX, deltaY);
                    // To match camera frame
                    deltat = time -_time;
                    _time = time;
                    _deltaX = -deltaX / deltat;
                    _deltaY = deltaY / deltat;
                                        
                    return true; 
                }
                return false;
            }

        public:

            OpticalFlow(void) : PeripheralSensor(false, true) {}

            virtual bool getJacobianObservation(float * H, float * x) override
            {
                float z_est;
                // Saturate z estimation to avoid singularities
                if (x[2] < 0.1)
                {
                    z_est = 0.1;
                } else {
                    z_est = x[2];
                }

                // Jacobian of measurement model with respect to the error states
                // dh/dx * dx/ddx where dx/ddx = blkdiag(I_6 Q)
                // 
                //   H = [ 0 0 h_zx h_vx  0   0 0 0 0;...
                //         0 0 h_zy  0   h_vy 0 0 0 0;];
                float rotationComponent = x[6]*x[6]-x[7]*x[7]-x[8]*x[8]+x[9]*x[9]; // R(3,3)
                // Derive x measurement equation with respect to the error states (effectively vx and z)
                H[2] = (_Npix * deltat / _thetapix) * ((rotationComponent * x[3]) / (-z_est * z_est));
                H[3] = (_Npix * deltat / _thetapix) * (rotationComponent / z_est);
                
                // Derive y measurement equation with respect to the error states (effectively vy and z)
                H[11] = (_Npix * deltat / _thetapix) * ((rotationComponent * x[4]) / (-z_est * z_est));
                H[13] = (_Npix * deltat / _thetapix) * (rotationComponent / z_est);
                
                return true;
            }

            virtual bool getInnovation(float * z, float * x) override
            {
                // Saturate z estimation to avoid singularities
                float z_est;
                if (x[2] < 0.1)
                {
                    z_est = 0.1;
                } else {
                    z_est = x[2];
                }
                
                float _predictedObservation[2];
                float rotationComponent = x[6]*x[6]-x[7]*x[7]-x[8]*x[8]+x[9]*x[9]; // R(3,3)            
                // predicted number of accumulated pixels
                _predictedObservation[0] = (deltat * _Npix / _thetapix ) * ((x[3] * rotationComponent / z_est) - _omegaFactor * _rates[1]);
                _predictedObservation[1] = (deltat * _Npix / _thetapix ) * ((x[4] * rotationComponent / z_est) + _omegaFactor *_rates[0]);
                  
                z[0] = _deltaX - _predictedObservation[0];
                z[1] = _deltaY - _predictedObservation[1];
                
                return true;
            }
            
            virtual void getCovarianceCorrection(float * R) override
            {
                R[0] = 0.0625;
                R[4] = 0.0625;
            }
            
            virtual void getMeasures(eskf_state_t & state) override
            {
                _rates[0] = state.angularVelocities[0];
                _rates[1] = state.angularVelocities[1];
                _rates[2] = state.angularVelocities[2];
            }

            virtual bool Zinverse(float * Z, float * invZ) override
            {
                // Since Z by default is a 3x3 matrix, temporarily copy its
                // values in a 2x2 matrix to avoid messing up with the indexes
                float Ztmp[4];
                Ztmp[0] = Z[0];
                Ztmp[1] = Z[1];
                Ztmp[2] = Z[3];
                Ztmp[3] = Z[4];
                float invZtmp[4];
                float tmp[2];
                if (cholsl(Ztmp, invZtmp, tmp, 2))  return 1;
                invZ[0] = invZtmp[0];
                invZ[1] = invZtmp[1];
                invZ[3] = invZtmp[2];
                invZ[4] = invZtmp[3];
                return 0;
            }
            
            virtual bool isOpticalFlow(void) override { return true; }
            
            virtual bool getAccumulatedMotion(float & deltaX, float & deltaY) = 0;

    };  // class OpticalFlow 

} // namespace hf
