/*
   setpoint.hpp : Helper class for PID controllers using a set-point

   See https://raw.githubusercontent.com/wiki/iNavFlight/inav/images/nav_poshold_pids_diagram.jpg

   Copyright (c) 2019 Simon D. Levy, Juan Gallostra Acin, Pep Marti-Saumell

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

#include "receiver.hpp"
#include "filters.hpp"

namespace hf {

    class Setpoint {

        private: 

            // PID constants set by constructor
            float _posP;
            float _velP;
            float _velI;
            float _velD;

            // Parameter to avoid integral windup
            float _windupMax;
            // Parameters for velocity control outside deadband,
            // Perform a linear map from stick position to desired velocity
            float _m;
            float _n;
            
            float THRESHOLD = 0.1;
            float VERTICAL_VELOCITY = 0.15;
            // Values modified in-flight
            float deltaT;
            float _posTarget;
            bool  _inBandPrev;
            float _lastError;
            float _integralError;
            float _altitudeTarget;
            float _previousTime;
            // derivative error control variable to avoid high values
            // on controller activation 
            bool _firstIteration = true;

            bool inBand(float demand)
            {
                return fabs(demand) < Receiver::STICK_DEADBAND; 
            }

            void resetErrors(void)
            {
                _lastError = 0;
                _integralError = 0;
                _firstIteration = true;
            }
            
            float computeCorrection(float velTarget, float velActual, float deltaT)
            {
                float velError = velTarget - velActual;
                // Update error integral and error derivative
                _integralError = Filter::constrainAbs(_integralError + velError * deltaT, _windupMax);
                float deltaError = (velError - _lastError) / deltaT;
                // Avoid high derivative errors on controller activation by skipping
                // the first derivative computation, where _lastError is 0
                if (_firstIteration)
                {
                  deltaError = 0;
                  _firstIteration = false;
                }
                _lastError = velError;

                // Compute control correction
                return _velP * velError + _velD * deltaError + _velI * _integralError;                       
            }

        public:

            bool gotSetpointCorrection(float setpoint, float posActual,
               float velActual, float currentTime, float & correction)
            {
              // Don't do anything until we have a positive deltaT
              float deltaT = currentTime - _previousTime;
              _previousTime = currentTime;
              if (deltaT == currentTime) return false;

              // compute velocity setpoint
              float velTarget;
              if(fabs(setpoint - posActual) < THRESHOLD)
              {
                velTarget = (setpoint - posActual) * _posP;
              }
              else {
                float sign = (setpoint - posActual) > 0 ? 1 : -1;
                velTarget = VERTICAL_VELOCITY * sign;
              }
              correction = computeCorrection(velTarget, velActual, deltaT);
              
              return true;              
            }
            
            bool gotManualCorrection(float demand, float posActual, 
              float velActual, float currentTime, float & correction)
            {
                // Don't do anything until we have a positive deltaT
                float deltaT = currentTime - _previousTime;
                _previousTime = currentTime;
                if (deltaT == currentTime) return false;

                // Reset position target if moved into stick deadband
                bool inBandCurr = inBand(demand);
                if (inBandCurr && !_inBandPrev) {
                    _posTarget = posActual;
                    // resetErrors();
                }
                // Reset errors when moving stick out of deadband
                // and changing to velocity control
                // if (!inBandCurr && _inBandPrev)
                // {
                    // resetErrors();
                // }
                // compute velocity setpoint: inside deadband from distance error,
                // outside deadband velocity control
                float velTarget;
                if(inBandCurr)
                {
                  velTarget = (_posTarget - posActual) * _posP;
                }
                else {
                  velTarget = demand >= 0 ? _m * fabs(demand) + _n : -(_m * fabs(demand) + _n);
                }
                
                correction = computeCorrection(velTarget, velActual, deltaT);
                _inBandPrev = inBandCurr;
                
                return true;
            }

            void init(float posP, float velP, float velI, float velD,
               float windupMax, float vMax, float vMin)
            {
                _posP = posP; 
                _velP = velP; 
                _velI = velI; 
                _velD = velD; 
                _windupMax = windupMax;
                _m = (vMax - vMin) / (1.0f - Receiver::STICK_DEADBAND);
                _n = vMin - _m * Receiver::STICK_DEADBAND;
                resetErrors();
                _posTarget = 0;
                _previousTime = 0;
                _inBandPrev = false;
            }

    };  // class Setpoint

} // namespace hf
