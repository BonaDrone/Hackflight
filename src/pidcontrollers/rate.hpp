/*
   rate.hpp : rate PID controller

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

#include <cstdint>
#include <cstring>
#include <algorithm>
#include <limits>
#include <cmath>

#include "receiver.hpp"
#include "filters.hpp"
#include "debug.hpp"
#include "datatypes.hpp"
#include "pidcontroller.hpp"

namespace hf {

    // shared with Hackflight class
    enum {
        AXIS_ROLL = 0,
        AXIS_PITCH,
        AXIS_YAW
    };

    class Rate : public PID_Controller {

        friend class Hackflight;

        private: 

            // Arbitrary constants
            const float GYRO_WINDUP_MAX             = 2.5f;
            const float BIG_GYRO_DEGREES_PER_SECOND = 180.0f; 
            const float BIG_YAW_DEMAND              = 0.1f;
            const float MAX_ARMING_ANGLE_DEGREES    = 25.0f;

            float _bigGyroRate;
            
            float _lastGyro[2];
            float _gyroDeltaError1[2]; 
            float _gyroDeltaError2[2];
            float _errorGyroI[3];

            // Arrays of PID constants for pitch and roll
            float _PConstants[2];
            float _IConstants[2];
            float _DConstants[2];
            // Yaw PID constants set in constructor
           float _gyroYawP; 
           float _gyroYawI;
            
            // Array for rates offset to balance unbalanced quads
            float _offsetError[3];
            
            float degreesToRadians(float deg)
            {
                return M_PI * deg / 180.;
            }

            void init(void)
            {
                // Zero-out previous values for D term
                for (uint8_t axis=0; axis<2; ++axis) {
                    _lastGyro[axis] = 0;
                    _gyroDeltaError1[axis] = 0;
                    _gyroDeltaError2[axis] = 0;
                }

                // Convert degree parameters to radians for use later
                _bigGyroRate = degreesToRadians(BIG_GYRO_DEGREES_PER_SECOND);
                maxArmingAngle = degreesToRadians(MAX_ARMING_ANGLE_DEGREES);

                // Initialize offset errors
                _offsetError[0] = 0.0; // Roll
                _offsetError[1] = 0.0; // Pitch
                _offsetError[2] = 0.0;  // Yaw

                // Initialize gyro error integral
                resetIntegral();
            }

            float computeITermGyro(float error, float rateI, float rcCommand, float gyro[3], float deltat, uint8_t axis)
            {
                // Avoid integral windup
                _errorGyroI[axis] = Filter::constrainAbs(_errorGyroI[axis] + error*deltat, GYRO_WINDUP_MAX);
                
                // Reset integral on quick gyro change or large gyroYaw command
                if ((fabs(gyro[axis]) > _bigGyroRate) || ((axis == AXIS_YAW) && (fabs(rcCommand) > BIG_YAW_DEMAND)))
                {
                  _errorGyroI[axis] = 0;
                }
                                
                return (_errorGyroI[axis] * rateI);
            }

            // Computes PID for pitch or roll
            float computeCyclicPid(float rcCommand, float gyro[3], float deltat, uint8_t imuAxis)
            {
                float error = rcCommand * _demandsToRate - gyro[imuAxis] + _offsetError[imuAxis];      
                
                // I
                float ITerm = computeITermGyro(error, _IConstants[imuAxis], rcCommand, gyro, deltat, imuAxis);
                // ITerm *= _proportionalCyclicDemand;

                // D 
                // derivative of rate instead of derivative of error to avoid "Derivative Kick"
                // http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
                float gyroDelta = gyro[imuAxis] - _lastGyro[imuAxis];
                _lastGyro[imuAxis] = gyro[imuAxis];
                float DTerm = - gyroDelta * _DConstants[imuAxis] / deltat;
                
                return computePid(_PConstants[imuAxis], _PTerm[imuAxis], ITerm, DTerm, gyro, _offsetError[imuAxis], imuAxis);
            } 

            float computePid(float rateP, float PTerm, float ITerm, float DTerm, float gyro[3], float offset, uint8_t axis)
            {
                PTerm = (PTerm * _demandsToRate - gyro[axis] + offset) * rateP;

                return PTerm + ITerm + DTerm;
            }
            
            void computeReferenceDemands(float _demands[3], state_t &  state, demands_t & demands)
            {
              _demands[0] = demands.roll;
              _demands[0] = demands.pitch;
              _demands[2] = (state.executingMission || state.executingStack) ? demands.setpointRate[2] : demands.yaw;
            }

        protected:

            // For PTerm computation
            float _PTerm[2]; // roll, pitch

            float maxArmingAngle;

            float _demandsToRate;

            // proportion of cyclic demand compared to its maximum
            // float _proportionalCyclicDemand;

            void resetIntegral(void)
            {
                _errorGyroI[AXIS_ROLL] = 0;
                _errorGyroI[AXIS_PITCH] = 0;
                _errorGyroI[AXIS_YAW] = 0;
            }

        public:

            Rate(float gyroRollP, float gyroRollI, float gyroRollD,
                       float gyroPitchP, float gyroPitchI, float gyroPitchD,
                       float gyroYawP, float gyroYawI, float demandsToRate = 1.0f) :
                _gyroYawP(gyroYawP), 
                _gyroYawI(gyroYawI),
                _demandsToRate(demandsToRate)
            {
                init();
                // Constants arrays
                _PConstants[0] = gyroRollP;
                _PConstants[1] = gyroPitchP;
                _IConstants[0] = gyroRollI;
                _IConstants[1] = gyroPitchI;
                _DConstants[0] = gyroRollD;
                _DConstants[1] = gyroPitchD;

            }
            
            Rate(float gyroRollPitchP, float gyroRollPitchI, float gyroRollPitchD,
                       float gyroYawP, float gyroYawI, float demandsToRate = 1.0f) :
                _gyroYawP(gyroYawP), 
                _gyroYawI(gyroYawI), 
                _demandsToRate(demandsToRate)
            {
                init();
                // Constants arrays
                _PConstants[0] = gyroRollPitchP;
                _PConstants[1] = gyroRollPitchP;
                _IConstants[0] = gyroRollPitchI;
                _IConstants[1] = gyroRollPitchI;
                _DConstants[0] = gyroRollPitchD;
                _DConstants[1] = gyroRollPitchD;
            }

            bool modifyDemands(state_t & state, demands_t & demands, float currentTime)
            {
                static float lastTime = 0.0;
                
                float deltat = currentTime - lastTime;
                lastTime = currentTime ;
                
                float _demands[3];
                computeReferenceDemands(_demands, state, demands);
                _PTerm[0] = _demands[0];
                _PTerm[1] = _demands[1];

                // Pitch, roll use Euler angles
                demands.roll  = computeCyclicPid(_demands[0],  state.UAVState->angularVelocities, deltat, AXIS_ROLL);
                demands.pitch = computeCyclicPid(_demands[1], state.UAVState->angularVelocities, deltat, AXIS_PITCH);
                
                // For gyroYaw, P term comes directly from RC command or rate setpoint, and D term is zero
                float yawError = _demands[2] * _demandsToRate - state.UAVState->angularVelocities[AXIS_YAW];
                float ITermGyroYaw = computeITermGyro(yawError, _gyroYawI, _demands[2], state.UAVState->angularVelocities, deltat, AXIS_YAW);
                _demands[2] = computePid(_gyroYawP, _demands[2], ITermGyroYaw, 0, state.UAVState->angularVelocities, _offsetError[AXIS_YAW], AXIS_YAW);

                // Prevent "gyroYaw jump" during gyroYaw correction
                demands.yaw = Filter::constrainAbs(_demands[2], 0.1 + fabs(_demands[2]));

                // We've always gotta do this!
                return true;
            }

            void updateReceiver(demands_t & demands, bool throttleIsDown)
            {
                // Compute proportion of cyclic demand compared to its maximum
                // _proportionalCyclicDemand = Filter::max(fabs(demands.roll), fabs(demands.pitch)) / 0.5f;
                
                // When landed, reset integral component of PID
                if (throttleIsDown) {
                    resetIntegral();
                }
            }

    };  // class Rate

} // namespace hf
