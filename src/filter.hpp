/*
   filter.hpp: Static filtering methods

   Adapted from

    https://github.com/multiwii/baseflight/blob/master/src/imu.c

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
   along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cmath>

#include "debug.hpp"
#include "algebra.hpp"

namespace hf {

    class Filter {

        public:

            static float max(float a, float b)
            {
                return a > b ? a : b;
            }

            static float deadband(float value, float deadband)
            {
                if (fabs(value) < deadband) {
                    value = 0;
                } else if (value > 0) {
                    value -= deadband;
                } else if (value < 0) {
                    value += deadband;
                }
                return value;
            }

            static float complementary(float a, float b, float c)
            {
                return a * c + b * (1 - c);
            }

            static float constrainMinMax(float val, float min, float max)
            {
                return (val<min) ? min : ((val>max) ? max : val);
            }

            static float constrainAbs(float val, float max)
            {
                return constrainMinMax(val, -max, +max);
            }

    }; // class Filter

    class KalmanFilter {
      private:
        float currentState[3] = {0, 0, 1};
        float currErrorCovariance[3][3] = {{100, 0, 0},{0, 100, 0},{0, 0, 100}};
        float H[3][3] = {{9.81, 0, 0}, {0, 9.81, 0}, {0, 0, 9.81}};
        float previousAccelSensor[3] = {0, 0, 0};
        float ca;
        float sigmaGyro;
        float sigmaAccel;

        void getPredictionCovariance(float covariance[3][3], float previousState[3],
                                     float deltat)
        {
            // required matrices for the operations
            float sigma[3][3];
            float identity[3][3];
            identityMatrix3x3(identity);
            float skewMatrix[3][3];
            skew(skewMatrix, previousState);
            float tmp[3][3];
            // Compute the prediction covariance matrix
            scaleMatrix3x3(sigma, pow(this->sigmaGyro, 2), identity);
            matrixProduct3x3(tmp, skewMatrix, sigma);
            matrixProduct3x3(covariance, tmp, skewMatrix);
            scaleMatrix3x3(covariance, -pow(deltat, 2), covariance);
        }

        void getMeasurementCovariance(float covariance[3][3])
        {
            // required matrices for the operations
            float sigma[3][3];
            float identity[3][3];
            identityMatrix3x3(identity);
            float tmp[3][3];
            float norm;
            // Compute measurement covariance
            scaleMatrix3x3(sigma, pow(this->sigmaAccel, 2), identity);
            vectorLength(& norm, this->previousAccelSensor);
            scaleAndAccumulateMatrix3x3(sigma, (1.0/3.0)*pow(this->ca, 2)*norm, identity);
            copyMatrix3x3(covariance, sigma);
        }

        void predictState(float predictedState[3], float gyro[3], float deltat)
        {
            // helper matrices
            float identity[3][3];
            identityMatrix3x3(identity);
            float skewFromGyro[3][3];
            skew(skewFromGyro, gyro);
            float tmp[3][3];
            // Predict state
            scaleAndAccumulateMatrix3x3(identity, -deltat, skewFromGyro);
            matrixDotVector3x3(predictedState, identity, this->currentState);
            normalizeVector(predictedState);
        }

        void predictErrorCovariance(float covariance[3][3], float gyro[3], float deltat)
        {
            // required matrices
            float Q[3][3];
            float identity[3][3];
            identityMatrix3x3(identity);
            float skewFromGyro[3][3];
            skew(skewFromGyro, gyro);
            float tmp[3][3];
            float tmpTransposed[3][3];
            float tmp2[3][3];
            // predict error covariance
            getPredictionCovariance(Q, this->currentState, deltat);
            scaleAndAccumulateMatrix3x3(identity, -deltat, skewFromGyro);
            copyMatrix3x3(tmp, identity);
            transposeMatrix3x3(tmpTransposed, tmp);
            matrixProduct3x3(tmp2, tmp, this->currErrorCovariance);
            matrixProduct3x3(covariance, tmp2, tmpTransposed);
            scaleAndAccumulateMatrix3x3(covariance, 1.0, Q);
        }

        void updateGain(float gain[3][3], float errorCovariance[3][3])
        {
            // required matrices
            float R[3][3];
            float HTransposed[3][3];
            transposeMatrix3x3(HTransposed, this->H);
            float tmp[3][3];
            float tmp2[3][3];
            float tmp2Inverse[3][3];
            // update kalman gain
            // P.dot(H.T).dot(inv(H.dot(P).dot(H.T) + R))
            getMeasurementCovariance(R);
            matrixProduct3x3(tmp, errorCovariance, HTransposed);
            matrixProduct3x3(tmp2, this->H, tmp);
            scaleAndAccumulateMatrix3x3(tmp2, 1.0, R);
            invert3x3(tmp2Inverse, tmp2);
            matrixProduct3x3(gain, tmp, tmp2Inverse);
        }

        void updateState(float updatedState[3], float predictedState[3], float gain[3][3], float accel[3])
        {
            // required matrices
            float tmp[3];
            float tmp2[3];
            float measurement[3];
            scaleVector(tmp, this->ca, this->previousAccelSensor);
            subtractVectors(measurement, accel, tmp);
            // update state with measurement
            // predicted_state + K.dot(measurement - H.dot(predicted_state))
            matrixDotVector3x3(tmp, this->H, predictedState);
            subtractVectors(tmp, measurement, tmp);
            matrixDotVector3x3(tmp2, gain, tmp);
            sumVectors(updatedState, predictedState, tmp2);
            normalizeVector(updatedState);
        }

        void updateErrorCovariance(float covariance[3][3], float errorCovariance[3][3], float gain[3][3])
        {
            // required matrices
            float identity[3][3];
            identityMatrix3x3(identity);
            float tmp[3][3];
            float tmp2[3][3];
            // update error covariance with measurement
            matrixProduct3x3(tmp, gain, this->H);
            matrixProduct3x3(tmp2, tmp, errorCovariance);
            scaleAndAccumulateMatrix3x3(identity, -1.0, tmp2);
            copyMatrix3x3(covariance, tmp2);
        }

      public:

        KalmanFilter(float ca, float sigmaGyro, float sigmaAccel)
        {
            this->ca = ca;
            this->sigmaGyro = sigmaGyro;
            this->sigmaAccel = sigmaAccel;
        }

        float estimate(float gyro[3], float accel[3], float deltat)
        {
          float predictedState[3];
          float updatedState[3];
          float errorCovariance[3][3];
          float updatedErrorCovariance[3][3];
          float gain[3][3];
          float accelSensor[3];
          float tmp[3];
          float accelEarth;
          scaleVector(accel, 9.81, accel); // Scale accel readings since they are measured in gs
          // perform estimation
          predictState(predictedState, gyro, deltat);
          predictErrorCovariance(errorCovariance, gyro, deltat);
          updateGain(gain, errorCovariance);
          // The above has been tested
          updateState(updatedState, predictedState, gain, accel);
          updateErrorCovariance(updatedErrorCovariance, errorCovariance, gain);
          // Store required values for next iteration
          copyVector(this->currentState, updatedState);
          copyMatrix3x3(this->currErrorCovariance, updatedErrorCovariance);
          // return vertical acceleration estimate
          scaleVector(tmp, 9.81, updatedState);
          subtractVectors(accelSensor, accel, tmp);
          copyVector(this->previousAccelSensor, accelSensor);
          dotProductVectors(& accelEarth, accelSensor, updatedState);
          return accelEarth;
        }

    }; // Class KalmanFilter

    class ComplementaryFilter {

      private:

        // filter gain
        float gain[2];
        // Zero-velocity update
        float accelThreshold;
        static const uint8_t ZUPT_SIZE = 12;
        uint8_t ZUPTIdx;
        float   ZUPT[ZUPT_SIZE];

        float ApplyZUPT(float accel, float vel)
        {
            // first update ZUPT array with latest estimation
            ZUPT[ZUPTIdx] = accel;
            // and move index to next slot
            uint8_t nextIndex = (ZUPTIdx + 1) % ZUPT_SIZE;
            ZUPTIdx = nextIndex;
            // Apply Zero-velocity update
            for (uint8_t k = 0; k < ZUPT_SIZE; ++k) {
                if (ZUPT[k] > this->accelThreshold) return 0.0;
            }
            return vel;
        }


      public:

        ComplementaryFilter(float sigmaAccel, float sigmaBaro, float accelThreshold)
        {
            // Compute the filter gain
            this->gain[0] = sqrt(2 * sigmaAccel / sigmaBaro);
            this->gain[1] = sigmaAccel / sigmaBaro;
            // If acceleration is below the threshold the ZUPT counter
            // will be increased
            this->accelThreshold = accelThreshold;
            // initialize zero-velocity update
            ZUPTIdx = 0;
            for (uint8_t k = 0; k < ZUPT_SIZE; ++k) {
                ZUPT[k] = 0;
            }
        }

        void estimate(float * velocity, float * altitude, float baroAltitude, float accel, float deltat)
        {
            // Apply complementary filter
            *altitude = *altitude + deltat*((*velocity) + (this->gain[0] + this->gain[1]*deltat/2)*(baroAltitude-(*altitude)))+
             accel*pow(deltat, 2)/2;
            *velocity = *velocity + deltat*(this->gain[1]*(baroAltitude-(*altitude)) + accel);
            // Compute zero-velocity update
            *velocity = ApplyZUPT(accel, *velocity);
        }

    }; // Class ComplementaryFilter

} // namespace hf
