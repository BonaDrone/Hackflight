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

        void getPredictionCovariance(float covariance[3][3], float state_prev[3],
                                     float deltat, float sigma_gyro)
        {
            // required matrices for the operations
            float sigma[3][3];
            float identity[3][3];
            identify_matrix_3x3(identity);
            float skew_matrix[3][3];
            skew(skew_matrix, state_prev);
            float tmp[3][3];
            // Compute the prediction covariance matrix
            scale_matrix_3x3(sigma, pow(sigma_gyro, 2), identity);
            matrix_product_3x3(tmp, skew_matrix, sigma);
            matrix_product_3x3(covariance, tmp, skew_matrix);
            scale_matrix_3x3(covariance, -pow(deltat, 2), covariance);
        }

        void getMeasurementCovariance(float covariance[3][3], float ca,
                                      float sigma_accel, float a_sensor_prev[3])
        {
            // required matrices for the operations
            float sigma[3][3];
            float identity[3][3];
            identify_matrix_3x3(identity);
            float tmp[3][3];
            float norm;
            // Compute measurement covariance
            scale_matrix_3x3(sigma, pow(sigma_accel, 2), identity);
            vec_length(norm, a_sensor_prev);
            accum_scale_matrix_3x3(sigma, (1.0/3.0)*pow(ca, 2)*norm, identity);
            copy_matrix_3x3(covariance, sigma);
        }

        float predictState(float predictedState[3], float state[3],
                           float gyro[3], float deltat)
        {
            // helper matrices
            float identity[3][3];
            identify_matrix_3x3(identity);
            float skew_from_gyro[3][3];
            skew(skew_from_gyro, gyro);
            float tmp[3][3];
            // Predict state
            accum_scale_matrix_3x3(identity, -deltat, skew_from_gyro);
            mat_dot_vec_3x3(predictedState, identity, state);
            vec_normalize(predictedState);
        }

        void predictErrorCovariance(float covariance[3][3], float state[3], float gyro[3],
                                    float deltat, float errorCovariance[3][3], float sigma_gyro )
        {
            // required matrices
            float Q[3][3];
            float identity[3][3];
            identify_matrix_3x3(identity);
            float skew_from_gyro[3][3];
            skew(skew_from_gyro, gyro);
            float tmp[3][3];
            float tmp_trans[3][3];
            float tmp2[3][3];
            // predict error covariance
            getPredictionCovariance(Q, state, deltat, sigma_gyro);
            accum_scale_matrix_3x3(identity, -deltat, skew_from_gyro);
            copy_matrix_3x3(tmp, identity);
            transpose_matrix_3x3(tmp_trans, tmp);
            matrix_product_3x3(tmp2, tmp, errorCovariance);
            matrix_product_3x3(covariance, tmp2, tmp_trans);
            accum_scale_matrix_3x3(covariance, 1.0, Q);
        }

        void updateGain(float gain[3][3],float errorCovariance[3][3], float H[3][3],
                        float ca, float a_sensor_prev[3], float sigma_accel)
        {
            // required matrices
            float R[3][3];
            float H_trans[3][3];
            transpose_matrix_3x3(H_trans, H);
            float tmp[3][3];
            float tmp2[3][3];
            float tmp2_inv[3][3];
            // update kalman gain
            // P.dot(H.T).dot(inv(H.dot(P).dot(H.T) + R))
            getMeasurementCovariance(R, ca, sigma_accel, a_sensor_prev);
            matrix_product_3x3(tmp, errorCovariance, H_trans);
            matrix_product_3x3(tmp2, H, tmp);
            accum_scale_matrix_3x3(tmp2, 1.0, R);
            invert_3X3(tmp2_inv, tmp2);
            matrix_product_3x3(gain, tmp, tmp2_inv);
        }

        float updateState(float updatedState[3], float predictedState[3], float gain[3][3],
                          float measurement[3], float H[3][3])
        {
            // required matrices
            float tmp[3];
            float tmp2[3];
            // update state with measurement
            // predicted_state + K.dot(measurement - H.dot(predicted_state))
            mat_dot_vec_3x3(tmp, H, predictedState);
            vec_diff(tmp, measurement, tmp);
            mat_dot_vec_3x3(tmp2, gain, tmp);
            vec_sum(updatedState, predictedState, tmp2);
            vec_normalize(updatedState);
        }

        void updateErrorCovariance(float covariance[3][3], float errorCovariance[3][3],
                                   float H[3][3], float gain[3][3])
        {
            // required matrices
            float identity[3][3];
            identify_matrix_3x3(identity);
            float tmp[3][3];
            float tmp2[2][2];
            // update error covariance with measurement
            matrix_product_3x3(tmp, gain, H);
            matrix_product_3x3(tmp2, tmp, errorCovariance);
            accum_scale_matrix_3x3(identity, -1.0, tmp2);
            copy_matrix_3x3(covariance, tmp2);
        }

      public:

        KalmanFilter()
        {

        }

        float estimate()
        {

        }

        float getVerticalAcceleration()
        {
          
        }

    }; // Class KalmanFilter

    class ComplementaryFilter {

      private:

        // filter gain
        float gain[2];
        // Zero-velocity update
        float accel_threshold;
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
                if (ZUPT[k] > accel_threshold) return 0.0;
            }
            return vel;
        }


      public:

        ComplementaryFilter(float sigma_accel, float sigma_baro, float accel_threshold)
        {
            // Compute the filter gain
            gain[0] = sqrt(2 * sigma_accel / sigma_baro);
            gain[1] = sigma_accel / sigma_baro;
            // If acceleration is below the threshold the ZUPT counter
            // will be increased
            accel_threshold = accel_threshold;
            // initialize zero-velocity update
            ZUPTIdx = 0;
            for (uint8_t k = 0; k < ZUPT_SIZE; ++k) {
                ZUPT[k] = 0;
            }
        }

        void estimate(float * velocity, float * altitude, float baro_altitude, float accel, float deltat)
        {
            // Apply complementary filter
            *altitude = *altitude + deltat*((*velocity) + (gain[0] + gain[1]*deltat/2)*(baro_altitude-(*altitude)))+
             accel*pow(deltat, 2)/2;
            *velocity = *velocity + deltat*(gain[1]*(baro_altitude-(*altitude)) + accel);
            // Compute zero-velocity update
            *velocity = ApplyZUPT(accel, *velocity);
        }

    }; // Class ComplementaryFilter

} // namespace hf
