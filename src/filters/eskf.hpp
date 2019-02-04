/*
   eskf.hpp : Implementation of Error-state Kalman Filter for state estimation

   Copyright (c) 2019 BonaDrone (www.bonadrone.com)
   Developed by: Pep Marti-Saumell (jmarti<at>bonadrone.com>) & Juan Gallostra Acin (jgallostra<at>bonadrone.com)

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

#include "linalg.hpp"
#include "eskf_struct.hpp"
#include "eskf_sensor.hpp"

namespace hf {
  
  class ESKF {
    friend class Hackflight;
    
    protected:
      ESKF_Sensor * sensors[256];
      int sensor_count = 0;

    private:

      static const uint8_t errorStates = NEsta;
      static const uint8_t nominalStates = NNsta;
      static const uint8_t observations = Mobs;

      eskf_t eskf;
      eskf_p_t eskfp;

      double t_lastCall;
      double dt;
      
      void eskfp_init(void * eskf, int nn, int ne, int m)
      {
          float * dptr = (float *)eskf;
          eskfp.x = dptr;
          dptr += nn;
          eskfp.dx = dptr;
          dptr += ne;
          eskfp.qL = dptr;
          dptr += nn*nn;
          
          eskfp.P = dptr;
          dptr += ne*ne;
          eskfp.Q = dptr;
          dptr += ne*ne;
          eskfp.R = dptr;
          dptr += m*m;
          
          eskfp.K = dptr;
          dptr += ne*m;
          eskfp.Kt = dptr;
          dptr += m*ne;
          
          eskfp.Fdx = dptr;
          dptr += ne*ne;
          eskfp.H = dptr;
          dptr += m*ne;
          
          eskfp.Ht = dptr;
          dptr += ne*m;
          eskfp.Fdxt = dptr;
          dptr += ne*ne;
          
          eskfp.G = dptr;
          dptr += ne*ne;
          
          eskfp.fx = dptr;
          dptr += nn;
          eskfp.hx = dptr;
          dptr += m;
          
          eskfp.tmp0 = dptr;
          dptr += ne*ne;
          eskfp.tmp1 = dptr;
          dptr += ne*m;
          eskfp.tmp2 = dptr;
          dptr += m*ne;
          eskfp.tmp3 = dptr;
          dptr += m*m;
          eskfp.tmp4 = dptr;
          dptr += m*m;
          eskfp.tmp5 = dptr;
          dptr += nn;
          eskfp.tmp6 = dptr;
          dptr += ne*ne;
          eskfp.tmp7 = dptr;
          dptr += ne*ne;
          eskfp.tmp8 = dptr;
      }
      
      void synchState(void)
      {
          // Update euler angles
          float q[4] = {eskf.x[6], eskf.x[7], eskf.x[8], eskf.x[9]};
          Quaternion::computeEulerAngles(q, state.eulerAngles);
          // Convert heading from [-pi,+pi] to [0,2*pi]
          if (state.eulerAngles[2] < 0) {
              state.eulerAngles[2] += 2*M_PI;
          }
          // update vertical position and velocity
          state.position[0] = eskf.x[0];
          state.position[1] = eskf.x[1];
          state.position[2] = eskf.x[2];
          state.linearVelocities[0] = eskf.x[3];
          state.linearVelocities[1] = eskf.x[4];
          state.linearVelocities[2] = eskf.x[5];
      }

    public:

      ESKF(){}

      eskf_state_t state;

      void init(void)
      {
          eskfp_init(&eskf, nominalStates, errorStates, observations);
          /* zero-out matrices */
          zeros(eskfp.qL, nominalStates, nominalStates);
          zeros(eskfp.P, errorStates, errorStates);
          zeros(eskfp.Q, errorStates, errorStates);
          zeros(eskfp.R, observations, observations);
          zeros(eskfp.K, errorStates, observations);
          zeros(eskfp.Fdx, errorStates, errorStates);
          zeros(eskfp.H, observations, errorStates);
          
          // initial state
          eskfp.x[0] = 0.0; // position
          eskfp.x[1] = 0.0; 
          eskfp.x[2] = 0.0;
          eskfp.x[3] = 0.0; // velocity
          eskfp.x[4] = 0.0;
          eskfp.x[5] = 0.0;
          eskfp.x[6] = 1.0; // orientation
          eskfp.x[7] = 0.0;
          eskfp.x[8] = 0.0;
          eskfp.x[9] = 0.0;
          eskfp.x[10] = 0.0; // accel bias
          eskfp.x[11] = 0.0;
          eskfp.x[12] = 0.0;
          eskfp.x[13] = 0.301350; // gyro bias
          eskfp.x[14] = -0.818594;
          eskfp.x[15] = -0.701652;
          
          // Since P has already been zero-ed only elements != 0 have to be set
          // 1 column
          eskfp.P[0] = 0.0;
          // 2 column
          eskfp.P[16] = 0.0;
          // 3 column
          eskfp.P[32] = 0.0;
          // 4 column
          eskfp.P[48] = 0.0;
          // 5 column
          eskfp.P[64] = 0.0;
          // 6 column
          eskfp.P[80] = 0.0;
          // 7 column
          eskfp.P[96] = 0.01;
          // 8 column
          eskfp.P[112] = 0.01;
          // 9 column
          eskfp.P[128] = 0.0;
          // 10 column
          eskfp.P[144] = 0.000001;
          // 11 column
          eskfp.P[160] = 0.000001;
          // 12 column
          eskfp.P[176] = 0.000001;
          // 13 column
          eskfp.P[192] = 0.000001;
          // 14 column
          eskfp.P[208] = 0.000001;
          // 15 column
          eskfp.P[224] = 0.000001;
      }

      void addSensorESKF(ESKF_Sensor * sensor)
      {
          sensors[sensor_count++] = sensor;
      }
      
      int update(ESKF_Sensor * sensor, float time) 
      {
          /*
          This method should:
            1. Obtain the the error-state Jacobian.
            2. Update the nominal state estimate by integrating it
            3. From the error-state Jacobian, the process noise and the past 
               iteration covariance estimate the current Covariance (and enforce
               its symmetry?)
          */
                          
          // Check sensor
          if (sensor->ready(time)) {
              // Update state with gyro rates
              sensor->modifyState(state, time);                    
          } 
          
          // Compute deltat
          double t_now = (double)micros();
          dt = (t_now - t_lastCall)/1000000.0f;
          t_lastCall = t_now;
          
          sensor->integrateNominalState(eskfp.fx, eskfp.x, dt);
          sensor->getJacobianErrors(eskfp.Fdx, eskfp.x, dt);
          sensor->getCovarianceEstimation(eskfp.Q);

          // Normalize quaternion
          float quat_tmp[4] = { eskf.fx[6], eskf.fx[7], eskf.fx[8], eskf.fx[9] };
          float norm_quat_tmp[4];
          norvec(quat_tmp, norm_quat_tmp, 4);
          eskf.fx[6] = norm_quat_tmp[0];
          eskf.fx[7] = norm_quat_tmp[1];
          eskf.fx[8] = norm_quat_tmp[2];
          eskf.fx[9] = norm_quat_tmp[3];
          
          // Copy back estimated states into x
          copyvec(eskfp.fx, eskfp.x, nominalStates);
          
          // Predict covariance
          /* P_k = Fdx_{k-1} P_{k-1} Fdx^T_{k-1} + Q_{k-1} */
          transpose(eskfp.Fdx, eskfp.Fdxt, errorStates, errorStates);
          mulmat(eskfp.Fdx, eskfp.P, eskfp.tmp0, errorStates, errorStates, errorStates);
          mulmat(eskfp.tmp0, eskfp.Fdxt, eskfp.tmp6, errorStates, errorStates, errorStates);
          accum(eskfp.tmp6, eskfp.Q, errorStates, errorStates);
          makesym(eskfp.tmp6, eskfp.P, errorStates);

          Serial.println("P");
          printMatrix(eskfp.P, errorStates, errorStates);

          Serial.print(eskf.x[0], 8);
          Serial.print(",");
          Serial.print(eskf.x[3], 8);
          Serial.print(",");
          Serial.println(eskf.x[2], 8);
          
          Serial.println("Update");

          /* success */
          synchState();
          return 0;

      } // update
      
      int correct(ESKF_Sensor * sensor, float time) 
      {
          /* This method should:
            1. Obtain the Jacobian of the correction measurement model
            2. Obtain the innovation value:
               innovation = measurement - measurement prediction for the estimated state
            3. Obtain the measurement noise
            4. Compute the gain
            5. Estimate the error-states 
            6. Update Covariance
            7. Inject errors
            8. Update Covariance if required and enforce symmetry
            9. Reset errors
          */
          
          // zero used matrices
          // This is required because not all sensors have the same number of 
          // observations and matrices are dimensioned so that they can store the
          // max number of observations. When correcting states with a sensor that
          // has less observations we don't want residual values from previous
          // calculations to affect the current correction.
          zeroCorrectMatrices();

          bool JacobianOk = sensor->getJacobianObservation(eskfp.H, eskfp.x);
          bool InnovationOk = sensor->getInnovation(eskfp.hx, eskfp.x);
          sensor->getCovarianceCorrection(eskfp.R);
          
          if (!JacobianOk || !InnovationOk)
          {
            return 1;
          }
          Serial.println("H");
          printMatrix(eskfp.H, observations, errorStates);

          // Compute gain:
          /* K_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1} */
          transpose(eskfp.H, eskfp.Ht, observations, errorStates);
          mulmat(eskfp.P, eskfp.Ht, eskfp.tmp1, errorStates, errorStates, observations); // P*H'
          mulmat(eskfp.H, eskfp.P, eskfp.tmp2, observations, errorStates, errorStates);  // H*P
          mulmat(eskfp.tmp2, eskfp.Ht, eskfp.tmp3, observations, errorStates, observations); // H*P*H'
          accum(eskfp.tmp3, eskfp.R, observations, observations);                 // Z = H*P*H' + R
          
          Serial.println("Z");
          printMatrix(eskfp.tmp3, observations, observations);

          
          // if (cholsl(eskfp.tmp3, eskfp.tmp4, eskfp.tmp5, observations)) return 1; // tmp4 = Z^-1
          if (sensor->Zinverse(eskfp.tmp3, eskfp.tmp4)) return 1; // tmp4 = Z^-1
          mulmat(eskfp.tmp1, eskfp.tmp4, eskfp.K, errorStates, observations, observations); // K = P*H'*Z^-1
          
          Serial.println("K");
          printMatrix(eskfp.K, errorStates, observations);

          /* \hat{x}_k = \hat{x_k} + K_k(z_k - h(\hat{x}_k)) */
          mulvec(eskfp.K, eskfp.hx, eskfp.dx, errorStates, observations);
          
          /* P_k = P_k - K_k Z_k K^T_k  */
          //transpose(eskfp.K, eskfp.Kt, observations, errorStates);
          //mulmat(eskfp.K, eskfp.tmp3, eskfp.tmp0, errorStates, observations, errorStates);
          //mulmat(eskfp.tmp0, eskfp.Kt, eskfp.tmp3, errorStates, errorStates, observations);
          //sub(eskfp.Pp, eskfp.tmp3, eskfp.tmp0, errorStates);
          //makesym(eskfp.tmp0, eskfp.P, errorStates);
          
          /* P = (I-KH)*P*(I-KH)' + KZK' */
          mulmat(eskfp.K, eskfp.H, eskfp.tmp6, errorStates, observations, errorStates); // K*H
          negate(eskfp.tmp6, errorStates, errorStates); // -K*H
          mat_addeye(eskfp.tmp6, errorStates); // -K*H + I
          transpose(eskfp.tmp6, eskfp.tmp7, errorStates, errorStates); // (-K*H + I)'
          mulmat(eskfp.tmp6, eskfp.P, eskfp.tmp8, errorStates, errorStates, errorStates); // (-K*H + I)*P
          mulmat(eskfp.tmp8, eskfp.tmp7, eskfp.P, errorStates, errorStates, errorStates); // (-K*H + I)*P*(-K*H + I)'
          // Z is stored in eskfp.tmp3 and K in eskfp.K
          transpose(eskfp.K, eskfp.Kt, errorStates, observations); // K'
          mulmat(eskfp.tmp3, eskfp.Kt, eskfp.tmp2, observations, observations, errorStates); // Z*K'
          mulmat(eskfp.K, eskfp.tmp2, eskfp.tmp0, errorStates, observations, errorStates); // K*Z*K'
          accum(eskfp.P, eskfp.tmp0, errorStates, errorStates); 
          
          Serial.println("P");
          printMatrix(eskfp.P, errorStates, errorStates);
          
          /* Error injection */
          // XXX Quaternion injection as a method
          float tmp[4];
          tmp[0] = 1.0;
          tmp[1] = eskfp.dx[6]/2.0;
          tmp[2] = eskfp.dx[7]/2.0;
          tmp[3] = eskfp.dx[8]/2.0;
          float quat_tmp[4] = {eskfp.x[6], eskfp.x[7], eskfp.x[8], eskfp.x[9]}; 
          Quaternion::computeqL(eskfp.qL, quat_tmp);
          mulvec(eskfp.qL, tmp, eskfp.tmp5, 4, 4);
          norvec(eskfp.tmp5, tmp, 4);
          eskf.x[6] = tmp[0];
          eskf.x[7] = tmp[1];
          eskf.x[8] = tmp[2];
          eskf.x[9] = tmp[3];
          // Inject rest of errors
          eskfp.x[0] += eskfp.dx[0]; // position
          eskfp.x[1] += eskfp.dx[1];
          eskfp.x[2] += eskfp.dx[2];
          eskfp.x[3] += eskfp.dx[3]; // velocity
          eskfp.x[4] += eskfp.dx[4];
          eskfp.x[5] += eskfp.dx[5];
          eskfp.x[10] += eskfp.dx[9]; // accel bias
          eskfp.x[11] += eskfp.dx[10];
          eskfp.x[12] += eskfp.dx[11];
          eskfp.x[13] += eskfp.dx[12]; // gyro bias
          eskfp.x[14] += eskfp.dx[13];
          eskfp.x[15] += eskfp.dx[14];

          eskfp.x[15] = 0.00; // Brute force yaw bias to 0

          /* Update covariance*/
          /*eskfp.tmp5[0] = eskfp.dx[0]/2.0;
          eskfp.tmp5[1] = eskfp.dx[1]/2.0;
          eskfp.tmp5[2] = eskfp.dx[2]/2.0;
          newSkew(eskfp.tmp5, eskfp.G);
          negate(eskfp.G, errorStates, errorStates);
          mat_addeye(eskfp.G, errorStates);
          transpose(eskfp.G, eskfp.tmp0, errorStates, errorStates);
          mulmat(eskfp.Pp, eskfp.tmp0, eskfp.tmp9, errorStates, errorStates, errorStates);
          mulmat(eskfp.G, eskfp.tmp9, eskfp.tmp10, errorStates, errorStates, errorStates);
          
          // Force its symmetry: P = (P + P')/2
          makesym(eskfp.tmp10, eskfp.P, errorStates);*/
          
          /* reset error state */
          zeros(eskfp.dx, errorStates, 1);
          /* success */
          synchState();

          Serial.print(eskf.x[0], 8);
          Serial.print(",");
          Serial.print(eskf.x[3], 8);
          Serial.print(",");
          Serial.println(eskf.x[2], 8);
                    
          return 0;
      } // correct

      void zeroCorrectMatrices(void)
      {
          zeros(eskfp.H, observations, errorStates);
          zeros(eskfp.Ht, errorStates, observations);
          zeros(eskfp.K, errorStates, observations);
          zeros(eskfp.Kt, observations, errorStates);
          zeros(eskfp.hx, observations, 1);
          zeros(eskfp.R, observations, observations);          
          
          zeros(eskfp.tmp0, errorStates, errorStates);
          zeros(eskfp.tmp1, errorStates, observations);
          zeros(eskfp.tmp2, observations, errorStates);
          zeros(eskfp.tmp3, observations, observations);
          zeros(eskfp.tmp4, observations, observations);
          zeros(eskfp.tmp5, nominalStates, 1);
          zeros(eskfp.tmp6, errorStates, errorStates);
          zeros(eskfp.tmp7, errorStates, errorStates);
          zeros(eskfp.tmp8, errorStates, errorStates);
      } // zeroCorrectMatrices

      // XXX Debug
      void printMatrix(float * M, int r, int c)
      {
          for (int ii=0; ii<r; ++ii)
          {
            for (int jj=0; jj<c; ++jj)
            {
              if (jj == c-1)
              {
                Serial.println(M[ii*c+jj],8);
              }
              else
              {
                Serial.print(M[ii*c+jj],8);
                Serial.print(",");
              }
              
            }
          }
      }

  }; // class ESKF
  
} // namespace hf
