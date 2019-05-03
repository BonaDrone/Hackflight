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
      uint8_t sensor_count = 0;

    private:

      // NEsta, NNsta, Mobs are defined in eskf_struct.hpp
      static const uint8_t errorStates = NEsta;
      static const uint8_t nominalStates = NNsta;
      static const uint8_t observations = Mobs;
      static const uint8_t mnQuat = MNQuat;
      
      // Velocity low pass filters
      static const uint8_t HISTORY = 50;
      hf::LowPassFilter _lpVelX = hf::LowPassFilter(HISTORY);
      hf::LowPassFilter _lpVelY = hf::LowPassFilter(HISTORY);

      eskf_t eskf;
      eskf_p_t eskfp;

      double t_lastCall;
      double dt;

      void eskfp_init(void * eskf, int nn, int ne, int m, int mnquat)
      {
          float * dptr = (float *)eskf;
          eskfp.x = dptr;
          dptr += nn;
          eskfp.dx = dptr;
          dptr += ne;
          eskfp.qL = dptr;
          dptr += mnquat*mnquat;

          eskfp.P = dptr;
          dptr += ne*ne;
          eskfp.Q = dptr;
          dptr += ne*ne;
          eskfp.R = dptr;
          dptr += m*m;

          eskfp.K = dptr;
          dptr += ne*m;

          eskfp.Fdx = dptr;
          dptr += ne*ne;
          eskfp.H = dptr;
          dptr += m*ne;

          // eskfp.G = dptr;
          // dptr += ne*ne;

          eskfp.fx = dptr;
          dptr += nn;
          eskfp.hx = dptr;

      }

      void synchState()
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

          // Transform linear velocities to IMU frame
          float world_vels[3] = {eskf.x[3], eskf.x[4], eskf.x[5]};
          float vels[3];
          velocityToIMUFrame(vels, world_vels, q);

          state.linearVelocities[0] = _lpVelX.update(vels[0]);
          state.linearVelocities[1] = _lpVelY.update(vels[1]);
          state.linearVelocities[2] = vels[2];

      }

      void covariancePrediction(float * Fdx, float * P, float * Q)
      {
          float tmp0[errorStates*errorStates] = {};
          float tmp6[errorStates*errorStates] = {};
          float Fdxt[errorStates*errorStates] = {};
          /* P_k = Fdx_{k-1} P_{k-1} Fdx^T_{k-1} + Q_{k-1} */
          transpose(Fdx, Fdxt, errorStates, errorStates);
          mulmat(Fdx, P, tmp0, errorStates, errorStates, errorStates);
          mulmat(tmp0, Fdxt, tmp6, errorStates, errorStates, errorStates);
          accum(tmp6, Q, errorStates, errorStates);

          makesym(tmp6, P, errorStates);
      }
      
      int computeGain(float * P, float * H, float * R, float * K, ESKF_Sensor * sensor)
      {
          float tmp1[errorStates*observations] = {};
          float tmp2[observations*errorStates] = {};
          float tmp3[observations*observations] = {};
          float tmp4[observations*observations] = {};
          float tmp9[observations*observations] = {};
          float Ht[errorStates*observations] = {};
          
          transpose(H, Ht, observations, errorStates);
          mulmat(P, Ht, tmp1, errorStates, errorStates, observations); // P*H'
          mulmat(H, P, tmp2, observations, errorStates, errorStates);  // H*P
          mulmat(tmp2, Ht, tmp3, observations, errorStates, observations); // H*P*H'
          accum(tmp3, R, observations, observations);                 // Z = H*P*H' + R
          
          makesym(tmp3, tmp9, observations);
          
          // if (cholsl(eskfp.tmp3, eskfp.tmp4, eskfp.tmp5, observations)) return 1; // tmp4 = Z^-1
          if (sensor->Zinverse(tmp9, tmp4)) return 1; // tmp4 = Z^-1
          
          mulmat(tmp1, tmp4, K, errorStates, observations, observations); // K = P*H'*Z^-1

          // Do not let optical flow correct height and vertical velocity
          if (sensor->isOpticalFlow())
          {
            K[2] = 0;
            K[5] = 0;
          }

          return 0;
      }
      
      void covarianceUpdate(float * K, float * H, float * R, float * P)
      {
          float tmp0[errorStates*errorStates] = {};
          float tmp2[observations*errorStates] = {};
          float tmp6[errorStates*errorStates] = {};
          float tmp7[errorStates*errorStates] = {};
          float tmp8[errorStates*errorStates] = {};
          float tmp9[errorStates*errorStates] = {};
          float Kt[observations*errorStates] = {};
          
          /* P = (I-KH)*P*(I-KH)' + KRK' */
          mulmat(K, H, tmp6, errorStates, observations, errorStates); // K*H
          negate(tmp6, errorStates, errorStates); // -K*H
          mat_addeye(tmp6, errorStates); // -K*H + I
          transpose(tmp6, tmp7, errorStates, errorStates); // (-K*H + I)'
          mulmat(tmp6, P, tmp9, errorStates, errorStates, errorStates); // (-K*H + I)*P
          mulmat(tmp9, tmp7, tmp6, errorStates, errorStates, errorStates); // (-K*H + I)*P*(-K*H + I)'
          transpose(K, Kt, errorStates, observations); // K'
          mulmat(R, Kt, tmp2, observations, observations, errorStates); // R*K'
          mulmat(K, tmp2, tmp0, errorStates, observations, errorStates); // K*R*K'
          accum(tmp6, tmp0, errorStates, errorStates);
          
          makesym(tmp6, P, errorStates);
      }

    public:

      ESKF(){}

      eskf_state_t state;

      void init(void)
      {
          eskfp_init(&eskf, nominalStates, errorStates, observations, mnQuat);
          reset();
      }

      void reset(void)
      {
        /* zero-out matrices */
        zeros(eskfp.qL, mnQuat, mnQuat);
        zeros(eskfp.P, errorStates, errorStates);
        zeros(eskfp.Q, errorStates, errorStates);
        zeros(eskfp.R, observations, observations);
        zeros(eskfp.K, errorStates, observations);
        zeros(eskfp.Fdx, errorStates, errorStates);
        zeros(eskfp.H, observations, errorStates);

        // intialize lpfs
        _lpVelX.init();
        _lpVelY.init();

        // initial state
        eskfp.x[0] = 0.0; // position
        eskfp.x[1] = 0.0;
        eskfp.x[2] = 0.0;
        eskfp.x[3] = 0.0; // velocity
        eskfp.x[4] = 0.0;
        eskfp.x[5] = 0.0;
        eskfp.x[6] = 1.0; // orientation (quaternion)
        eskfp.x[7] = 0.0;
        eskfp.x[8] = 0.0;
        eskfp.x[9] = 0.0;

        // Since P has already been zero-ed only elements != 0 have to be set
        // 1 column
        eskfp.P[0] = 0.0;
        // 2 column
        eskfp.P[10] = 0.0;
        // 3 column
        eskfp.P[20] = 0.0;
        // 4 column
        eskfp.P[30] = 0.0;
        // 5 column
        eskfp.P[40] = 0.0;
        // 6 column
        eskfp.P[50] = 0.0;
        // 7 column
        eskfp.P[60] = 0.01;
        // 8 column
        eskfp.P[70] = 0.01;
        // 9 column
        eskfp.P[80] = 0.0;
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
          covariancePrediction(eskfp.Fdx, eskfp.P, eskfp.Q);

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

          // Make all the entries of the used matrices zero
          // This is required because not all sensors have the same number of
          // observations and matrices are dimensioned so that they can store the
          // max number of observations. When correcting states with a sensor that
          // has less observations we don't want residual values from previous
          // calculations to affect the current correction.
          zeroCorrectMatrices();

          bool JacobianOk = sensor->getJacobianObservation(eskfp.H, eskfp.x);
          bool InnovationOk = sensor->getInnovation(eskfp.hx, eskfp.x);
          sensor->getCovarianceCorrection(eskfp.R);

          // Skip correction if there were any errors when obtaining
          // the Jacobian or the innovation
          if (!JacobianOk || !InnovationOk)
          {
              return 1;
          }

          // Compute gain:
          /* K_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1} */
          if (computeGain(eskfp.P, eskfp.H, eskfp.R, eskfp.K, sensor)) return 1;

          /* \hat{x}_k = \hat{x_k} + K_k(z_k - h(\hat{x}_k)) */
          mulvec(eskfp.K, eskfp.hx, eskfp.dx, errorStates, observations);

          // /* P_k = P_k - K_k Z_k K^T_k  */
          // //transpose(eskfp.K, eskfp.Kt, observations, errorStates);
          // //mulmat(eskfp.K, eskfp.tmp3, eskfp.tmp0, errorStates, observations, errorStates);
          // //mulmat(eskfp.tmp0, eskfp.Kt, eskfp.tmp3, errorStates, errorStates, observations);
          // //sub(eskfp.Pp, eskfp.tmp3, eskfp.tmp0, errorStates);
          // //makesym(eskfp.tmp0, eskfp.P, errorStates);
          // 
          /* P = (I-KH)*P*(I-KH)' + KRK' */
          covarianceUpdate(eskfp.K, eskfp.H, eskfp.R, eskfp.P);

          /* Error injection */
          // XXX Quaternion injection as a method
          float tmp[4];
          float tmp2[4];
          tmp[0] = 1.0;
          tmp[1] = eskf.dx[6]/2.0;
          tmp[2] = eskf.dx[7]/2.0;
          tmp[3] = eskf.dx[8]/2.0;
          float quat_tmp[4] = {eskf.x[6], eskf.x[7], eskf.x[8], eskf.x[9]}; 
          Quaternion::computeqL(eskfp.qL, quat_tmp);
          mulvec(eskfp.qL, tmp, tmp2, 4, 4);
          norvec(tmp2, tmp, 4);
          eskf.x[6] = tmp[0];
          eskf.x[7] = tmp[1];
          eskf.x[8] = tmp[2];
          eskf.x[9] = tmp[3];
          // Inject rest of errors
          eskf.x[0] += eskf.dx[0]; // position
          eskf.x[1] += eskf.dx[1];
          eskf.x[2] += eskf.dx[2];
          eskf.x[3] += eskf.dx[3]; // velocity
          eskf.x[4] += eskf.dx[4];
          eskf.x[5] += eskf.dx[5];

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

          return 0;
      } // correct

      void zeroCorrectMatrices(void)
      {
          zeros(eskfp.H, observations, errorStates);
          zeros(eskfp.K, errorStates, observations);
          zeros(eskfp.hx, observations, 1);
          zeros(eskfp.R, observations, observations);
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

      void velocityToIMUFrame(float vels[3], float world_vels[3], float q[4])
      {
          float R[9]; // Rotation matrix
          // Compute rotation matrix from quaternion
          R[0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
          R[1] = 2*q[1]*q[2] - 2*q[0]*q[3];
          R[2] = 2*q[1]*q[3] + 2*q[0]*q[2];
          R[3] = 2*q[1]*q[2] + 2*q[0]*q[3];
          R[4] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
          R[5] = 2*q[2]*q[3] - 2*q[0]*q[1];
          R[6] = 2*q[1]*q[3] - 2*q[0]*q[2];
          R[7] = 2*q[2]*q[3] + 2*q[0]*q[1];
          R[8] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

          mulvec(R, world_vels, vels, 3, 3);
      }

  }; // class ESKF

} // namespace hf
