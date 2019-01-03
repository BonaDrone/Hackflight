/*
   eskf.hpp : Implementation of Error-state Kalman Filter for state estimation

   Copyright (c) 2018 Juan Gallostra Acin, Pep Martí Saumell

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
          
          eskfp.Fx = dptr;
          dptr += nn*nn;
          eskfp.Fdx = dptr;
          dptr += ne*ne;
          eskfp.H = dptr;
          dptr += m*ne;
          
          eskfp.Ht = dptr;
          dptr += ne*m;
          eskfp.Fdxt = dptr;
          dptr += ne*ne;
          eskfp.Pp = dptr;
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
          dptr += m;
          eskfp.tmp6 = dptr;
          dptr += nn;
          eskfp.tmp7 = dptr;
          dptr += nn;
          eskfp.tmp8 = dptr;        
      }
      
      void computeqL(float * qL, float * x)
      {        
          qL[0] =  x[0];
          qL[4] =  x[1];
          qL[8] =  x[2];
          qL[12] =  x[3];
          
          qL[1] = -x[1];
          qL[5] =  x[0];
          qL[9] =  x[3];
          qL[13] = -x[2];
          
          qL[2]  = -x[2];
          qL[6]  = -x[3];
          qL[10] =  x[0];
          qL[14] =  x[1];
          
          qL[3] = -x[3];
          qL[7] =  x[2];
          qL[11] = -x[1];
          qL[15] =  x[0];
      }
      
      void synchState(void)
      {
          // Update euler angles
          float q[4] = {eskf.x[0], eskf.x[1], eskf.x[2], eskf.x[3]};
          Quaternion::computeEulerAngles(q, state.eulerAngles);
          // Convert heading from [-pi,+pi] to [0,2*pi]
          if (state.eulerAngles[2] < 0) {
              state.eulerAngles[2] += 2*M_PI;
          }
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
          zeros(eskfp.Fx, nominalStates, nominalStates);
          zeros(eskfp.Fdx, errorStates, errorStates);
          zeros(eskfp.H, observations, errorStates);
          
          eskfp.x[0] = 1.0;
          eskfp.x[1] = 0.0;
          eskfp.x[2] = 0.0;
          eskfp.x[3] = 0.0;
          
          eskfp.P[0] = 1.0;
          eskfp.P[1] = 0;
          eskfp.P[2] = 0;
          
          eskfp.P[3] = 0.0;
          eskfp.P[4] = 1.0;
          eskfp.P[5] = 0.0;
        
          eskfp.P[6] = 0.0;
          eskfp.P[7] = 0.0;
          eskfp.P[8] = 1.0;
      }

      void addSensorESKF(ESKF_Sensor * sensor)
      {
          sensors[sensor_count++] = sensor;
      }
      
      int update(ESKF_Sensor * sensor, float time) 
      {
          /*
          This method should:
            1. Obtain the nominal state Jacobian and the error-state Jacobian.
            2. Update the nominal state estimate
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
          
          sensor->getJacobianModel(eskfp.Fx, dt);
          sensor->getJacobianErrors(eskfp.Fdx, dt);
          sensor->getCovarianceEstimation(eskfp.Q);
          
          /* f(x) = F*eskfp.x; */
          mulvec(eskfp.Fx, eskfp.x, eskfp.tmp6, nominalStates, nominalStates);
          norvec(eskfp.tmp6, eskfp.fx, nominalStates);
          
          /* P_k = Fdx_{k-1} P_{k-1} Fdx^T_{k-1} + Q_{k-1} */
          mulmat(eskfp.Fdx, eskfp.P, eskfp.tmp0, errorStates, errorStates, errorStates);
          transpose(eskfp.Fdx, eskfp.Fdxt, errorStates, errorStates);
          mulmat(eskfp.tmp0, eskfp.Fdxt, eskfp.tmp1, errorStates, errorStates, errorStates);
          accum(eskfp.tmp1, eskfp.Q, errorStates, errorStates);
          //makesym(eskfp.tmp1, eskfp.P, errorStates);
          makesym(eskfp.tmp1, eskfp.Pp, errorStates);

          /* success */
          synchState();
          return 0;
      }
      
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
          
          // Check sensor
          if (sensor->ready(time)) {
              // Update state with gyro rates
              sensor->modifyState(state, time);                    
          } 
          
          // Comming from eskf.update the state is stored in fx
          sensor->getJacobianObservation(eskfp.H, eskfp.fx);
          sensor->getInnovation(eskfp.hx, eskfp.fx);
          sensor->getCovarianceCorrection(eskfp.R);

          // Compute gain:
          /* K_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1} */
          transpose(eskfp.H, eskfp.Ht, observations, errorStates);
          mulmat(eskfp.Pp, eskfp.Ht, eskfp.tmp1, errorStates, errorStates, observations); // P*H'
          mulmat(eskfp.H, eskfp.Pp, eskfp.tmp2, observations, errorStates, errorStates);  // H*P
          mulmat(eskfp.tmp2, eskfp.Ht, eskfp.tmp3, observations, errorStates, observations); // H*P*H'
          accum(eskfp.tmp3, eskfp.R, observations, observations);                 // Z = H*P*H' + R
          if (cholsl(eskfp.tmp3, eskfp.tmp4, eskfp.tmp5, observations)) return 1; // tmp4 = Z^-1
          mulmat(eskfp.tmp1, eskfp.tmp4, eskfp.K, errorStates, observations, observations); // K = P*H'*Z^-1
          
          // /* \hat{x}_k = \hat{x_k} + K_k(z_k - h(\hat{x}_k)) */
          //norvec(z, eskfp.tmp5, errorStates);
          //norvec(eskfp.hx, eskfp.tmp8, observations);
          //sub(eskfp.tmp5, eskfp.tmp8, eskfp.hx, observations);
          mulvec(eskfp.K, eskfp.hx, eskfp.dx, errorStates, observations);
          
          /* P_k = P_k - K_k Z_k K^T_k  */
          transpose(eskfp.K, eskfp.Kt, observations, errorStates);
          mulmat(eskfp.K, eskfp.tmp3, eskfp.tmp0, errorStates, observations, errorStates);
          mulmat(eskfp.tmp0, eskfp.Kt, eskfp.tmp3, errorStates, errorStates, observations);
          sub(eskfp.Pp, eskfp.tmp3, eskfp.tmp0, errorStates);
          makesym(eskfp.tmp0, eskfp.P, errorStates);
          
          /* Error injection */
          eskfp.tmp6[0] = 1.0;
          eskfp.tmp6[1] = eskfp.dx[0]/2.0;
          eskfp.tmp6[2] = eskfp.dx[1]/2.0;
          eskfp.tmp6[3] = eskfp.dx[2]/2.0;
          computeqL(eskfp.qL, eskfp.fx);
          mulvec(eskfp.qL, eskfp.tmp6, eskfp.tmp7, nominalStates, nominalStates);
          norvec(eskfp.tmp7, eskfp.x, nominalStates);

          /* Update covariance*/
          eskfp.tmp5[0] = eskfp.dx[0]/2.0;
          eskfp.tmp5[1] = eskfp.dx[1]/2.0;
          eskfp.tmp5[2] = eskfp.dx[2]/2.0;
          skew(eskfp.tmp5, eskfp.G);
          negate(eskfp.G, errorStates, errorStates);
          mat_addeye(eskfp.G, errorStates);
          transpose(eskfp.G, eskfp.tmp0, errorStates, errorStates);
          mulmat(eskfp.P, eskfp.tmp0, eskfp.Pp, errorStates, errorStates, errorStates);
          mulmat(eskfp.G, eskfp.Pp, eskfp.P, errorStates, errorStates, errorStates);
          
          /* reset error state */
          zeros(eskfp.dx, errorStates, 1);
          /* success */
          synchState();
          return 0;
      }

      // XXX Debug
      void printMatrix3(float M[3][3], int r, int c)
      {
          for (int ii=0; ii<r; ++ii)
          {
            for (int jj=0; jj<c; ++jj)
            {
              if (jj == c-1)
              {
                Serial.println(M[ii][jj],8);
              }
              else
              {
                Serial.print(M[ii][jj],8);
                Serial.print(",");
              }
              
            }
          }
      }
      
      void printMatrix4(float M[4][4], int r, int c)
      {
          for (int ii=0; ii<r; ++ii)
          {
            for (int jj=0; jj<c; ++jj)
            {
              if (jj == c-1)
              {
                Serial.println(M[ii][jj],8);
              }
              else
              {
                Serial.print(M[ii][jj],8);
                Serial.print(",");
              }
              
            }
          }
      }

  }; // class ESKF
  
} // namespace hf
