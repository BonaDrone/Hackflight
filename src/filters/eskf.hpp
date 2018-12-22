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
    
    private:
      
      Matrix x;     /*nominal state vector */
      Matrix dx;    /*error-state vector*/
      Matrix qL;    /*Left matrix quaternion*/

      Matrix P;     /* prediction error covariance */
      Matrix Q;     /* process noise covariance */
      Matrix R;     /* measurement error covariance */

      Matrix K;     /* Kalman gain; a.k.a. K */
      Matrix Kt;    /* transpose Kalman gain; a.k.a. K */

      Matrix Fx;    /* Jacobian of process model */
      Matrix Fdx;   /* Jacobian of process model */
      Matrix H;     /* Jacobian of measurement model */

      Matrix Ht;    /* transpose of measurement Jacobian */
      Matrix Fdxt;  /* transpose of process Jacobian */
      Matrix Pp;    /* P, post-prediction, pre-update */
      
      Matrix G;  

      Matrix fx;   /* output of user defined f() state-transition function */
      Matrix hx;   /* output of user defined h() measurement function */

      /* temporary storage */
      Matrix tmp0;
      Matrix tmp1;
      Matrix tmp2;
      Matrix tmp3;
      Matrix tmp4;
      Matrix tmp5;
      Matrix tmp6; 
      Matrix tmp7;
      Matrix tmp8;
      //eskf_t eskf;

      static const uint8_t errorStates = 3;
      static const uint8_t nominalStates = 4;
      static const uint8_t observations = 3;

      double t_lastCall;
      double dt;

      ESKF_Sensor * _sensors[256];
      int _sensor_count = 0;
      
      
      void computeqL(void)
      {
          qL.set(0, 0, x.get(0, 0));
          qL.set(1, 0, x.get(1, 0));
          qL.set(2, 0, x.get(2, 0));
          qL.set(3, 0, x.get(3, 0));
          
          qL.set(0, 1, -x.get(1, 0));
          qL.set(1, 1, x.get(0, 0));
          qL.set(2, 1, x.get(3, 0));
          qL.set(3, 1, -x.get(2, 0));
          
          qL.set(0, 2, -x.get(2, 0));
          qL.set(1, 2, -x.get(3, 0));
          qL.set(2, 2, x.get(0, 0));
          qL.set(3, 2, x.get(1, 0));
          
          qL.set(0, 3, -x.get(3, 0));
          qL.set(1, 3, x.get(2, 0));
          qL.set(2, 3, -x.get(1, 0));
          qL.set(3, 3, x.get(0, 0));
      }
      
      void initMatrices(void)
      {
        x =  Matrix(nominalStates, 1);     /*nominal state vector */
        dx =  Matrix(errorStates, 1);    /*error-state vector*/
        qL =  Matrix(nominalStates, nominalStates);    /*Left quaternion*/

        P =  Matrix(errorStates, errorStates);     /* prediction error covariance */
        Q =  Matrix(errorStates, errorStates);     /* process noise covariance */
        R =  Matrix(observations, observations);     /* measurement error covariance */

        K =  Matrix(errorStates, observations);     /* Kalman gain; a.k.a. K */
        Kt =  Matrix(observations, errorStates);    /* transpose Kalman gain; a.k.a. K */

        Fx =  Matrix(nominalStates, nominalStates);    /* Jacobian of process model */
        Fdx =  Matrix(errorStates, errorStates);   /* Jacobian of process model */
        H =  Matrix(observations, errorStates);     /* Jacobian of measurement model */

        Ht =  Matrix(errorStates, observations);    /* transpose of measurement Jacobian */
        Fdxt =  Matrix(errorStates, errorStates);  /* transpose of process Jacobian */
        Pp =  Matrix(errorStates, errorStates);    /* P, post-prediction, pre-update */
        
        G =  Matrix(errorStates, errorStates);  

        fx =  Matrix(nominalStates, 1);   /* output of user defined f() state-transition function */
        hx =  Matrix(observations, 1);   /* output of user defined h() measurement function */

        /* temporary storage */
        tmp0 =  Matrix(errorStates, errorStates);
        tmp1 =  Matrix(errorStates, observations);
        tmp2 =  Matrix(observations, errorStates);
        tmp3 =  Matrix(observations, observations);
        tmp4 =  Matrix(observations, observations);
        tmp5 =  Matrix(observations, 1);
        tmp6 =  Matrix(nominalStates, 1); 
        tmp7 =  Matrix(nominalStates, 1);
        tmp8 =  Matrix(observations, 1);
      }
      
    public:

      ESKF(){}

      void init(void)
      {
          initMatrices();
          x.set(0, 0, 1.0);
          x.set(1, 0, 0.0);
          x.set(2, 0, 0.0);
          x.set(3, 0, 0.0);
          
          P.set(0, 0, 1.0);
          P.set(1, 0, 0.0);
          P.set(2, 0, 0.0);
          
          P.set(0, 1, 0.0);
          P.set(1, 1, 1.0);
          P.set(2, 1, 0.0);

          P.set(0, 2, 0.0);
          P.set(1, 2, 0.0);
          P.set(2, 2, 1.0);
      }

      void addSensorESKF(ESKF_Sensor * sensor)
      {
          _sensors[_sensor_count++] = sensor;
      }
      
      int update(void) {
        uint8_t sensorIndex = 0; 
        /*
        This method should:
          1. Obtain the nominal state Jacobian and the error-state Jacobian.
          2. Update the nominal state estimate
          3. From the error-state Jacobian, the process noise and the past 
             iteration covariance estimate the current Covariance (and enforce
             its symmetry?)
        */
        Serial.println("Called");
        // Compute deltat
        double t_now = micros();
        dt = (t_now - t_lastCall)/1000000.0;
        t_lastCall = t_now;
        
        _sensors[sensorIndex]->getJacobianModel(Fx, dt);
        _sensors[sensorIndex]->getJacobianErrors(Fdx, dt);
        _sensors[sensorIndex]->getCovarianceEstimation(Q, errorStates);
        
        /* f(x) = F*x; */
        Matrix::mult(Fx, x, tmp6);
        Matrix::norvec(tmp6, fx);

        /* P_k = Fdx_{k-1} P_{k-1} Fdx^T_{k-1} + Q_{k-1} */
        Matrix::mult(Fdx, P, tmp0);
        Matrix::trans(Fdx, Fdxt);
        Matrix::mult(tmp0, Fdxt, tmp1);
        Matrix::accum(tmp1, Q);
        Matrix::makesym(tmp1, Pp);

        /* success */
        return 0;
      }
      
      int correct(void) {
        uint8_t sensorIndex = 1; 
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
        _sensors[sensorIndex]->getJacobianObservation(H, x, errorStates);
        _sensors[sensorIndex]->getInnovation(hx, x);
        _sensors[sensorIndex]->getCovarianceCorrection(R);
        // Compute gain:
        /* K_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1} */
        Matrix::trans(H, Ht);
        Matrix::mult(Pp, Ht, tmp1);             // P*H'
        Matrix::mult(H, Pp, tmp2);              // H*P
        Matrix::mult(tmp2, Ht, tmp3);           // H*P*H'
        Matrix::accum(tmp3, R);                               // Z = H*P*H' + R
        if (Matrix::cholsl(tmp3, tmp4, tmp5)) return 1; // tmp4 = Z^-1
        Matrix::mult(tmp1, tmp4, K);            // K = P*H'*Z^-1
        
        // Estimate errors:
        /* dx = K * hx */
        Matrix::mult(K, hx, dx);

        // Update covariance
        /* P_k = P_k - K_k Z_k K^T_k  */
        Matrix::trans(K, Kt);
        Matrix::mult(K, tmp3, tmp0);
        Matrix::mult(tmp0, Kt, tmp3);
        Matrix::sub(Pp, tmp3, tmp0);
        Matrix::makesym(tmp0, P);

        /* Error injection */
        // XXX This is sensor dependent
        tmp6.set(0, 0, 1.0);
        tmp6.set(1, 0, dx.get(0, 0)/2.0);
        tmp6.set(2, 0, dx.get(1, 0)/2.0);
        tmp6.set(3, 0, dx.get(2, 0)/2.0);
        computeqL();
        Matrix::mult(qL, tmp6, tmp7);
        if (Matrix::norvec(tmp7, x)) return 1;

        /* Update covariance*/
        // XXX Only when correcting estimation
        tmp5.set(0,0, dx.get(0, 0)/2.0);
        tmp5.set(1,0, dx.get(1, 0)/2.0);
        tmp5.set(2,0, dx.get(2, 0)/2.0);
        if (Matrix::skew(tmp5, G)) return 1;
        Matrix::negate(G);
        Matrix::addeye(G);
        Matrix::trans(G, tmp0);
        Matrix::mult(P, tmp0, Pp);
        Matrix::mult(G, Pp, P);

        /* reset error state */
        Matrix::zeros(dx);

        /* success */
        return 0;
      }
      
      void getState(float q[4])
      {
        q[0] = x.get(0,0);
        q[1] = x.get(1,0);
        q[2] = x.get(2,0);
        q[3] = x.get(3,0);
      }

  }; // class ESKF
  
} // namespace hf
