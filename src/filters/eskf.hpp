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
    
      eskf_t eskf;

      uint8_t errorStates = 3;
      uint8_t states = 4;

      double t_lastCall;
      double dt;

      ESKF_Sensor * _sensors[256];
      int _sensor_count = 0;
      
      
      void computeqL(void)
      {
          eskf.qL->setDimensions(4, 4);
          
          eskf.qL->set(0, 0, eskf.x->get(0, 0));
          eskf.qL->set(1, 0, eskf.x->get(1, 0));
          eskf.qL->set(2, 0, eskf.x->get(2, 0));
          eskf.qL->set(3, 0, eskf.x->get(3, 0));
          
          eskf.qL->set(0, 1, -eskf.x->get(1, 0));
          eskf.qL->set(1, 1, eskf.x->get(0, 0));
          eskf.qL->set(2, 1, eskf.x->get(3, 0));
          eskf.qL->set(3, 1, -eskf.x->get(2, 0));
          
          eskf.qL->set(0, 2, -eskf.x->get(2, 0));
          eskf.qL->set(1, 2, -eskf.x->get(3, 0));
          eskf.qL->set(2, 2, eskf.x->get(0, 0));
          eskf.qL->set(3, 2, eskf.x->get(1, 0));
          
          eskf.qL->set(0, 3, -eskf.x->get(3, 0));
          eskf.qL->set(1, 3, eskf.x->get(2, 0));
          eskf.qL->set(2, 3, -eskf.x->get(1, 0));
          eskf.qL->set(3, 3, eskf.x->get(0, 0));
      }
      
    public:

      void init(void)
      {
          eskf.P->setDimensions(errorStates, errorStates);
          eskf.x->setDimensions(states, 1);
          
          eskf.x->set(0, 0, 1.0);
          eskf.x->set(1, 0, 0.0);
          eskf.x->set(2, 0, 0.0);
          eskf.x->set(3, 0, 0.0);
          
          eskf.P->set(0, 0, 1.0);
          eskf.P->set(1, 0, 0.0);
          eskf.P->set(2, 0, 0.0);
          
          eskf.P->set(0, 1, 0.0);
          eskf.P->set(1, 1, 1.0);
          eskf.P->set(2, 1, 0.0);

          eskf.P->set(0, 2, 0.0);
          eskf.P->set(1, 2, 0.0);
          eskf.P->set(2, 2, 1.0);
      }

      void addSensorESKF(ESKF_Sensor * sensor)
      {
          _sensors[_sensor_count++] = sensor;
      }
      
      int update(void) {
        uint8_t sensorIndex = 1; 
        /*
        This method should:
          1. Obtain the nominal state Jacobian and the error-state Jacobian.
          2. Update the nominal state estimate
          3. From the error-state Jacobian, the process noise and the past 
             iteration covariance estimate the current Covariance (and enforce
             its symmetry?)
        */
        
        // Compute deltat
        double t_now = micros();
        dt = (t_now - t_lastCall)/1000000.0;
        t_lastCall = t_now;
        
        _sensors[sensorIndex]->getJacobianModel(eskf.Fx, dt);
        _sensors[sensorIndex]->getJacobianErrors(eskf.Fdx, dt);
        _sensors[sensorIndex]->getCovarianceEstimation(eskf.Q, errorStates);
        
        /* f(x) = F*eskf.x; */
        Matrix::mult(eskf.Fx, eskf.x, eskf.tmp6);
        Matrix::norvec(eskf.tmp6, eskf.fx);

        /* P_k = Fdx_{k-1} P_{k-1} Fdx^T_{k-1} + Q_{k-1} */
        Matrix::mult(eskf.Fdx, eskf.P, eskf.tmp0);
        Matrix::trans(eskf.Fdx, eskf.Fdxt);
        Matrix::mult(eskf.tmp0, eskf.Fdxt, eskf.tmp1);
        Matrix::accum(eskf.tmp1, eskf.Q);
        Matrix::makesym(eskf.tmp1, eskf.Pp);

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
        _sensors[sensorIndex]->getJacobianObservation(eskf.H, eskf.x, errorStates);
        _sensors[sensorIndex]->getInnovation(eskf.hx, eskf.x);
        _sensors[sensorIndex]->getCovarianceCorrection(eskf.R);
        // Compute gain:
        /* K_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1} */
        Matrix::trans(eskf.H, eskf.Ht);
        Matrix::mult(eskf.Pp, eskf.Ht, eskf.tmp1);             // P*H'
        Matrix::mult(eskf.H, eskf.Pp, eskf.tmp2);              // H*P
        Matrix::mult(eskf.tmp2, eskf.Ht, eskf.tmp3);           // H*P*H'
        Matrix::accum(eskf.tmp3, eskf.R);                               // Z = H*P*H' + R
        if (Matrix::cholsl(eskf.tmp3, eskf.tmp4, eskf.tmp5)) return 1; // tmp4 = Z^-1
        Matrix::mult(eskf.tmp1, eskf.tmp4, eskf.K);            // K = P*H'*Z^-1
        
        // Estimate errors:
        /* dx = K * hx */
        Matrix::mult(eskf.K, eskf.hx, eskf.dx);

        // Update covariance
        /* P_k = P_k - K_k Z_k K^T_k  */
        Matrix::trans(eskf.K, eskf.Kt);
        Matrix::mult(eskf.K, eskf.tmp3, eskf.tmp0);
        Matrix::mult(eskf.tmp0, eskf.Kt, eskf.tmp3);
        Matrix::sub(eskf.Pp, eskf.tmp3, eskf.tmp0);
        Matrix::makesym(eskf.tmp0, eskf.P);

        /* Error injection */
        // XXX This is sensor dependent
        eskf.tmp6->setDimensions(4,1);
        eskf.tmp6->set(0, 0, 1.0);
        eskf.tmp6->set(1, 0, eskf.dx->get(0, 0)/2.0);
        eskf.tmp6->set(2, 0, eskf.dx->get(1, 0)/2.0);
        eskf.tmp6->set(3, 0, eskf.dx->get(2, 0)/2.0);
        computeqL();
        Matrix::mult(eskf.qL, eskf.tmp6, eskf.tmp7);
        if (Matrix::norvec(eskf.tmp7, eskf.x)) return 1;

        /* Update covariance*/
        // XXX Only when correcting estimation
        eskf.tmp5->setDimensions(3, 1);
        eskf.tmp5->set(0,0, eskf.dx->get(0, 0)/2.0);
        eskf.tmp5->set(1,0, eskf.dx->get(1, 0)/2.0);
        eskf.tmp5->set(2,0, eskf.dx->get(2, 0)/2.0);
        if (Matrix::skew(eskf.tmp5, eskf.G)) return 1;
        Matrix::negate(eskf.G);
        Matrix::addeye(eskf.G);
        Matrix::trans(eskf.G, eskf.tmp0);
        Matrix::mult(eskf.P, eskf.tmp0, eskf.Pp);
        Matrix::mult(eskf.G, eskf.Pp, eskf.P);

        /* reset error state */
        Matrix::zeros(eskf.dx);

        /* success */
        return 0;
      }
      
      void getState(float q[4])
      {
        q[0] = eskf.x->get(0,0);
        q[1] = eskf.x->get(1,0);
        q[2] = eskf.x->get(2,0);
        q[3] = eskf.x->get(3,0);
      }

  }; // class ESKF
  
} // namespace hf
