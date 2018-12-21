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

      double t_lastCall;
      double dt;

      ESKF_Sensor * _sensors[256];
      _sensor_count = 0;
      
    public:
      void init(void)
      {

      }

      void addSensorESKF(ESKF_Sensor * sensor)
      {
          _sensors[_sensor_count++] = sensor;
      }
      
      int update(void) {
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
        
        _sensors[sensorIndex].getJacobianModel(ekf->Fx, dt);
        _sensors[sensorIndex].getJacobianErrors(ekf->Fdx, dt);
        _sensors[sensorIndex].getCovarianceEstimation(ekf->Q);
        
        /* f(x) = F*ekf.x; */
        Matrix::mult(ekf->Fx, ekf->x, ekf->tmp6);
        Matrix::norvec(ekf->tmp6, ekf->fx);

        /* P_k = Fdx_{k-1} P_{k-1} Fdx^T_{k-1} + Q_{k-1} */
        Matrix::mult(ekf->Fdx, ekf->P, ekf->tmp0);
        Matrix::trans(ekf->Fdx, ekf->Fdxt);
        Matrix::mult(ekf->tmp0, ekf->Fdxt, ekf->tmp1);
        Matrix::accum(ekf->tmp1, ekf->Q);
        Matrix::makesym(ekf->tmp1, ekf->Pp);

        /* success */
        return 0;
        
      }
      
      int correct(void) {
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
        _sensors[sensorIndex].getJacobianObservation(eskf->H, eskf->x, eskf->dx->_rows);
        _sensors[sensorIndex].getInnovation(eskf->hx, eskf->x);
        _sensors[sensorIndex].getCovarianceCorrection(eskf->R);
        // Compute gain:
        /* K_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1} */
        Matrix::trans(eskf->H, eskf->Ht);
        Matrix::mult(eskf->Pp, eskf->Ht, eskf->tmp1);             // P*H'
        Matrix::mult(eskf->H, eskf->Pp, eskf->tmp2);              // H*P
        Matrix::mult(eskf->tmp2, eskf->Ht, eskf->tmp3);           // H*P*H'
        accum(eskf->tmp3, eskf->R);                               // Z = H*P*H' + R
        if (cholsl(eskf->tmp3, eskf->tmp4, eskf->tmp5)) return 1; // tmp4 = Z^-1
        Matrix::mult(eskf->tmp1, eskf->tmp4, eskf->K);            // K = P*H'*Z^-1
        
        // Estimate errors:
        /* dx = K * hx */
        Matrix::mult(ekf->K, ekf->hx, ekf->dx);

        // Update covariance
        /* P_k = P_k - K_k Z_k K^T_k  */
        Matrix::trans(ekf->K, ekf->Kt);
        Matrix::mult(ekf->K, ekf->tmp3, ekf->tmp0);
        Matrix::mult(ekf->tmp0, ekf->Kt, ekf->tmp3);
        Matrix::sub(ekf->Pp, ekf->tmp3, ekf->tmp0);
        Matrix::makesym(ekf->tmp0, ekf->P);

        /* Error injection */
        // XXX This is sensor dependent
        ekf->tmp6[0] = 1.0;
        ekf->tmp6[1] = ekf->dx[0]/2.0;
        ekf->tmp6[2] = ekf->dx[1]/2.0;
        ekf->tmp6[3] = ekf->dx[2]/2.0;
        Matrix::mult(ekf->qL, ekf->tmp6, ekf->tmp7);
        if (Matrix::norvec(ekf->tmp7, ekf->x)) return 1;

        /* Update covariance*/
        // XXX Only when correcting estimation
        ekf->tmp5[0] = ekf->dx[0]/2.0;
        ekf->tmp5[1] = ekf->dx[1]/2.0;
        ekf->tmp5[2] = ekf->dx[2]/2.0;
        if (Matrix::skew(ekf->tmp5, ekf->G)) return 1;
        Matrix::negate(ekf->G);
        Matrix::addeye(ekf->G);
        Matrix::trans(ekf->G, ekf->tmp0);
        Matrix::mult(ekf->P, ekf->tmp0, ekf->Pp);
        Matrix::mult(ekf->G, ekf->Pp, ekf->P);

        /* reset error state */
        Matrix::zeros(ekf->dx);

        /* success */
        return 0;
      }

  }; // class ESKF
  
} // namespace hf
