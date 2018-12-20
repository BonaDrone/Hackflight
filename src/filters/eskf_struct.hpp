/*
    eskf_struct.hpp: common data structure for ESKF. This struct stores the 
    current state of the filter

    You should #include this file after using #define for NNsta (nominal states),
    NEsta (error states) and Mobs (observations)

    Copyright (c) 2018 Simon D. Levy, Juan Gallostra Acin, Pep Martí Saumell

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

namespace hf {

  typedef struct {

      Matrix * x;    /*nominal state vector */
      Matrix * dx;   /*error-state vector*/
      Matrix * qL; /*Left matrix quaternion*/

      Matrix * P;  /* prediction error covariance */
      Matrix * Q;  /* process noise covariance */
      Matrix * R;  /* measurement error covariance */

      Matrix * K;  /* Kalman gain; a.k.a. K */
      Matrix * Kt;  /* transpose Kalman gain; a.k.a. K */

      Matrix * Fx;  /* Jacobian of process model */
      Matrix * Fdx;  /* Jacobian of process model */
      Matrix * H;  /* Jacobian of measurement model */

      Matrix * Ht; /* transpose of measurement Jacobian */
      Matrix * Fdxt; /* transpose of process Jacobian */
      Matrix * Pp; /* P, post-prediction, pre-update */
      
      Matrix * G;  

      Matrix * fx;   /* output of user defined f() state-transition function */
      Matrix * hx;   /* output of user defined h() measurement function */

      /* temporary storage */
      Matrix * tmp0;
      Matrix * tmp1;
      Matrix * tmp2;
      Matrix * tmp3;
      Matrix * tmp4;
      Matrix * tmp5;
      Matrix * tmp6; 
      Matrix * tmp7;
      Matrix * tmp8;

  } ekf_t;
  
} // namespace hf
