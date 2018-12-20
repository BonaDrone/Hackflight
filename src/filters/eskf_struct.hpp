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
namespace hf {

  typedef struct {

      int nn;          /* number of state values */
      int ne;          /* number of state values */
      int m;          /* number of observables */

      double x[NNsta];    /*nominal state vector */
      double dx[NEsta];   /*error-state vector*/
      double qL[NNsta][NNsta]; /*Left matrix quaternion*/

      double P[NEsta][NEsta];  /* prediction error covariance */
      double Q[NEsta][NEsta];  /* process noise covariance */
      double R[Mobs][Mobs];  /* measurement error covariance */

      double K[NEsta][Mobs];  /* Kalman gain; a.k.a. K */
      double Kt[Mobs][NEsta];  /* transpose Kalman gain; a.k.a. K */

      double Fx[NNsta][NNsta];  /* Jacobian of process model */
      double Fdx[NEsta][NEsta];  /* Jacobian of process model */
      double H[Mobs][NEsta];  /* Jacobian of measurement model */

      double Ht[NEsta][Mobs]; /* transpose of measurement Jacobian */
      double Fdxt[NEsta][NEsta]; /* transpose of process Jacobian */
      double Pp[NEsta][NEsta]; /* P, post-prediction, pre-update */
      
      double G[NEsta][NEsta];  

      double fx[NNsta];   /* output of user defined f() state-transition function */
      double hx[Mobs];   /* output of user defined h() measurement function */

      /* temporary storage */
      double tmp0[NEsta][NEsta];
      double tmp1[NEsta][Mobs];
      double tmp2[Mobs][NEsta];
      double tmp3[Mobs][Mobs];
      double tmp4[Mobs][Mobs];
      double tmp5[Mobs];
      double tmp6[NNsta]; 
      double tmp7[NNsta];
      double tmp8[Mobs];

  } ekf_t;
  
} // namespace hf
