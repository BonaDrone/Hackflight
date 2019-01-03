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

#define NNsta 4
#define NEsta 3
#define Mobs 3

namespace hf {

  typedef struct {

      float x[NNsta];    /*nominal state vector */
      float dx[NEsta];   /*error-state vector*/
      float qL[NNsta][NNsta]; /*Left matrix quaternion*/

      float P[NEsta][NEsta];  /* prediction error covariance */
      float Q[NEsta][NEsta];  /* process noise covariance */
      float R[Mobs][Mobs];  /* measurement error covariance */

      float K[NEsta][Mobs];  /* Kalman gain; a.k.a. K */
      float Kt[Mobs][NEsta];  /* transpose Kalman gain; a.k.a. K */

      float Fx[NNsta][NNsta];  /* Jacobian of process model */
      float Fdx[NEsta][NEsta];  /* Jacobian of process model */
      float H[Mobs][NEsta];  /* Jacobian of measurement model */

      float Ht[NEsta][Mobs]; /* transpose of measurement Jacobian */
      float Fdxt[NEsta][NEsta]; /* transpose of process Jacobian */
      float Pp[NEsta][NEsta]; /* P, post-prediction, pre-update */
      
      float G[NEsta][NEsta];  

      float fx[NNsta];   /* output of user defined f() state-transition function */
      float hx[Mobs];   /* output of user defined h() measurement function */

      /* temporary storage */
      float tmp0[NEsta][NEsta];
      float tmp1[NEsta][Mobs];
      float tmp2[Mobs][NEsta];
      float tmp3[Mobs][Mobs];
      float tmp4[Mobs][Mobs];
      float tmp5[Mobs];
      float tmp6[NNsta]; 
      float tmp7[NNsta];
      float tmp8[Mobs];

  } eskf_t;
  
  typedef struct {

      float * x;    /*nominal state vector */
      float * dx;   /*error-state vector*/
      float * qL; /*Left matrix quaternion*/

      float * P;  /* prediction error covariance */
      float * Q;  /* process noise covariance */
      float * R;  /* measurement error covariance */

      float * K;  /* Kalman gain; a.k.a. K */
      float * Kt;  /* transpose Kalman gain; a.k.a. K */

      float * Fx;  /* Jacobian of process model */
      float * Fdx;  /* Jacobian of process model */
      float * H;  /* Jacobian of measurement model */

      float * Ht; /* transpose of measurement Jacobian */
      float * Fdxt; /* transpose of process Jacobian */
      float * Pp; /* P, post-prediction, pre-update */
      
      float * G;  

      float * fx;   /* output of user defined f() state-transition function */
      float * hx;   /* output of user defined h() measurement function */

      /* temporary storage */
      float * tmp0;
      float * tmp1;
      float * tmp2;
      float * tmp3;
      float * tmp4;
      float * tmp5;
      float * tmp6; 
      float * tmp7;
      float * tmp8;

  } eskf_p_t;
  
  typedef struct {
    
    float eulerAngles[3];
    float linearVelocities[3];
    float angularVelocities[3];
    float position[3];
    
  } eskf_state_t;
  
} // namespace hf
