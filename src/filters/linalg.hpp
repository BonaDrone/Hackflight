/*
   linalg.hpp : Simple linear algebra support
   
   Copyright (c) 2018 Simon D. Levy
   
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

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

namespace hf {

  /* Cholesky-decomposition matrix-inversion code, adapated from
     http://jean-pierre.moreau.pagesperso-orange.fr/Cplus/choles_cpp.txt */
  static int choldc1(float * a, float * p, int n) 
  {
      int i,j,k;
      float sum;

      for (i = 0; i < n; i++) {
          for (j = i; j < n; j++) {
              sum = a[i*n+j];
              for (k = i - 1; k >= 0; k--) {
                  sum -= a[i*n+k] * a[j*n+k];
              }
              if (i == j) {
                  if (sum <= 0) {
                      return 1; /* error */
                  }
                  p[i] = sqrt(sum);
              }
              else {
                  a[j*n+i] = sum / p[i];
              }
          }
      }

      return 0; /* success */
  }

  static int choldcsl(float * A, float * a, float * p, int n) 
  {
      int i,j,k; float sum;
      for (i = 0; i < n; i++) 
          for (j = 0; j < n; j++) 
              a[i*n+j] = A[i*n+j];
      if (choldc1(a, p, n)) return 1;
      for (i = 0; i < n; i++) {
          a[i*n+i] = 1 / p[i];
          for (j = i + 1; j < n; j++) {
              sum = 0;
              for (k = i; k < j; k++) {
                  sum -= a[j*n+k] * a[k*n+i];
              }
              a[j*n+i] = sum / p[j];
          }
      }

      return 0; /* success */
  }


  static int cholsl(float * A, float * a, float * p, int n) 
  {
      int i,j,k;
      if (choldcsl(A,a,p,n)) return 1;
      for (i = 0; i < n; i++) {
          for (j = i + 1; j < n; j++) {
              a[i*n+j] = 0.0;
          }
      }
      for (i = 0; i < n; i++) {
          a[i*n+i] *= a[i*n+i];
          for (k = i + 1; k < n; k++) {
              a[i*n+i] += a[k*n+i] * a[k*n+i];
          }
          for (j = i + 1; j < n; j++) {
              for (k = j; k < n; k++) {
                  a[i*n+j] += a[k*n+i] * a[k*n+j];
              }
          }
      }
      for (i = 0; i < n; i++) {
          for (j = 0; j < i; j++) {
              a[i*n+j] = a[j*n+i];
          }
      }

      return 0; /* success */
  }

  static void zeros(float * a, int m, int n)
  {
      int j;
      for (j=0; j<m*n; ++j)
          a[j] = 0;
  }

  /* C <- A * B */
  static void mulmat(float * a, float * b, float * c, int arows, int acols, int bcols)
  {
      int i, j,l;

      for(i=0; i<arows; ++i)
          for(j=0; j<bcols; ++j) {
              c[i*bcols+j] = 0;
              for(l=0; l<acols; ++l)
                  c[i*bcols+j] += a[i*acols+l] * b[l*bcols+j];
          }
  }

  static void mulvec(float * a, float * x, float * y, int m, int n)
  {
      int i, j;

      for(i=0; i<m; ++i) {
          y[i] = 0;
          for(j=0; j<n; ++j)
              y[i] += x[j] * a[i*n+j];
      }
  }

  static void transpose(float * a, float * at, int m, int n)
  {
      int i,j;

      for(i=0; i<m; ++i)
          for(j=0; j<n; ++j) {
              at[j*m+i] = a[i*n+j];
          }
  }

  /* A <- A + B */
  static void accum(float * a, float * b, int m, int n)
  {        
      int i,j;

      for(i=0; i<m; ++i)
          for(j=0; j<n; ++j)
              a[i*n+j] += b[i*n+j];
  }

  /* C <- A + B */
  static void add(float * a, float * b, float * c, int n)
  {
      int j;

      for(j=0; j<n; ++j)
          c[j] = a[j] + b[j];
  }


  /* C <- A - B */
  static void sub(float * a, float * b, float * c, int n)
  {
      int j;

      for(j=0; j<n; ++j)
          c[j] = a[j] - b[j];
  }

  static void negate(float * a, int m, int n)
  {        
      int i, j;

      for(i=0; i<m; ++i)
          for(j=0; j<n; ++j)
              a[i*n+j] = -a[i*n+j];
  }

  static void mat_addeye(float * a, int n)
  {
      int i;
      for (i=0; i<n; ++i)
          a[i*n+i] += 1;
  }

  // skew matrix from a vector of dimension 3
  static void skew(float * x, float * c)
  {
      c[0] =  0.0;
      c[3] =  x[2];
      c[6] = -x[1];
      
      c[1] = -x[2];
      c[4] =  0.0;
      c[7] =  x[0];
      
      c[2] =  x[1];
      c[5] = -x[0];
      c[8] =  0.0;
  }
  
  // XXX Hardcoded skew matrix from a vector of dimension 3 in to a 6x6 matrix
  // XXX Do it in a generic way
  static void newSkew(float * x, float * c)
  {
      c[0] =  0.0;
      c[6] =  x[2];
      c[12] = -x[1];
      
      c[1] = -x[2];
      c[7] =  0.0;
      c[13] =  x[0];
      
      c[2] =  x[1];
      c[8] = -x[0];
      c[14] =  0.0;
  }

  static void norvec(float * x, float * y, int n)
  {
      float norm = 0;
      
      for (int ii=0; ii<n; ++ii)
        norm += x[ii]*x[ii];

      if (norm == 0)
      {
        for (int ii=0; ii<n; ++ii)
        {
          y[ii] = x[ii];  
        }
        return;
      }
      norm = 1.0/sqrt(norm);
      
      for (int ii=0; ii<n; ++ii)
        y[ii] = x[ii]*norm;
        
  }

  static void makesym(float *a, float *b, int n)
  {
      for (int ii=0; ii<n; ++ii)
      {
        for (int jj=0; jj<n; ++jj)
        {
          b[ii*n+jj] = (a[ii+jj*n] + a[jj+ii*n])/2.0;
        }
      }
  }


} // namespace hf