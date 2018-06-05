/*
  algebra.hpp: This file contains a number of utilities useful for handling
  3D vectors

  This work is an adaptation from vvector.h, written by Linas Vepstras. The
  original code can be found at:

  https://github.com/markkilgard/glut/blob/master/lib/gle/vvector.h

  HISTORY:
  Written by Linas Vepstas, August 1991
  Added 2D code, March 1993
  Added Outer products, C++ proofed, Linas Vepstas October 1993
  Adapted for Hackflight's needs by Juan Gallostra June 2018

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
  along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <cmath>

namespace hf {
  // Copy 3D vector
  void vec_copy(float b[3],float a[3])
  {
     (b)[0] = (a)[0];
     (b)[1] = (a)[1];
     (b)[2] = (a)[2];
  }


  // Vector difference
  void vec_diff(float v21[3], float v2[3], float v1[3])
  {
     (v21)[0] = (v2)[0] - (v1)[0];
     (v21)[1] = (v2)[1] - (v1)[1];
     (v21)[2] = (v2)[2] - (v1)[2];
  }

  // Vector sum
  void vec_sum(float v21[3], float v2[3], float v1[3])
  {
     (v21)[0] = (v2)[0] + (v1)[0];
     (v21)[1] = (v2)[1] + (v1)[1];
     (v21)[2] = (v2)[2] + (v1)[2];
  }

  // scalar times vector
  void vec_scale(float c[3],float a, float b[3])
  {
     (c)[0] = (a)*(b)[0];
     (c)[1] = (a)*(b)[1];
     (c)[2] = (a)*(b)[2];
  }

  // accumulate scaled vector
  void vec_accum(float c[3], float a, float b[3])
  {
     (c)[0] += (a)*(b)[0];
     (c)[1] += (a)*(b)[1];
     (c)[2] += (a)*(b)[2];
  }

  // Vector dot product
  void vec_dot_product(float c, float a[3], float b[3])
  {
     c = (a)[0]*(b)[0] + (a)[1]*(b)[1] + (a)[2]*(b)[2];
  }

  // Vector length
  void vec_length(float len, float a[3])
  {
     len = (a)[0]*(a)[0] + (a)[1]*(a)[1];
     len += (a)[2]*(a)[2];
     len = sqrt (len);
  }

  // Normalize vector
  void vec_normalize(float a[3])
  {
     double len;
     vec_length(len,a);
     if (len != 0.0) {
        len = 1.0 / len;
        a[0] *= len;
        a[1] *= len;
        a[2] *= len;
     }
  }

  // 3D Vector cross product yeilding vector
  void vec_cross_product(float c[3], float a[3], float b[3])
  {
     c[0] = (a)[1] * (b)[2] - (a)[2] * (b)[1];
     c[1] = (a)[2] * (b)[0] - (a)[0] * (b)[2];
     c[2] = (a)[0] * (b)[1] - (a)[1] * (b)[0];
  }

  // initialize matrix
  void identify_matrix_3x3(float m[3][3])
  {
     m[0][0] = 1.0;
     m[0][1] = 0.0;
     m[0][2] = 0.0;

     m[1][0] = 0.0;
     m[1][1] = 1.0;
     m[1][2] = 0.0;

     m[2][0] = 0.0;
     m[2][1] = 0.0;
     m[2][2] = 1.0;
  }

  // matrix copy
  void copy_matrix_3x3(float b[3][3], float a[3][3])
  {
     b[0][0] = a[0][0];
     b[0][1] = a[0][1];
     b[0][2] = a[0][2];

     b[1][0] = a[1][0];
     b[1][1] = a[1][1];
     b[1][2] = a[1][2];

     b[2][0] = a[2][0];
     b[2][1] = a[2][1];
     b[2][2] = a[2][2];
  }

  // matrix transpose
  void transpose_matrix_3x3(float b[3][3], float a[3][3])
  {
     b[0][0] = a[0][0];
     b[0][1] = a[1][0];
     b[0][2] = a[2][0];

     b[1][0] = a[0][1];
     b[1][1] = a[1][1];
     b[1][2] = a[2][1];

     b[2][0] = a[0][2];
     b[2][1] = a[1][2];
     b[2][2] = a[2][2];
  }

  // multiply matrix by scalar
  void scale_matrix_3x3(float b[3][3], float s, float a[3][3])
  {
     b[0][0] = (s) * a[0][0];
     b[0][1] = (s) * a[0][1];
     b[0][2] = (s) * a[0][2];

     b[1][0] = (s) * a[1][0];
     b[1][1] = (s) * a[1][1];
     b[1][2] = (s) * a[1][2];

     b[2][0] = (s) * a[2][0];
     b[2][1] = (s) * a[2][1];
     b[2][2] = (s) * a[2][2];
  }

  // multiply matrix by scalar and add result to another matrix
  void accum_scale_matrix_3x3(float b[3][3], float s, float a[3][3])
  {
     b[0][0] += (s) * a[0][0];
     b[0][1] += (s) * a[0][1];
     b[0][2] += (s) * a[0][2];

     b[1][0] += (s) * a[1][0];
     b[1][1] += (s) * a[1][1];
     b[1][2] += (s) * a[1][2];

     b[2][0] += (s) * a[2][0];
     b[2][1] += (s) * a[2][1];
     b[2][2] += (s) * a[2][2];
  }

  // matrix product
  // c[x][y] = a[x][0]*b[0][y]+a[x][1]*b[1][y]+a[x][2]*b[2][y]+a[x][3]*b[3][y]
  void matrix_product_3x3(float c[3][3], float a[3][3], float b[3][3])
  {
     c[0][0] = a[0][0]*b[0][0]+a[0][1]*b[1][0]+a[0][2]*b[2][0];
     c[0][1] = a[0][0]*b[0][1]+a[0][1]*b[1][1]+a[0][2]*b[2][1];
     c[0][2] = a[0][0]*b[0][2]+a[0][1]*b[1][2]+a[0][2]*b[2][2];

     c[1][0] = a[1][0]*b[0][0]+a[1][1]*b[1][0]+a[1][2]*b[2][0];
     c[1][1] = a[1][0]*b[0][1]+a[1][1]*b[1][1]+a[1][2]*b[2][1];
     c[1][2] = a[1][0]*b[0][2]+a[1][1]*b[1][2]+a[1][2]*b[2][2];

     c[2][0] = a[2][0]*b[0][0]+a[2][1]*b[1][0]+a[2][2]*b[2][0];
     c[2][1] = a[2][0]*b[0][1]+a[2][1]*b[1][1]+a[2][2]*b[2][1];
     c[2][2] = a[2][0]*b[0][2]+a[2][1]*b[1][2]+a[2][2]*b[2][2];
  }

  // matrix times vector
  void mat_dot_vec_3x3(float p[3], float m[3][3], float v[3])
  {
     p[0] = m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2];
     p[1] = m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2];
     p[2] = m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2];
  }

  // determinant of matrix
  // Computes determinant of matrix m, returning d
  void determinant_3x3(float d, float m[3][3])
  {
     d = m[0][0] * (m[1][1]*m[2][2] - m[1][2] * m[2][1]);
     d -= m[0][1] * (m[1][0]*m[2][2] - m[1][2] * m[2][0]);
     d += m[0][2] * (m[1][0]*m[2][1] - m[1][1] * m[2][0]);
  }

  // adjoint of matrix
  // Computes adjoint of matrix m, returning a
  // (Note that adjoint is just the transpose of the cofactor matrix)
  void adjoint_3X3(float a[3][3], float m[3][3])
  {
     a[0][0] = m[1][1]*m[2][2] - m[1][2]*m[2][1];
     a[1][0] = - (m[1][0]*m[2][2] - m[2][0]*m[1][2]);
     a[2][0] = m[1][0]*m[2][1] - m[1][1]*m[2][0];
     a[0][1] = - (m[0][1]*m[2][2] - m[0][2]*m[2][1]);
     a[1][1] = m[0][0]*m[2][2] - m[0][2]*m[2][0];
     a[2][1] = - (m[0][0]*m[2][1] - m[0][1]*m[2][0]);
     a[0][2] = m[0][1]*m[1][2] - m[0][2]*m[1][1];
     a[1][2] = - (m[0][0]*m[1][2] - m[0][2]*m[1][0]);
     a[2][2] = m[0][0]*m[1][1] - m[0][1]*m[1][0];
  }

  // compute adjoint of matrix and scale
  // Computes adjoint of matrix m, scales it by s, returning a
  void scale_adjoint_3X3(float a[3][3], float s, float m[3][3])
  {
     a[0][0] = (s) * (m[1][1] * m[2][2] - m[1][2] * m[2][1]);
     a[1][0] = (s) * (m[1][2] * m[2][0] - m[1][0] * m[2][2]);
     a[2][0] = (s) * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

     a[0][1] = (s) * (m[0][2] * m[2][1] - m[0][1] * m[2][2]);
     a[1][1] = (s) * (m[0][0] * m[2][2] - m[0][2] * m[2][0]);
     a[2][1] = (s) * (m[0][1] * m[2][0] - m[0][0] * m[2][1]);

     a[0][2] = (s) * (m[0][1] * m[1][2] - m[0][2] * m[1][1]);
     a[1][2] = (s) * (m[0][2] * m[1][0] - m[0][0] * m[1][2]);
     a[2][2] = (s) * (m[0][0] * m[1][1] - m[0][1] * m[1][0]);
  }

  // inverse of matrix
  // Compute inverse of matrix a, returning determinant m and
  // inverse b
  void invert_3X3(float b[3][3], float a[3][3])
  {
     double tmp;
     determinant_3x3(tmp, a);
     tmp = 1.0 / (tmp);
     scale_adjoint_3X3(b, tmp, a);
  }

  // skew matrix from vector
  void skew(float a[3][3], float v[3])
  {
    a[0][1] = -v[2];
    a[0][2] = v[1];
    a[1][2] = -v[0];
    a[1][0] = v[2];
    a[2][0] = -v[1];
    a[2][1] = v[0];
  }

} // namespace hf
