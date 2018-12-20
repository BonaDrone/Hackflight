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

#include <string.h>
#include <debug.hpp>

namespace hf {

    class Matrix {

        private:

            // avoid dynamic memory allocation
            static const uint8_t MAXSIZE = 16;

            uint8_t _rows = 0;
            uint8_t _cols = 0;

            float _vals[MAXSIZE][MAXSIZE];

        public:

            void setDimensions(uint8_t rows, uint8_t cols)
            {
                _rows = rows;
                _cols = cols;
                memset(_vals, 0, rows*cols*sizeof(float));              
            }

            float get(uint8_t j, uint8_t k)
            {
                return _vals[j][k];
            }

            float set(uint8_t j, uint8_t k, float val)
            {
                _vals[j][k] = val;
            }

            void dump(void)
            {
                for (uint8_t j=0; j<_rows; ++j) {
                    for (uint8_t k=0; k<_cols; ++k) {
                        Debug::printf("%+2.2f ", _vals[j][k]);
                    }
                    Debug::printf("\n");
                }
            }

            // AT <- A'
            static void trans(Matrix & a, Matrix & at)
            {
                at.setDimensions(a._cols, a._rows);
                for (uint8_t j=0; j<a._rows; ++j) {
                    for (uint8_t k=0; k<a._cols; ++k) {
                        at._vals[k][j] = a._vals[j][k];
                    }
                }
            }

            // C <- A * B
            static void mult(Matrix & a, Matrix & b, Matrix & c)
            {
                c.setDimensions(a._rows, b._cols);
                for(uint8_t i=0; i<a._rows; ++i) {
                    for(uint8_t j=0; j<b._cols; ++j) {
                        c._vals[i][j] = 0;
                        for(uint8_t k=0; k<a._cols; ++k) {
                            c._vals[i][j] += a._vals[i][k] *b._vals[k][j];
                        }
                    }
                }
            }
            
            // C <- A + B
            static void add(Matrix & a, Matrix & b, Matrix & c)
            {
              c.setDimensions(a._rows, a._cols);
              for(uint8_t i=0; i<a._rows; ++i) {
                  for(uint8_t j=0; j<a._cols; ++j) {
                      c._vals[i][j] = a._vals[i][j] + b._vals[i][j];
                      }
                  }
            }              

            // A <- A + B
            static void accum(Matrix & a, Matrix & b)
            {
              for(uint8_t i=0; i<a._rows; ++i) {
                  for(uint8_t j=0; j<a._cols; ++j) {
                      a._vals[i][j] += b._vals[i][j];
                      }
                  }
            }              

            // C <- A - B
            static void sub(Matrix & a, Matrix & b, Matrix & c)
            {
              c.setDimensions(a._rows, a._cols);
              for(uint8_t i=0; i<a._rows; ++i) {
                  for(uint8_t j=0; j<a._cols; ++j) {
                      c._vals[i][j] = a._vals[i][j] - b._vals[i][j];
                      }
                  }
            }              
            
            /* Cholesky-decomposition matrix-inversion code, adapated from
            http://jean-pierre.moreau.pagesperso-orange.fr/Cplus/choles_cpp.txt */

            static int choldc1(Matrix & a, Matrix & p) {
                int i,j,k;
                double sum;

                for (i = 0; i < a._rows; i++) {
                    for (j = i; j < a._cols; j++) {
                        sum = a._vals[i][j];
                        for (k = i - 1; k >= 0; k--) {
                            sum -= a._vals[i][k] * a._vals[j][k];
                        }
                        if (i == j) {
                            if (sum <= 0) {
                                return 1; /* error */
                            }
                            p._vals[i][0] = sqrt(sum);
                        }
                        else {
                            a._vals[j][i] = sum / p._vals[i][0];
                        }
                    }
                }

                return 0; /* success */
            }

            static int choldcsl(Matrix & A, Matrix & a, Matrix & p) 
            {
                int i,j,k; double sum;
                for (i = 0; i < A._rows; i++) 
                    for (j = 0; j < A._cols; j++) 
                        a._vals[i][j] = A._vals[i][j];
                if (choldc1(a, p)) return 1;
                for (i = 0; i < a._rows; i++) {
                    a._vals[i][i] = 1 / p._vals[i][0];
                    for (j = i + 1; j < a._cols; j++) {
                        sum = 0;
                        for (k = i; k < j; k++) {
                            sum -= a._vals[j][k] * a._vals[k][i];
                        }
                        a._vals[j][i] = sum / p._vals[j][0];
                    }
                }

                return 0; /* success */
            }


            static int cholsl(Matrix & A, Matrix & a, Matrix & p) 
            {
                int i,j,k;
                a.setDimensions(A._rows, A._cols);
                p.setDimensions(A._rows, 1);
                if (choldcsl(A,a,p)) return 1;
                for (i = 0; i < A._rows; i++) {
                    for (j = i + 1; j < A._cols; j++) {
                        a._vals[i][j] = 0.0;
                    }
                }
                for (i = 0; i < A._rows; i++) {
                    a._vals[i][i] *= a._vals[i][i];
                    for (k = i + 1; k < A._rows; k++) {
                        a._vals[i][i] += a._vals[k][i] * a._vals[k][i];
                    }
                    for (j = i + 1; j < A._cols; j++) {
                        for (k = j; k < A._cols; k++) {
                            a._vals[i][j] += a._vals[k][i] * a._vals[k][j];
                        }
                    }
                }
                for (i = 0; i < A._rows; i++) {
                    for (j = 0; j < i; j++) {
                        a._vals[i][j] = a._vals[j][i];
                    }
                }
                return 0; /* success */
            }

    };  // class Matrix

} // namespace hf