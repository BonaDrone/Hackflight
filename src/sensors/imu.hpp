/*
   imu.hpp : Support for imu (a.k.a. gyroscope + accelerometer) 

   Copyright (c) 2019 BonaDrone (www.bonadrone.com)
   Developed by: Pep Marti-Saumell (jmarti<at>bonadrone.com>) & Juan Gallostra Acin (jgallostra<at>bonadrone.com)

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

#include <cmath>
#include <math.h>

#include "debug.hpp"
#include "sensor.hpp"
#include "surfacemount.hpp"
#include "board.hpp"

namespace hf {

    class IMU : public SurfaceMountSensor {

        friend class Hackflight;

        public:

            // IMU will be used to estimate but not to correct
            IMU(void) : SurfaceMountSensor(true, false)
            {
                memset(_rates, 0, 3*sizeof(float));
                memset(_accels, 0, 3*sizeof(float));
            }

            virtual void integrateNominalState(float * fx, float * x, double dt) override
            {
                // 1 column
                fx[0] = x[0] + dt*x[3];
                // 2 column
                fx[1] =  x[1] + dt*x[4];
                // 3 column
                fx[2] =  x[2] + dt*x[5];
                // 4 column
                fx[3] =  x[3] - dt*((x[10] - _accels[0])*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) - (x[11] - _accels[1])*(2*x[6]*x[9] - 2*x[7]*x[8]) + (x[12] - _accels[2])*(2*x[6]*x[8] + 2*x[7]*x[9]));
                // 5 column
                fx[4] =  x[4] - dt*((x[11] - _accels[1])*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) + (x[10] - _accels[0])*(2*x[6]*x[9] + 2*x[7]*x[8]) - (x[12] - _accels[2])*(2*x[6]*x[7] - 2*x[8]*x[9]));
                // 6 column
                fx[5] =  x[5] - dt*(9.80665 + (x[12] - _accels[2])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]) - (x[10] - _accels[0])*(2*x[6]*x[8] - 2*x[7]*x[9]) + (x[11] - _accels[1])*(2*x[6]*x[7] + 2*x[8]*x[9]));
                // 7 column
                fx[6] =  x[6] + (dt*x[7]*(x[13] - _rates[0]))/2 + (dt*x[8]*(x[14] - _rates[1]))/2 + (dt*x[9]*(x[15] - _rates[2]))/2;
                // 8 column
                fx[7] =  x[7] - (dt*x[6]*(x[13] - _rates[0]))/2 - (dt*x[8]*(x[15] - _rates[2]))/2 + (dt*x[9]*(x[14] - _rates[1]))/2;
                // 9 column
                fx[8] =  x[8] - (dt*x[6]*(x[14] - _rates[1]))/2 + (dt*x[7]*(x[15] - _rates[2]))/2 - (dt*x[9]*(x[13] - _rates[0]))/2;
                // 10 column
                fx[9] =  x[9] - (dt*x[6]*(x[15] - _rates[2]))/2 - (dt*x[7]*(x[14] - _rates[1]))/2 + (dt*x[8]*(x[13] - _rates[0]))/2;
                // 11 column
                fx[10] =  x[10];
                // 12 column
                fx[11] =  x[11];
                // 13 column
                fx[12] =  x[12];
                // 14 column
                fx[13] =  x[13];
                // 15 column
                fx[14] =  x[14];
                // 16 column
                fx[15] =  x[15];
            }
            
            virtual void getJacobianErrors(float * Fdx, float * x, double dt) override
            {
                // 1 column
                Fdx[0] =  1;
                Fdx[15] =  0;
                Fdx[30] =  0;
                Fdx[45] =  0;
                Fdx[60] =  0;
                Fdx[75] =  0;
                Fdx[90] =  0;
                Fdx[105] =  0;
                Fdx[120] =  0;
                Fdx[135] =  0;
                Fdx[150] =  0;
                Fdx[165] =  0;
                Fdx[180] =  0;
                Fdx[195] =  0;
                Fdx[210] =  0;
                // 2 column
                Fdx[1] =  0;
                Fdx[16] =  1;
                Fdx[31] =  0;
                Fdx[46] =  0;
                Fdx[61] =  0;
                Fdx[76] =  0;
                Fdx[91] =  0;
                Fdx[106] =  0;
                Fdx[121] =  0;
                Fdx[136] =  0;
                Fdx[151] =  0;
                Fdx[166] =  0;
                Fdx[181] =  0;
                Fdx[196] =  0;
                Fdx[211] =  0;
                // 3 column
                Fdx[2] =  0;
                Fdx[17] =  0;
                Fdx[32] =  1;
                Fdx[47] =  0;
                Fdx[62] =  0;
                Fdx[77] =  0;
                Fdx[92] =  0;
                Fdx[107] =  0;
                Fdx[122] =  0;
                Fdx[137] =  0;
                Fdx[152] =  0;
                Fdx[167] =  0;
                Fdx[182] =  0;
                Fdx[197] =  0;
                Fdx[212] =  0;
                // 4 column
                Fdx[3] =  dt;
                Fdx[18] =   0;
                Fdx[33] =   0;
                Fdx[48] =   1;
                Fdx[63] =   0;
                Fdx[78] =   0;
                Fdx[93] =   0;
                Fdx[108] =   0;
                Fdx[123] =   0;
                Fdx[138] =   0;
                Fdx[153] =   0;
                Fdx[168] =   0;
                Fdx[183] =   0;
                Fdx[198] =   0;
                Fdx[213] =   0;
                // 5 column
                Fdx[4] =   0;
                Fdx[19] =  dt;
                Fdx[34] =   0;
                Fdx[49] =   0;
                Fdx[64] =   1;
                Fdx[79] =   0;
                Fdx[94] =   0;
                Fdx[109] =   0;
                Fdx[124] =   0;
                Fdx[139] =   0;
                Fdx[154] =   0;
                Fdx[169] =   0;
                Fdx[184] =   0;
                Fdx[199] =   0;
                Fdx[214] =   0;
                // 6 column
                Fdx[5] =   0;
                Fdx[20] =   0;
                Fdx[35] =  dt;
                Fdx[50] =   0;
                Fdx[65] =   0;
                Fdx[80] =   1;
                Fdx[95] =   0;
                Fdx[110] =   0;
                Fdx[125] =   0;
                Fdx[140] =   0;
                Fdx[155] =   0;
                Fdx[170] =   0;
                Fdx[185] =   0;
                Fdx[200] =   0;
                Fdx[215] =   0;
                // 7 column
                Fdx[6] =                                                                                0;
                Fdx[21] =                                                                                0;
                Fdx[36] =                                                                                0;
                Fdx[51] =          -dt*((x[11] - _accels[1])*(2*x[6]*x[8] + 2*x[7]*x[9]) + (x[12] - _accels[2])*(2*x[6]*x[9] - 2*x[7]*x[8]));
                Fdx[66] =   dt*((x[12] - _accels[2])*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) + (x[11] - _accels[1])*(2*x[6]*x[7] - 2*x[8]*x[9]));
                Fdx[81] =  -dt*((x[11] - _accels[1])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]) - (x[12] - _accels[2])*(2*x[6]*x[7] + 2*x[8]*x[9]));
                Fdx[96] =                                                                                1;
                Fdx[111] =                                                                   dt*(x[15] - _rates[2]);
                Fdx[126] =                                                                  -dt*(x[14] - _rates[1]);
                Fdx[141] =                                                                                0;
                Fdx[156] =                                                                                0;
                Fdx[171] =                                                                                0;
                Fdx[186] =                                                                                0;
                Fdx[201] =                                                                                0;
                Fdx[216] =                                                                                0;
                // 8 column
                Fdx[7] =                                                                                0;
                Fdx[22] =                                                                                0;
                Fdx[37] =                                                                                0;
                Fdx[52] =  -dt*((x[12] - _accels[2])*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) - (x[10] - _accels[0])*(2*x[6]*x[8] + 2*x[7]*x[9]));
                Fdx[67] =          -dt*((x[10] - _accels[0])*(2*x[6]*x[7] - 2*x[8]*x[9]) + (x[12] - _accels[2])*(2*x[6]*x[9] + 2*x[7]*x[8]));
                Fdx[82] =   dt*((x[10] - _accels[0])*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]) + (x[12] - _accels[2])*(2*x[6]*x[8] - 2*x[7]*x[9]));
                Fdx[97] =                                                                  -dt*(x[15] - _rates[2]);
                Fdx[112] =                                                                                1;
                Fdx[127] =                                                                   dt*(x[13] - _rates[0]);
                Fdx[142] =                                                                                0;
                Fdx[157] =                                                                                0;
                Fdx[172] =                                                                                0;
                Fdx[187] =                                                                                0;
                Fdx[202] =                                                                                0;
                Fdx[217] =                                                                                0;
                // 9 column
                Fdx[8] =                                                                                0;
                Fdx[23] =                                                                                0;
                Fdx[38] =                                                                                0;
                Fdx[53] =   dt*((x[11] - _accels[1])*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]) + (x[10] - _accels[0])*(2*x[6]*x[9] - 2*x[7]*x[8]));
                Fdx[68] =  -dt*((x[10] - _accels[0])*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]) - (x[11] - _accels[1])*(2*x[6]*x[9] + 2*x[7]*x[8]));
                Fdx[83] =          -dt*((x[10] - _accels[0])*(2*x[6]*x[7] + 2*x[8]*x[9]) + (x[11] - _accels[1])*(2*x[6]*x[8] - 2*x[7]*x[9]));
                Fdx[98] =                                                                   dt*(x[14] - _rates[1]);
                Fdx[113] =                                                                  -dt*(x[13] - _rates[0]);
                Fdx[128] =                                                                                1;
                Fdx[143] =                                                                                0;
                Fdx[158] =                                                                                0;
                Fdx[173] =                                                                                0;
                Fdx[188] =                                                                                0;
                Fdx[203] =                                                                                0;
                Fdx[218] =                                                                                0;
                // 10 column
                Fdx[9] =                                0;
                Fdx[24] =                                0;
                Fdx[39] =                                0;
                Fdx[54] =  -dt*(x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9]);
                Fdx[69] =          -dt*(2*x[6]*x[9] + 2*x[7]*x[8]);
                Fdx[84] =           dt*(2*x[6]*x[8] - 2*x[7]*x[9]);
                Fdx[99] =                                0;
                Fdx[114] =                                0;
                Fdx[129] =                                0;
                Fdx[144] =                                1;
                Fdx[159] =                                0;
                Fdx[174] =                                0;
                Fdx[189] =                                0;
                Fdx[204] =                                0;
                Fdx[219] =                                0;
                // 11 column
                Fdx[10] =                                0;
                Fdx[25] =                                0;
                Fdx[40] =                                0;
                Fdx[55] =           dt*(2*x[6]*x[9] - 2*x[7]*x[8]);
                Fdx[70] =  -dt*(x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9]);
                Fdx[85] =          -dt*(2*x[6]*x[7] + 2*x[8]*x[9]);
                Fdx[100] =                                0;
                Fdx[115] =                                0;
                Fdx[130] =                                0;
                Fdx[145] =                                0;
                Fdx[160] =                                1;
                Fdx[175] =                                0;
                Fdx[190] =                                0;
                Fdx[205] =                                0;
                Fdx[220] =                                0;
                // 12 column
                Fdx[11] =                                0;
                Fdx[26] =                                0;
                Fdx[41] =                                0;
                Fdx[56] =          -dt*(2*x[6]*x[8] + 2*x[7]*x[9]);
                Fdx[71] =           dt*(2*x[6]*x[7] - 2*x[8]*x[9]);
                Fdx[86] =  -dt*(x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9]);
                Fdx[101] =                                0;
                Fdx[116] =                                0;
                Fdx[131] =                                0;
                Fdx[146] =                                0;
                Fdx[161] =                                0;
                Fdx[176] =                                1;
                Fdx[191] =                                0;
                Fdx[206] =                                0;
                Fdx[221] =                                0;
                // 13 column
                Fdx[12] =    0;
                Fdx[27] =    0;
                Fdx[42] =    0;
                Fdx[57] =    0;
                Fdx[72] =    0;
                Fdx[87] =    0;
                Fdx[102] =  -dt;
                Fdx[117] =    0;
                Fdx[132] =    0;
                Fdx[147] =    0;
                Fdx[162] =    0;
                Fdx[177] =    0;
                Fdx[192] =    1;
                Fdx[207] =    0;
                Fdx[222] =    0;
                // 14 column
                Fdx[13] =    0;
                Fdx[28] =    0;
                Fdx[43] =    0;
                Fdx[58] =    0;
                Fdx[73] =    0;
                Fdx[88] =    0;
                Fdx[103] =    0;
                Fdx[118] =  -dt;
                Fdx[133] =    0;
                Fdx[148] =    0;
                Fdx[163] =    0;
                Fdx[178] =    0;
                Fdx[193] =    0;
                Fdx[208] =    1;
                Fdx[223] =    0;
                // 15 column
                Fdx[14] =    0;
                Fdx[29] =    0;
                Fdx[44] =    0;
                Fdx[59] =    0;
                Fdx[74] =    0;
                Fdx[89] =    0;
                Fdx[104] =    0;
                Fdx[119] =    0;
                Fdx[134] =  -dt;
                Fdx[149] =    0;
                Fdx[164] =    0;
                Fdx[179] =    0;
                Fdx[194] =    0;
                Fdx[209] =    0;
                Fdx[224] =    1;
            }
            
            virtual void getCovarianceEstimation(float * Q) override
            {
              
                // 1 column
                Q[0] =  0;
                Q[15] =  0;
                Q[30] =  0;
                Q[45] =  0;
                Q[60] =  0;
                Q[75] =  0;
                Q[90] =  0;
                Q[105] =  0;
                Q[120] =  0;
                Q[135] =  0;
                Q[150] =  0;
                Q[165] =  0;
                Q[180] =  0;
                Q[195] =  0;
                Q[210] =  0;
                // 2 column
                Q[1] =  0;
                Q[16] =  0;
                Q[31] =  0;
                Q[46] =  0;
                Q[61] =  0;
                Q[76] =  0;
                Q[91] =  0;
                Q[106] =  0;
                Q[121] =  0;
                Q[136] =  0;
                Q[151] =  0;
                Q[166] =  0;
                Q[181] =  0;
                Q[196] =  0;
                Q[211] =  0;
                // 3 column
                Q[2] =  0;
                Q[17] =  0;
                Q[32] =  0;
                Q[47] =  0;
                Q[62] =  0;
                Q[77] =  0;
                Q[92] =  0;
                Q[107] =  0;
                Q[122] =  0;
                Q[137] =  0;
                Q[152] =  0;
                Q[167] =  0;
                Q[182] =  0;
                Q[197] =  0;
                Q[212] =  0;
                // 4 column
                Q[3] =  0;
                Q[18] =   0;
                Q[33] =   0;
                Q[48] =   0.0001;
                Q[63] =   0;
                Q[78] =   0;
                Q[93] =   0;
                Q[108] =   0;
                Q[123] =   0;
                Q[138] =   0;
                Q[153] =   0;
                Q[168] =   0;
                Q[183] =   0;
                Q[198] =   0;
                Q[213] =   0;
                // 5 column
                Q[4] =   0;
                Q[19] =  0;
                Q[34] =   0;
                Q[49] =   0;
                Q[64] =   0.0001;
                Q[79] =   0;
                Q[94] =   0;
                Q[109] =   0;
                Q[124] =   0;
                Q[139] =   0;
                Q[154] =   0;
                Q[169] =   0;
                Q[184] =   0;
                Q[199] =   0;
                Q[214] =   0;
                // 6 column
                Q[5] =   0;
                Q[20] =   0;
                Q[35] =  0;
                Q[50] =   0;
                Q[65] =   0;
                Q[80] =   0.0001;
                Q[95] =   0;
                Q[110] =   0;
                Q[125] =   0;
                Q[140] =   0;
                Q[155] =   0;
                Q[170] =   0;
                Q[185] =   0;
                Q[200] =   0;
                Q[215] =   0;
                // 7 column
                Q[6] = 0;
                Q[21] = 0;
                Q[36] = 0;
                Q[51] = 0;
                Q[66] = 0;
                Q[81] = 0;
                Q[96] = 0.0001;
                Q[111] = 0;
                Q[126] = 0;
                Q[141] = 0;
                Q[156] = 0;
                Q[171] = 0;
                Q[186] = 0;
                Q[201] = 0;
                Q[216] = 0;
                // 8 column
                Q[7] = 0;
                Q[22] = 0;
                Q[37] = 0;
                Q[52] = 0;
                Q[67] = 0;
                Q[82] = 0;
                Q[97] = 0;
                Q[112] = 0.0001;
                Q[127] = 0;
                Q[142] = 0;
                Q[157] = 0;
                Q[172] = 0;
                Q[187] = 0;
                Q[202] = 0;
                Q[217] = 0;
                // 9 column
                Q[8] = 0;
                Q[23] = 0;
                Q[38] = 0;
                Q[53] = 0;
                Q[68] = 0;
                Q[83] = 0;
                Q[98] = 0;
                Q[113] = 0;
                Q[128] = 0.0001;
                Q[143] = 0;
                Q[158] = 0;
                Q[173] = 0;
                Q[188] = 0;
                Q[203] = 0;
                Q[218] = 0;
                // 10 column
                Q[9] = 0;
                Q[24] = 0;
                Q[39] = 0;
                Q[54] = 0;
                Q[69] = 0;
                Q[84] = 0;
                Q[99] = 0;
                Q[114] = 0;
                Q[129] = 0;
                Q[144] = 0.0001;
                Q[159] = 0;
                Q[174] = 0;
                Q[189] = 0;
                Q[204] = 0;
                Q[219] = 0;
                // 11 column
                Q[10] = 0;
                Q[25] = 0;
                Q[40] = 0;
                Q[55] = 0;
                Q[70] = 0;
                Q[85] = 0;
                Q[100] = 0;
                Q[115] = 0;
                Q[130] = 0;
                Q[145] = 0;
                Q[160] = 0.0001;
                Q[175] = 0;
                Q[190] = 0;
                Q[205] = 0;
                Q[220] = 0;
                // 12 column
                Q[11] = 0;
                Q[26] = 0;
                Q[41] = 0;
                Q[56] = 0;
                Q[71] = 0;
                Q[86] = 0;
                Q[101] = 0;
                Q[116] = 0;
                Q[131] = 0;
                Q[146] = 0;
                Q[161] = 0;
                Q[176] = 0.0001;
                Q[191] = 0;
                Q[206] = 0;
                Q[221] = 0;
                // 13 column
                Q[12] = 0;
                Q[27] = 0;
                Q[42] = 0;
                Q[57] = 0;
                Q[72] = 0;
                Q[87] = 0;
                Q[102] = 0;
                Q[117] = 0;
                Q[132] = 0;
                Q[147] = 0;
                Q[162] = 0;
                Q[177] = 0;
                Q[192] = 0.0001;
                Q[207] = 0;
                Q[222] = 0;
                // 14 column
                Q[13] = 0;
                Q[28] = 0;
                Q[43] = 0;
                Q[58] = 0;
                Q[73] = 0;
                Q[88] = 0;
                Q[103] = 0;
                Q[118] = 0;
                Q[133] = 0;
                Q[148] = 0;
                Q[163] = 0;
                Q[178] = 0;
                Q[193] = 0;
                Q[208] = 0.0001;
                Q[223] = 0;
                // 15 column
                Q[14] = 0;
                Q[29] = 0;
                Q[44] = 0;
                Q[59] = 0;
                Q[74] = 0;
                Q[89] = 0;
                Q[104] = 0;
                Q[119] = 0;
                Q[134] = 0;
                Q[149] = 0;
                Q[164] = 0;
                Q[179] = 0;
                Q[194] = 0;
                Q[209] = 0;
                Q[224] = 0.0001;

            }

        protected:

            virtual void modifyState(eskf_state_t & state, float time) override
            {
                (void)time;

                // NB: We negate X, Y rates to simplify PID controller
                state.angularVelocities[0] =  _rates[0];
                state.angularVelocities[1] = -_rates[1];
                state.angularVelocities[2] = -_rates[2];
            }

            virtual bool ready(float time) override
            {
                (void)time;

                bool result = board->getIMU(_rates, _accels);

                return result;
            }
            
            virtual bool shouldUpdateESKF(float time) override
            {
                return true;
            }

        private:

            float _rates[3];
            float _accels[3];

    };  // class IMU

} // namespace
