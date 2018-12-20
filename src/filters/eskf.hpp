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

#define NNsta = 4   // Four nominal states: quaternion 
#define NEsta = 3   // Three error state values: angle error
#define Mobs = 3    // Three correction observations: accelerometer


#include "linalg.hpp"
#include "eskf_struct.hpp"

namespace hf {
  
  class ESKF {
    
    private:
    
      eskf_t eskf;
      
    protected:

  }; // class ESKF
  
} // namespace hf
