/*
  eskf_sensor.hpp: Sensor abstraction for ESKF
 
  Copyright (C) 2018 Juan Gallostra, Pep Martí Saumell
  
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
#include "sensor.hpp"

// Inheritance structure will be:
// RealSensor : PeripheralSensor/SurfaceMountSensor : ESKF_Sensor : Sensor
namespace hf {

    class ESKF_Sensor : public Sensor {
      
      private:

        uint8_t _observationRows = 0;
        
      protected:

        // XXX After debugging, this methods should become pure virtual methods
        virtual void getJacobianObservation(Matrix * H, Matrix * x, int errorStates) { (void)H; (void)x; }
        
        virtual void getInnovation(Matrix * z, Matrix * x) { (void)z; }
        
        virtual void getCovarianceCorrection(Matrix * N) { (void)N; }

      public:
        
        uint8_t setObservationRows(uint8_t rows)
        {
            _observationRows = rows;
        }
        
        uint8_t getObservationRows(void)
        {
            return _observationRows;
        }

    }; // class ESKF_Sensor

} // namespace hf