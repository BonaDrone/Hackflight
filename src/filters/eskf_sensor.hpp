/*
  eskf_sensor.hpp: Sensor abstraction for ESKF
 
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

#include "linalg.hpp"
#include "sensor.hpp"

// Inheritance structure will be:
// RealSensor : PeripheralSensor/SurfaceMountSensor : ESKF_Sensor : Sensor
namespace hf {

    class ESKF_Sensor : public Sensor {
        
      public:
        
        
        ESKF_Sensor(bool isestimation, bool iscorrection)
        {
            isEstimation = isestimation;
            isCorrection = iscorrection;
        }
        
        bool isEstimation;
        bool isCorrection;
        
        // This methods should be overriden by sensors that estimate
        virtual void integrateNominalState(float * fx, float * x, float * q, double dt) { (void)fx; (void)x; }
        
        virtual void getJacobianErrors(float * Fdx, float * x, float * q, double dt) { (void)Fdx; (void)x; }
        
        virtual void getCovarianceEstimation(float * Q) { (void)Q; }

        // This methods should be overriden by sensors that correct estimations
        virtual bool getJacobianObservation(float * H, float * x, float * q) { (void)H; (void)x; }
        
        virtual bool getInnovation(float * z, float * x, float * q) { (void)z; (void)x; }
        
        virtual void getCovarianceCorrection(float * N) { (void)N; }
        
        // This method should be overriden if control over update/correct
        // frequency is desired. It should return true when the sensor is
        // ready to update/correct the state and false otherwise
        virtual bool shouldUpdateESKF(float time, state_t & state) { return true; }
        
        // This method might be overriden to return the appropriate inverse of Z
        virtual bool Zinverse(float * Z, float * invZ)
        {
          float tmp[Mobs];
          if (cholsl(Z, invZ, tmp, Mobs))
          { 
            return 1;
          }
          return 0;
        }
        
        // This method should be overriden if the sensor needs some measures (such as rates) that
        // are not accessible from the state
        virtual void getMeasures(eskf_state_t & state) {(void)state;}
        
        virtual bool isOpticalFlow(void) { return false; }
        
        virtual void setSensorData(float data[3]) {(void)data;}

    }; // class ESKF_Sensor

} // namespace hf