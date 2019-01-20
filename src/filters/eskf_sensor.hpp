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
        
      public:
        
        
        ESKF_Sensor(bool isestimation, bool iscorrection)
        {
            isEstimation = isestimation;
            isCorrection = iscorrection;
        }
        
        bool isEstimation;
        bool isCorrection;
        
        // This methods should be overriden by sensors that estimate
        virtual void getJacobianModel(float * Fx, float * x, double dt) { (void)Fx; }
        
        virtual void getJacobianErrors(float * Fdx, float * x, double dt) { (void)Fdx; }
        
        virtual void getCovarianceEstimation(float * Q) { (void)Q; }

        // This methods should be overriden by sensors that correct estimations
        virtual void getJacobianObservation(float * H, float * x) { (void)H; (void)x; }
        
        virtual void getInnovation(float * z, float * x) { (void)z; }
        
        virtual void getCovarianceCorrection(float * N) { (void)N; }
        
        // This method should be overriden by all sensors and return true when
        // the sensor is ready to update/correct the state and false otherwise
        virtual bool shouldUpdateESKF(float time) { return true; }

    }; // class ESKF_Sensor

} // namespace hf