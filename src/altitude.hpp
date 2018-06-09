/*
    altitude.hpp: Altitude estimation via barometer/accelerometer fusion

    Adapted from

    https://github.com/multiwii/baseflight/blob/master/src/imu.c

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

# pragma once

#include "datatypes.hpp"
#include "sensors/barometer.hpp"
#include "sensors/imu.hpp"
#include "filter.hpp"

namespace hf {

  class AltitudeEstimator {
    private:
      // sensor abstractions
      Barometer baro = Barometer();
      IMU imu = IMU();
      // required parameters for the filters used for the estimations
      // sensor's standard deviations
      float sigma_accel = 0.2;
      float sigma_gyro = 0.2;
      float sigma_baro = 5;
      // gravity
      float g = 9.81;
      // Acceleration markov chain model state transition constant
      float ca = 0.5;
      // Zero-velocity update acceleration threshold
      float accel_threshold = 0.3;
      // Sampling period
      float previousTime = millis();
      // required filters for altitude and vertical velocity estimation
      KalmanFilter kalman = KalmanFilter(ca, sigma_gyro, sigma_accel);
      ComplementaryFilter complementary = ComplementaryFilter(sigma_accel, sigma_baro, accel_threshold);
      // Estimated past vertical acceleration
      float pastVerticalAccel = 0;
      // fake calibration
      int count = 0;

    public:
      // estimated altitude and vertical velocity
      float estimatedAltitude = 0;
      float estimatedVelocity = 0;

      void init(void)
      {
          baro.init();
      }

      void updateBaro(bool armed, float pressure)
      {
          baro.update(pressure);
          // Calibrate barometer when the drone is resting
          if (!armed && count < 100){
            baro.calibrate();
            count++;
            return;
          }
          return;
      }

      void updateAcceleration(float _accel[3])
      {
          imu.updateAcceleration(_accel);
      }

      void updateGyro(float _gyro[3])
      {
          imu.updateGyro(_gyro);
      }

      float estimate()
      {
          float currentTime = millis();
          float verticalAccel = kalman.estimate( imu.gyro,
                                                 imu.accel,
                                                 (currentTime-previousTime)/1000.0);
          Serial.println(verticalAccel);
          complementary.estimate(& estimatedVelocity,
                                 & estimatedAltitude,
                                 baro.getAltitude(),
                                 pastVerticalAccel,
                                 (currentTime-previousTime)/1000.0);
          Serial.println(estimatedAltitude);
          pastVerticalAccel = verticalAccel;
          previousTime = currentTime;
          return estimatedAltitude;
      }

  }; // class AltitudeEstimator

  class AltitudeHold {

    private:
      // State variables
      bool holding;
      float referenceAltitude;
      float initialThrottle; // [0, 1]
      float pid;

    public:
      // Altitude estimation
      AltitudeEstimator altitudeEstimator = AltitudeEstimator();

      AltitudeHold()
      {

      }

      void handleAuxSwitch(demands_t & demands)
      {
          if (demands.aux > 0) {
            holding = true;
            referenceAltitude = altitudeEstimator.estimate();
            // This is the reference throttle to hover
            // at the current altitude
            initialThrottle = demands.throttle;
        }
        else {
            holding = false;
        }
      }

      // If for some reason any of the demands received from
      // the transmitter have to be modified we do it here
      void modifyDemands(demands_t & demands)
      {
          if (holding) {
            demands.throttle = initialThrottle + pid;
          }
      }

      void update(bool armed, float pressure)
      {
        altitudeEstimator.updateBaro(armed, pressure);
      }

  }; // class AltitudeHold
} // namespace hf
