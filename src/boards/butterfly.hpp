/*
   butterfly.hpp : Implementation of Hackflight Board routines for Butterfly
                   dev board + MPU9250 IMU + MS5637 barometer + brushless motors

   Additional libraries required: https://github.com/simondlevy/MPU9250
                                  https://github.com/simondlevy/MS5637

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

#include <Wire.h>
#include <Servo.h>

#include <MPU9250_Passthru.h> 

#include "filters.hpp"
#include "hackflight.hpp"
#include "softquat.hpp"

namespace hf {

    // Interrupt support 
    static bool gotNewData;
    static void interruptHandler()
    {
        gotNewData = true;
    }

    class Butterfly : public SoftwareQuaternionBoard {

        private:

            // Motor pins
            const uint8_t MOTOR_PINS[4] = {3, 4, 5, 6};

            // Min, max PWM values
            const uint16_t PWM_MIN = 1000;
            const uint16_t PWM_MAX = 2000;

            // Butterfly board follows Arduino standard for LED pin
            const uint8_t LED_PIN = 13;

            // MPU9250 add-on board has interrupt on Butterfly pin 8
            const uint8_t INTERRUPT_PIN = 8;

            // Paramters to experiment with ------------------------------------------------------------------------

            // MPU9250 full-scale settings
            static const MPUIMU::Ascale_t ASCALE  = MPUIMU::AFS_8G;
            static const MPUIMU::Gscale_t GSCALE  = MPUIMU::GFS_2000DPS;
            static const MPU9250::Mscale_t MSCALE = MPU9250::MFS_16BITS;
            static const MPU9250::Mmode_t  MMODE  = MPU9250::M_100Hz;

            // SAMPLE_RATE_DIVISOR: (1 + SAMPLE_RATE_DIVISOR) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so 
            // SAMPLE_RATE_DIVISOR = 0 means 1 kHz sample rate for both accel and gyro, 4 means 200 Hz, etc.
            static const uint8_t SAMPLE_RATE_DIVISOR = 0;         

            // Instance variables -----------------------------------------------------------------------------------

            // Use the MPU9250 in pass-through mode
            MPU9250_Passthru _imu = MPU9250_Passthru(ASCALE, GSCALE, MSCALE, MMODE, SAMPLE_RATE_DIVISOR);

       protected:

            void delayMilliseconds(uint32_t msec)
            {
                delay(msec);
            }

            void setLed(bool isOn)
            { 
                digitalWrite(LED_PIN, isOn ? LOW : HIGH);
            }

            uint8_t serialAvailableBytes(void)
            {
                return Serial.available();
            }

            uint8_t serialReadByte(void)
            {
                return Serial.read();
            }

            void serialWriteByte(uint8_t c)
            {
                Serial.write(c);
            }

            void writeMotor(uint8_t index, float value)
            {
                analogWrite(MOTOR_PINS[index], (uint16_t)(PWM_MIN+value*(PWM_MAX-PWM_MIN)) >> 3);
            }

            virtual uint32_t getMicroseconds(void) override
            {
                return micros();
            }

            void delaySeconds(float sec)
            {
                delay((uint32_t)(1000*sec));
            }


            bool imuRead(void)
            {
                if (gotNewData) {

                    gotNewData = false;

                    if (_imu.checkNewAccelGyroData()) {

                        _imu.readAccelerometer(_ay, _ax, _az);
                        _imu.readGyrometer(_gy, _gx, _gz);

                        _gx = -_gx;

                        return true;

                    } 
                } 

                return false;
            }

            void updateQuaternion(float deltat) 
            {                   
                _quaternionFilter.update(-_ay, _ax, _az, _gy, _gx, -_gz, deltat); 
            }

        public:

            Butterfly(void)
            {
                // Begin serial comms
                Serial.begin(115200);

                // Setup LED pin and turn it off
                pinMode(LED_PIN, OUTPUT);
                digitalWrite(LED_PIN, HIGH);

                // Set up the interrupt pin, it's set as active high, push-pull
                pinMode(INTERRUPT_PIN, INPUT);
                attachInterrupt(INTERRUPT_PIN, interruptHandler, RISING);  

                // Connect to the ESCs and send them the baseline values
                for (uint8_t k=0; k<4; ++k) {
                  pinMode(MOTOR_PINS[k], OUTPUT);
                  analogWrite(MOTOR_PINS[k], PWM_MIN>>3);
                }

                // Start I^2C
                Wire.begin();
                Wire.setClock(400000); // I2C frequency at 400 kHz

                // Wait a bit
                delay(100);

                // Start the MPU9250
                switch (_imu.begin()) {

                    case MPUIMU::ERROR_IMU_ID:
                        error("Bad IMU device ID");
                        break;
                    case MPUIMU::ERROR_MAG_ID:
                        error("Bad magnetometer device ID");
                        break;
                    case MPUIMU::ERROR_SELFTEST:
                        //error("Failed self-test");
                        break;
                    default:
                        break;
                }

                // Do general real-board initialization
                RealBoard::init();
            }


    }; // class Butterfly

    void Board::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
