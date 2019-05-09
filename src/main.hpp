/*
   main.hpp : Bonadrone's hackflight entry point. This file creates a Hackflight
   instance taking into account the defined parameters stored in the EEPROM and 
   runs it. It acts as a wrapper that automatically creates the correct Hackflight
   instance as a function of the stored parameters.  

   Copyright (c) 2019 BonaDrone (www.bonadrone.com)
   Developed by: Pep Marti-Saumell (jmarti<at>bonadrone.com>) & Juan Gallostra Acin (jgallostra<at>bonadrone.com)

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MEReceiverHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <EEPROM.h>
#include "hackflight.hpp"

// Board, receiver and mixer used
#include "boards/bonadrone.hpp"
#include "receivers/sbus.hpp"
#include "mixers/quadx.hpp"

// Additional Sensors
// (Gyrometer and accelerometer are included by default)
#include <PMW3901.h>
#include "sensors/rangefinders/vl53l1x.hpp"
#include "sensors/opticalflow.hpp"

// Additional Controllers 
// (Rate is included by default as it is always required)
#include "pidcontrollers/level.hpp"
#include "pidcontrollers/althold.hpp"
#include "pidcontrollers/poshold.hpp"

// Change this as needed
#define SBUS_SERIAL Serial1

static constexpr uint8_t CHANNEL_MAP[6] = {2,0,1,3,5,4};

namespace hf {

    class HackflightWrapper : public Hackflight {

        private:

              // Params
              bool _hasPositioningBoard = false;
              bool _isMosquito90 = false;
              bool _calibrateESC = false;
              bool _txCalibrated = false;
              // Connection status (false unless proven otherwise)
              bool _positionBoardConnected = false;
              // Rate PID params
              float _gyroRollP;
              float _gyroRollI;
              float _gyroRollD;              
              float _gyroPitchP;
              float _gyroPitchI;
              float _gyroPitchD;
              float _gyroYawP;
              float _gyroYawI;
              float _demandsToRate;
              // Level Params
              float _levelP;
              // AltHold params
              float _altHoldP;
              float _altHoldVelP;
              float _altHoldVelI;
              float _altHoldVelD;
              float _minAltitude;
              // PosHold params
              float _posHoldVelP;
              float _posHoldVelI;
              float _posHoldVelD;
              // range params
              float _rx;
              float _ry;
              float _rz;
              // transmitter trims
              float _min[4] = {0,0,0,0}; 
              float _center[3] = {0,0,0}; 
              float _max[4] = {0,0,0,0}; 

              // Required objects to run Hackflight
              hf::MixerQuadX mixer;
              hf::SBUS_Receiver rc = hf::SBUS_Receiver(CHANNEL_MAP, SERIAL_SBUS, &SBUS_SERIAL);
                  
              void loadParameters(void)
              {
                  // Parameters are currently defined in Hackflight because is
                  // the one in charge of serial communications. Hence, MSP
                  // message handlers are defined there.
                  uint8_t config = EEPROM.read(GENERAL_CONFIG);
                  _isMosquito90 = (config >> MOSQUITO_VERSION) & 1;
                  _hasPositioningBoard = (config >> POSITIONING_BOARD) & 1;
                  _calibrateESC = (config >> CALIBRATE_ESC) & 1;
                  _txCalibrated = (config >> TX_CALIBRATED) & 1;
                  // Load Rate PID parameters (each float is 4 bytes)
                  EEPROM.get(PID_CONSTANTS, _gyroRollP);
                  EEPROM.get(PID_CONSTANTS + 1 * sizeof(float), _gyroRollI);
                  EEPROM.get(PID_CONSTANTS + 2 * sizeof(float), _gyroRollD);
                  EEPROM.get(PID_CONSTANTS + 3 * sizeof(float), _gyroPitchP);
                  EEPROM.get(PID_CONSTANTS + 4 * sizeof(float), _gyroPitchI);
                  EEPROM.get(PID_CONSTANTS + 5 * sizeof(float), _gyroPitchD);
                  EEPROM.get(PID_CONSTANTS + 6 * sizeof(float), _gyroYawP);
                  EEPROM.get(PID_CONSTANTS + 7 * sizeof(float), _gyroYawI);
                  EEPROM.get(PID_CONSTANTS + 8 * sizeof(float), _demandsToRate);
                  EEPROM.get(PID_CONSTANTS + 9 * sizeof(float), _levelP);
                  EEPROM.get(PID_CONSTANTS + 10 * sizeof(float), _altHoldP);
                  EEPROM.get(PID_CONSTANTS + 11 * sizeof(float), _altHoldVelP);
                  EEPROM.get(PID_CONSTANTS + 12 * sizeof(float), _altHoldVelI);
                  EEPROM.get(PID_CONSTANTS + 13 * sizeof(float), _altHoldVelD);
                  EEPROM.get(PID_CONSTANTS + 14 * sizeof(float), _minAltitude);
                  EEPROM.get(PID_CONSTANTS + 15 * sizeof(float), _posHoldVelP);
                  EEPROM.get(PID_CONSTANTS + 16 * sizeof(float), _posHoldVelI);
                  EEPROM.get(PID_CONSTANTS + 17 * sizeof(float), _posHoldVelD);
                  EEPROM.get(RANGE_PARAMS, _rx);
                  EEPROM.get(RANGE_PARAMS + 1 * sizeof(float), _ry);
                  EEPROM.get(RANGE_PARAMS + 2 * sizeof(float), _rz);
                  EEPROM.get(TRANSMITER_TRIMS, _min[0]);
                  EEPROM.get(TRANSMITER_TRIMS + 1 * sizeof(float), _max[0]);
                  EEPROM.get(TRANSMITER_TRIMS + 2 * sizeof(float), _min[1]);
                  EEPROM.get(TRANSMITER_TRIMS + 3 * sizeof(float), _center[0]);
                  EEPROM.get(TRANSMITER_TRIMS + 4 * sizeof(float), _max[1]);
                  EEPROM.get(TRANSMITER_TRIMS + 5 * sizeof(float), _min[2]);
                  EEPROM.get(TRANSMITER_TRIMS + 6 * sizeof(float), _center[1]);
                  EEPROM.get(TRANSMITER_TRIMS + 7 * sizeof(float), _max[2]);
                  EEPROM.get(TRANSMITER_TRIMS + 8 * sizeof(float), _min[3]);
                  EEPROM.get(TRANSMITER_TRIMS + 9 * sizeof(float), _center[2]);
                  EEPROM.get(TRANSMITER_TRIMS + 10 * sizeof(float), _max[3]);
              }
              
              void calibrateESCsStandard(void)
              {
                  // Motor pins
                  int MOTOR_PINS[4] = {3, 4, 5, 6};
                
                  // PWM Values
                  uint16_t BASELINE = 1000;
                  uint16_t MIDVAL   = 1500;
                  uint16_t MAXVAL   = 2000;

                  uint8_t LED_R = 25;
                  uint8_t LED_G = 26;
                  uint8_t LED_B = 38;

                  pinMode(LED_R, OUTPUT);  
                  pinMode(LED_G, OUTPUT);
                  pinMode(LED_B, OUTPUT);

                  digitalWrite(LED_R, HIGH);
                  digitalWrite(LED_G, HIGH);
                  digitalWrite(LED_B, HIGH);
    
                  digitalWrite(LED_R, LOW);
                  delay(500);
                  digitalWrite(LED_R, HIGH);
                  digitalWrite(LED_G, LOW);
                  delay(500);
                  digitalWrite(LED_G, HIGH);
                  digitalWrite(LED_B, LOW);
                  delay(500);
    
                  // Blue LED ON when calibrating (first part)
                  for (int k=0; k<4; ++k)
                  {
                      pinMode(MOTOR_PINS[k], OUTPUT);    
                      analogWrite(MOTOR_PINS[k], MAXVAL >> 3);
                  }
                  delay(10000);
                  digitalWrite(LED_B, HIGH);
    
                  // Green LED ON when calibrating (second part)
                  digitalWrite(LED_G, LOW);
                  for (int k=0; k<4; ++k)
                  {
                      analogWrite(MOTOR_PINS[k], BASELINE >> 3);
                  }
                  delay(10000);
                  digitalWrite(LED_G, HIGH);
              }

              void calibrateESCsMultiShot(void)
              {
                  // Motor pins
                  int MOTOR_PINS[4] = {3, 4, 5, 6};

                  // PWM Values
                  uint16_t BASELINE = 100;
                  uint16_t MAXVAL   = 500;
                  uint8_t LED_R = 25;
                  uint8_t LED_G = 26;
                  uint8_t LED_B = 38;
      
                  pinMode(LED_R, OUTPUT);  
                  pinMode(LED_G, OUTPUT);
                  pinMode(LED_B, OUTPUT);
                  
                  // LED sequence that signals begin of calibration
                  digitalWrite(LED_R, HIGH);
                  digitalWrite(LED_G, HIGH);
                  digitalWrite(LED_B, HIGH);

                  digitalWrite(LED_R, LOW);
                  delay(500);
                  digitalWrite(LED_R, HIGH);
                  digitalWrite(LED_G, LOW);
                  delay(500);
                  digitalWrite(LED_G, HIGH);
                  digitalWrite(LED_B, LOW);
                  delay(500);

                  // Blue LED ON when calibrating (first part)
                  for (int k=0; k<4; ++k)
                  {
                      pinMode(MOTOR_PINS[k], OUTPUT);
                      analogWriteFrequency(MOTOR_PINS[k], 2000);
                      analogWriteRange(MOTOR_PINS[k], 10000);
                      analogWrite(MOTOR_PINS[k], MAXVAL);
                  }

                  delay(10000);
                  digitalWrite(LED_B, HIGH);

                  // Green LED ON when calibrating (second part)
                  digitalWrite(LED_G, LOW);
                  for (int k=0; k<4; ++k)
                  {
                      analogWrite(MOTOR_PINS[k], BASELINE);
                  }
                  delay(10000);
                  digitalWrite(LED_G, HIGH);
              }

              void trimReceiver(void)
              {
                  // Set trims for R, P, Y
                  for (int k=0; k<3; k++)
                  {
                    float m_pos = 1.0 / (_max[k+1] - _center[k]); 
                    float n_pos =  - (m_pos * _center[k]);
                    float m_neg = - 1.0 / (_min[k+1] - _center[k]); 
                    float n_neg =  - (m_neg * _center[k]);
                    rc.setTrim(m_pos, n_pos, m_neg, n_neg, k+1);
                  }
                  // Set Throttle trims
                  float m = 2.0 / (_max[0] - _min[0]);
                  float n = 1.0 - m*_max[0];
                  rc.setTrim(m, n, m, n, 0);
              }

        protected:


        public:

            hf::Hackflight h;

            void init()
            {  
              
                loadParameters();
                // begin the serial port for the ESP32
                Serial4.begin(115200);
                
                rc.setCalibrationStatus(_txCalibrated);
                // Trim receiver via software
                trimReceiver();

                // Instantiate controllers after loading parameters
                hf::Rate * ratePid = new hf::Rate(
                  _gyroRollP,
                  _gyroRollI,
                  _gyroRollD,
                  _gyroPitchP,
                  _gyroPitchI,
                  _gyroPitchD,                  
                  _gyroYawP,
                  _gyroYawI,
                  _demandsToRate);
                ratePid->setPIDType(RATE);
                
                hf::Level * level = new hf::Level(_levelP);  // Pitch Level P
                level->setDemandsToRate(_demandsToRate);
                level->setPIDType(LEVEL);

                // Add additional sensors
                if (_hasPositioningBoard)
                {
                    hf::AltitudeHold * althold = new hf::AltitudeHold(
                        _altHoldP,      // Altitude Hold P -> this will set velTarget to 0
                        _altHoldVelP,   // Altitude Hold Velocity P
                        _altHoldVelI,   // Altitude Hold Velocity I
                        _altHoldVelD,   // Altitude Hold Velocity D
                        _minAltitude);  // Min altitude
                    althold->setPIDType(ALTHOLD);
                    h.addPidController(althold, 0);

                    hf::PositionHold * poshold = new hf::PositionHold(
                        0.0,            // Position Hold P -> this will set velTarget to 0
                        _posHoldVelP,   // Position Hold Velocity P
                        _posHoldVelI,   // Position Hold Velocity I
                        _posHoldVelD);   // Position Hold Velocity D
                    poshold->setPIDType(POSHOLD);
                    h.addPidController(poshold, 0);

                }

                // 0 means the controller will always be active, but by changing
                // that number it can be linked to a different aux state
                h.addPidController(level, 0);

                if (_isMosquito90) {
                    h.init(new hf::BonadroneBrushed(), &rc, &mixer, ratePid);
                } else {
                    if (_calibrateESC)
                    {
                      calibrateESCsMultiShot();
                      uint8_t config = EEPROM.read(GENERAL_CONFIG);
                      EEPROM.write(GENERAL_CONFIG, config & ~(1 << CALIBRATE_ESC));
                    }
                    h.init(new hf::BonadroneMultiShot(), &rc, &mixer, ratePid);
                }
                // Add additional sensors
                if (_hasPositioningBoard)
                {
                    hf::VL53L1X_Rangefinder * rangefinder = new hf::VL53L1X_Rangefinder() ;
                    bool _rangeConnected = rangefinder->begin();
                    h.addSensor(rangefinder);
                    h.eskf.addSensorESKF(rangefinder);

                    hf::OpticalFlow * opticalflow = new hf::OpticalFlow();
                    bool _opticalConnected = opticalflow->begin();
                    h.addSensor(opticalflow);
                    h.eskf.addSensorESKF(opticalflow);
                    
                    _positionBoardConnected = _rangeConnected & _opticalConnected;
                }

                // Set parameters in hackflight instance so that 
                // they can be queried via MSP
                h.setParams(_hasPositioningBoard, _isMosquito90, _positionBoardConnected);
                
                // XXX Debuging
                // hardcodeMission();
                
            } // init
            
            // XXX Testing
            void hardcodeMission(void)
            {
              EEPROM.write(150, 1);   // Arm
              // EEPROM.write(151, 0);
              EEPROM.write(151, 4);   // Takeoff to 1 m
              EEPROM.write(152, 1);
              EEPROM.write(153, 11);  // Hover for 3 seconds
              EEPROM.write(154, 3);
              EEPROM.write(155, 3);   // Land
              // EEPROM.write(155, 0);
              EEPROM.write(156, 2);   // Disarm
            }

    }; // class HackflightWrapper

} // namespace
