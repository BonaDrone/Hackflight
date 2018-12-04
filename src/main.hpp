/*
   main.hpp : Bonadrone's hackflight entry point. This file creates a Hackflight
   instance taking into account the defined parameters stored in the EEPROM and 
   runs it. It acts as a wrapper that automatically creates the correct Hackflight
   instance as a function of the stored parameters.  

   Copyright (c) 2018 Juan Gallostra & Simon D. Levy

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
#include <VL53L1X.h>
#include "sensors/rangefinders/vl53l1x.hpp"
#include "sensors/opticalflow.hpp"

// Additional Controllers 
// (Rate is included by default as it is always required)
#include "pidcontrollers/level.hpp"

// Change this as needed
#define SBUS_SERIAL Serial1

static constexpr uint8_t CHANNEL_MAP[6] = {2,0,1,3,5,4};

namespace hf {

    class HackflightWrapper : public Hackflight {

        private:

              // Params
              bool hasPositioningBoard = false;
              bool isMosquito90 = false;
          
              // Map parameters to EEPROM addresses
              static const uint8_t MOSQUITO_VERSION  = 0;
              static const uint8_t POSITIONING_BOARD = 1;
                            
              // Required objects to run Hackflight 
              hf::Hackflight h;
              hf::MixerQuadX mixer;
              hf::SBUS_Receiver rc = hf::SBUS_Receiver(CHANNEL_MAP, SERIAL_SBUS, &SBUS_SERIAL);
              
              // Controllers
              hf::Rate ratePid = hf::Rate(
                  0.06f,  // Gyro Roll/Pitch P
                  0.01f,  // Gyro Roll/Pitch I
                  0.00f,  // Gyro Roll/Pitch D
                  0.06f,  // Gyro yaw P
                  0.01f,  // Gyro yaw I
                  5.00f); // Demands to rate

              hf::Level level = hf::Level(0.30f);  // Pitch Level P
                  
              void loadParameters(void)
              {
                  isMosquito90 = bool(EEPROM.read(MOSQUITO_VERSION));
                  hasPositioningBoard = bool(EEPROM.read(POSITIONING_BOARD));
              }

        protected:


        public:

            void init()
            {  
              
                loadParameters();
                // begin the serial port for the ESP32
                Serial4.begin(115200);

                // Trim receiver via software
                rc.setTrimRoll(-0.0030506f);
                rc.setTrimPitch(-0.0372178f);
                rc.setTrimYaw(-0.0384381f);

                // 0 means the controller will always be active, but by changing
                // that number it can be linked to a different aux state
                h.addPidController(&level, 0);
                
                if (isMosquito90) {
                    h.init(new hf::BonadroneBrushed(), &rc, &mixer, &ratePid);
                } else {
                    h.init(new hf::BonadroneMultiShot(), &rc, &mixer, &ratePid);
                }
                // Add additional sensors
                if (hasPositioningBoard)
                {
                    hf::VL53L1X_Rangefinder rangefinder;
                    rangefinder.begin();
                    h.addSensor(&rangefinder);
                    
                    hf::OpticalFlow opticalflow;
                    opticalflow.begin();
                    h.addSensor(&opticalflow);
                }
                
            } // init

            void update(void)
            {
                h.update();
            } // update 

    }; // class HackflightWrapper

} // namespace
