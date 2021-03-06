/*
   hackflight.hpp : general header, plus init and update methods

   Copyright (c) 2018 Simon D. Levy
   Contributed to by Alec Singer

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

#include <cmath>

#include "sensor.hpp"
#include "board.hpp"
#include "mspparser.hpp"
#include "mixer.hpp"
#include "receiver.hpp"
#include "debug.hpp"
#include "datatypes.hpp"
#include "pidcontroller.hpp"
#include "pidcontrollers/rate.hpp"
#include "sensors/peripheral.hpp"
#include "sensors/gyrometer.hpp"
#include "sensors/quaternion.hpp"

#include "planner.hpp"

namespace hf {

    class Hackflight : public MspParser {

        private: 

            // XXX use a proper version formating
            uint8_t _firmwareVersion = 0;

            // Passed to Hackflight::init() for a particular build
            Board      * _board;
            Receiver   * _receiver;
            Rate       * _ratePid;
            Mixer      * _mixer;
            Planner      planner;

            // PID controllers
            PID_Controller * _pid_controllers[256];
            uint8_t _pid_controller_count = 0;

            // Mandatory sensors on the board
            Gyrometer _gyrometer;
            Quaternion _quaternion; // not really a sensor, but we treat it like one!

            // Additional sensors 
            Sensor * _sensors[256];
            uint8_t _sensor_count;

            // Vehicle state
            state_t _state;

            // Demands sent to mixer
            demands_t _demands;

            // Safety
            bool _safeToArm;
            bool _failsafe;

            // Support for headless mode
            float _yawInitial;

            bool safeAngle(uint8_t axis)
            {
                return fabs(_state.eulerAngles[axis]) < _ratePid->maxArmingAngle;
            }

            void checkQuaternion(void)
            {
                // Some quaternion filters may need to know the current time
                float time = _board->getTime();

                // If quaternion data ready
                if (_quaternion.ready(time)) {

                    // Update state with new quaternion to yield Euler angles
                    _quaternion.modifyState(_state, time);

                    // Synch serial comms to quaternion check
                    _receiver->_gotNewFrame = false;
                    doSerialComms();
                }
            }

            void checkGyrometer(void)
            {
                // Some gyrometers may need to know the current time
                float time = _board->getTime();

                // If gyrometer data ready
                if (_gyrometer.ready(time)) {

                    // Update state with gyro rates
                    _gyrometer.modifyState(_state, time);

                    // For PID control, start with demands from receiver
                    memcpy(&_demands, &_receiver->demands, sizeof(demands_t));

                    // Synch PID controllers to gyro update
                    runPidControllers();

                    // Sync failsafe to gyro loop
                    checkFailsafe();

                    // Use updated demands to run motors
                    if (_state.armed && !_failsafe && !_receiver->throttleIsDown()) {
                        _mixer->runArmed(_demands);
                    }
                }
            }

            void runPidControllers(void)
            {
                // Each PID controllers is associated with at least one auxiliary switch state
                uint8_t auxState = _receiver->getAux1State();

                // Some PID controllers should cause LED to flash when they're active
                bool shouldFlash = false;

                for (uint8_t k=0; k<_pid_controller_count; ++k) {

                    PID_Controller * pidController = _pid_controllers[k];

                    float currentTime = _board->getTime();

                    // XXX we should allow associating PID controllers with particular aux states
                    if (pidController->auxState <= auxState) {  

                        if (pidController->modifyDemands(_state, _demands, currentTime) && pidController->shouldFlashLed()) {
                            shouldFlash = true;
                        }
                    }
                }

                // Flash LED for certain PID controllers
                _board->flashLed(shouldFlash);
            }


            void checkFailsafe(void)
            {
                if (_state.armed &&
                   (_receiver->lostSignal(_receiver->_bypassReceiver) || _board->isBatteryLow())) {
                    _mixer->cutMotors();
                    _state.armed = false;
                    _failsafe = true;
                    _board->showArmedStatus(false);
                    if (_receiver->_lostSignal)
                    {
                      _receiver->_bypassReceiver = false;
                    }
                }
            } 

            void checkReceiver(void)
            {
                // Check whether receiver data is available
                if (!_receiver->getDemands(_state.eulerAngles[AXIS_YAW] - _yawInitial)) return;

                // Update ratePid with cyclic demands
                _ratePid->updateReceiver(_receiver->demands, _receiver->throttleIsDown());

                // Disarm
                if (_state.armed && !_receiver->getAux2State()) {
                    _state.armed = false;
                } 

                // Avoid arming if aux2 switch down on startup
                if (!_safeToArm) {
                    _safeToArm = !_receiver->getAux2State();
                }

                // Arm (after lots of safety checks!)
                if (    _safeToArm &&
                        !_state.armed && 
                        _receiver->throttleIsDown() &&
                        _receiver->getAux2State() && 
                        !_failsafe && 
                        safeAngle(AXIS_ROLL) && 
                        safeAngle(AXIS_PITCH)) {
                    _state.armed = true;
                    _yawInitial = _state.eulerAngles[AXIS_YAW]; // grab yaw for headless mode
                }

                // Cut motors on throttle-down
                if (_state.armed && _receiver->throttleIsDown()) {
                    _mixer->cutMotors();
                }

                // Set LED based on arming status
                _board->showArmedStatus(_state.armed);

            } // checkReceiver

            void doSerialComms(void)
            {
                _board->setSerialFlag();
                while (_board->serialAvailableBytes() > 0) {

                    if (MspParser::parse(_board->serialReadByte())) {
                        _board->reboot();
                    }
                }

                while (MspParser::availableBytes() > 0) {
                    _board->serialWriteByte(MspParser::readByte());
                }

                // Support motor testing from GCS
                if (!_state.armed) {
                    _mixer->runDisarmed();
                }
            }

            void checkOptionalSensors(void)
            {
                for (uint8_t k=0; k<_sensor_count; ++k) {
                    Sensor * sensor = _sensors[k];
                    float time = _board->getTime();
                    if (sensor->ready(time)) {
                        sensor->modifyState(_state, time);
                    }
                }
            }

            void add_sensor(Sensor * sensor)
            {
                _sensors[_sensor_count++] = sensor;
            }

            void checkPlanner(void)
            {
              if (_state.executingMission == false) return;
              
            }

            // XXX only for debuging purposes
            void readEEPROM()
            {
                int address = 0;
                uint8_t value;
                while (true)
                {
                    // read a byte from the current address of the EEPROM
                    value = EEPROM.read(address);
                    Serial.print(address);
                    Serial.print("\t");
                    Serial.print(value);
                    Serial.println();
                    address = address + 1;
                    if (address == EEPROM.length()) {
                      address = 0;
                      break;
                    }
                  }
            }

        protected:

            // Map parameters to EEPROM addresses
            static const uint8_t GENERAL_CONFIG    = 0;
            static const uint8_t PID_CONSTANTS     = 1;
            // booleans values are stored as the bits of the byte at address 0
            static const uint8_t MOSQUITO_VERSION  = 0;
            static const uint8_t POSITIONING_BOARD = 1;


            virtual void handle_SET_ARMED_Request(uint8_t  flag)
            {
                if (flag) {  // got arming command: arm only if throttle is down
                    if (_receiver->throttleIsDown()) {
                        _state.armed = true;
                    }
                }
                else {          // got disarming command: always disarm
                    _state.armed = false;
                }
            }

            virtual void handle_RC_NORMAL_Request(float & c1, float & c2, float & c3, float & c4, float & c5, float & c6) override
            {
                c1 = _receiver->getRawval(0);
                c2 = _receiver->getRawval(1);
                c3 = _receiver->getRawval(2);
                c4 = _receiver->getRawval(3);
                c5 = _receiver->getRawval(4);
                c6 = _receiver->getRawval(5);
            }

            virtual void handle_ATTITUDE_RADIANS_Request(float & roll, float & pitch, float & yaw) override
            {
                roll  = _state.eulerAngles[0];
                pitch = _state.eulerAngles[1];
                yaw   = _state.eulerAngles[2];
            }

            virtual void handle_SET_MOTOR_NORMAL_Request(float  m1, float  m2, float  m3, float  m4) override
            {
                _mixer->motorsDisarmed[0] = m1;
                _mixer->motorsDisarmed[1] = m2;
                _mixer->motorsDisarmed[2] = m3;
                _mixer->motorsDisarmed[3] = m4;
            }
            
            virtual void handle_CLEAR_EEPROM_Request(uint8_t & code) override
            {
                for (int i = PARAMETER_SLOTS ; i < EEPROM.length() ; i++) 
                {
                    EEPROM.write(i, 0);
                }
            }
            
            virtual void handle_WP_MISSION_FLAG_Request(uint8_t & flag) override
            {
                _board->flashLed(true);
                delay(1000);
                _board->flashLed(false);
            }
            
            virtual void handle_WP_MISSION_BEGIN_Request(uint8_t & flag) override
            {
                _state.executingMission = true;
            }
            
            virtual void handle_FIRMWARE_VERSION_Request(uint8_t & version) override
            {
                version = _firmwareVersion; 
            }

            virtual void handle_SET_RC_NORMAL_Request(float  c1, float  c2, float  c3, float  c4, float  c5, float  c6) override
            {
                // Match SBUS channel map
                // received on -> goes to
                // 0->2,1->0,2->1,3->3,4->5,5->4
                // And 0->T, 1->R, 2->P, 3->Y, 4->AUX1, 5->AUX2
                float _channels[6] = {c2, c3, c1, c4, c6, c5};
                memset(_receiver->rawvals, 0, _receiver->MAXCHAN*sizeof(float));
                memcpy(_receiver->rawvals, _channels, _receiver->MAXCHAN*sizeof(float));
                _receiver->_gotNewFrame = true;
                _receiver->_bypassReceiver = true;
                _receiver->_lostSignal = false;
            }
            
            virtual void handle_LOST_SIGNAL_Request(uint8_t  flag) override
            {
                _receiver->_lostSignal = flag;
            }

            virtual void handle_SET_MOSQUITO_VERSION_Request(uint8_t version) override
            {
                uint8_t config = EEPROM.read(GENERAL_CONFIG);
                if (version)
                {
                  EEPROM.put(GENERAL_CONFIG, config | (1 << MOSQUITO_VERSION));
                } else {
                  EEPROM.put(GENERAL_CONFIG, config & ~(1 << MOSQUITO_VERSION));
                }
            }
            
            virtual void handle_SET_POSITIONING_BOARD_Request(uint8_t hasBoard) override
            {
                uint8_t config = EEPROM.read(GENERAL_CONFIG);
                if (hasBoard)
                {
                  EEPROM.put(GENERAL_CONFIG, config | (1 << POSITIONING_BOARD));
                } else {
                  EEPROM.put(GENERAL_CONFIG, config & ~(1 << POSITIONING_BOARD));
                }
            }
            
            virtual void handle_SET_PID_CONSTANTS_Request(float gyroRollPitchP,
                float gyroRollPitchI, float gyroRollPitchD, float gyroYawP,
                float gyroYawI, float demandsToRate, float levelP,
                float altHoldP, float altHoldVelP, float altHoldVelI, float altHoldVelD,
                float minAltitude, float param6, float param7,
                float param8, float param9) override
            {
                EEPROM.put(PID_CONSTANTS, gyroRollPitchP);
                EEPROM.put(PID_CONSTANTS + 1 * sizeof(float), gyroRollPitchI);
                EEPROM.put(PID_CONSTANTS + 2 * sizeof(float), gyroRollPitchD);
                EEPROM.put(PID_CONSTANTS + 3 * sizeof(float), gyroYawP);
                EEPROM.put(PID_CONSTANTS + 4 * sizeof(float), gyroYawI);
                EEPROM.put(PID_CONSTANTS + 5 * sizeof(float), demandsToRate);
                EEPROM.put(PID_CONSTANTS + 6 * sizeof(float), levelP);
                EEPROM.put(PID_CONSTANTS + 7 * sizeof(float), altHoldP);
                EEPROM.put(PID_CONSTANTS + 8 * sizeof(float), altHoldVelP);
                EEPROM.put(PID_CONSTANTS + 9 * sizeof(float), altHoldVelI);
                EEPROM.put(PID_CONSTANTS + 10 * sizeof(float), altHoldVelD);
                EEPROM.put(PID_CONSTANTS + 11 * sizeof(float), minAltitude);
                
            }

        public:

            void init(Board * board, Receiver * receiver, Mixer * mixer, Rate * ratePid, bool armed=false)
            {  
                // Store the essentials
                _board    = board;
                _receiver = receiver;
                _mixer    = mixer;
                _ratePid  = ratePid;
                

                // Support for mandatory sensors
                addSensor(&_quaternion, board);
                addSensor(&_gyrometer, board);

                // Support adding new sensors and PID controllers
                _sensor_count = 0;

                // Last PID controller is always ratePid (rate), aux state = 0
                addPidController(ratePid, 0);

                // Initialize state
                memset(&_state, 0, sizeof(state_t));

                // Support safety override by simulator
                _state.armed = armed;
                // Will be set to true when start mission message is received
                _state.executingMission = false;

                // Initialize MPS parser for serial comms
                MspParser::init();

                // Initialize the receiver
                _receiver->begin();
                
                // Initialize the planner
                planner.init(PARAMETER_SLOTS);
                // XXX Only for debuging purposes.
                //planner.printMission();


                // Tell the mixer which board to use
                _mixer->board = board; 

                // Setup failsafe
                _failsafe = false;

            } // init

            void addSensor(PeripheralSensor * sensor) 
            {
                add_sensor(sensor);
            }

            void addSensor(SurfaceMountSensor * sensor, Board * board) 
            {
                add_sensor(sensor);

                sensor->board = board;
            }

            void addPidController(PID_Controller * pidController, uint8_t auxState) 
            {
                pidController->auxState = auxState;

                _pid_controllers[_pid_controller_count++] = pidController;
            }

            void update(void)
            {
                // Check planner
                checkPlanner();
                // Grab control signal if available
                checkReceiver();

                // Check mandatory sensors
                checkGyrometer();
                checkQuaternion();

                // Check optional sensors
                checkOptionalSensors();
                
                // XXX Only for debuging purposes
                // readEEPROM();
            } 

    }; // class Hackflight

} // namespace
