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
#include "sensors/imu.hpp"
#include "sensors/accelerometer.hpp"
#include "sensors/quaternion.hpp"

#include "filters/eskf.hpp"

#include "planner.hpp"

namespace hf {

    class Hackflight : public MspParser {

        friend class HackflightWrapper;

        private: 

            // status variables
            // XXX use a proper version formating
            uint8_t _firmwareVersion = 1;
            bool _isMosquito90;
            bool _hasPositioningBoard;
            bool _positionBoardConnected;
            uint8_t _password[4];
            bool _passwordValid = false;
            // transmitter calibration
            tx_calibration_t _tx_calibration;
            
            // Passed to Hackflight::init() for a particular build
            Board      * _board;
            Receiver   * _receiver;
            Rate       * _ratePid;
            Mixer      * _mixer;
            Planner      planner;

            ESKF eskf = ESKF();

            // PID controllers
            PID_Controller * _pid_controllers[256];
            uint8_t _pid_controller_count = 0;

            // Mandatory sensors on the board
            IMU _imu;
            Quaternion _quaternion; // not really a sensor, but we treat it like one!
            Accelerometer _accelerometer;

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
            bool _lowBattery;
            
            // XXX Store it as an MSP parameter
            const float _BAT_MIN = 3.3; // Minimum voltage for 1S Battery

            // Support for headless mode
            float _yawInitial;

            bool safeAngle(uint8_t axis)
            {
                return fabs(_state.UAVState->eulerAngles[axis]) < _ratePid->maxArmingAngle;
            }
            
            void checkBattery()
            {
              // Frequency management
              static float lastTime = 0.0;
              
              // Battery management
              static float lastTimeLowBattery = 0.0;
              static float lastBatteryVoltage = 0.0;
              static bool  isLastLowBattery = false;
              
              // Check battery with a freq. of 2Hz
              if (_board->getTime() - lastTime > 0.5)
              {
                // Look if the battery is below the limit
                if (_state.batteryVoltage < _BAT_MIN && _state.batteryVoltage > 2.5)
                {
                  // Save the first time it detects low battery
                  lastTimeLowBattery = ((isLastLowBattery) ? lastTimeLowBattery:_board->getTime());
                  
                  // Low battery if there has been low battery for more than 5s
                  // XXX it might be necessary to trigger the low battery action from here
                  _lowBattery = (_board->getTime() - lastTimeLowBattery > 5);
                  
                  // ---- Provisional
                  if (_lowBattery)
                  {
                    pinMode(25, OUTPUT);
                    digitalWrite(25, LOW);
                  }
                  // ---- Provisional
                  
                  isLastLowBattery = true;
                }
                else
                {
                  isLastLowBattery = false;
                }
                lastTime = _board->getTime();
              }
                            
              
            }
            
            void updateControlSignal(void)
            { 
                // For PID control, start with demands from receiver
                memcpy(&_demands, &_receiver->demands, sizeof(demands_t));

                runPidControllers();

                // Use updated demands to run motors
                if (_state.armed && !_failsafe && !_receiver->throttleIsDown()) {
                  _mixer->runArmed(_demands);
                
                }
            }

            void updateStateEstimate(void)
            {                        
                // Check all sensors if they need an update estimate
                for (uint8_t k=0; k<eskf.sensor_count; ++k)
                {
                    ESKF_Sensor * sensor = eskf.sensors[k];
                    float time = _board->getTime();

                    if (sensor->isEstimation && sensor->shouldUpdateESKF(time))
                    {
                        eskf.update(sensor, time);
                    }
                }
            }
            
            void correctStateEstimate(void)
            {
                // Check all sensors if they need an update estimate
                for (uint8_t k=0; k<eskf.sensor_count; ++k)
                {
                    ESKF_Sensor * sensor = eskf.sensors[k];
                    float time = _board->getTime();

                    if (sensor->isCorrection && sensor->shouldUpdateESKF(time))
                    {
                        sensor->getMeasures(*_state.UAVState);
                        eskf.correct(sensor, time);
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
                        if (pidController->modifyDemands(*_state.UAVState, _demands, currentTime) && pidController->shouldFlashLed()) {
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
                   (_receiver->lostSignal(_receiver->getBypassReceiver()) || _board->isBatteryLow())) {
                    _mixer->cutMotors();
                    _state.armed = false;
                    _failsafe = true;
                    _board->showArmedStatus(false);
                    if (_receiver->getLostSignal())
                    {
                      _receiver->setBypassReceiver(false);
                    }
                }
            } 

            void checkReceiver(void)
            {
                // Check whether receiver data is available
                if (!_receiver->getDemands(_state.UAVState->eulerAngles[AXIS_YAW] - _yawInitial)) return;

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
                    _yawInitial = _state.UAVState->eulerAngles[AXIS_YAW]; // grab yaw for headless mode
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
                _receiver->setGotNewFrame(false);
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

            // Length of stored values
            static const uint8_t PASSWORD_LENGTH   = 4;
            // Map parameters to EEPROM addresses
            static const uint8_t GENERAL_CONFIG    = 0;
            static const uint8_t PASSWORD          = 1;
            static const uint8_t PID_CONSTANTS     = PASSWORD + sizeof(uint8_t) * PASSWORD_LENGTH;
            static const uint8_t N_PID_CONSTANTS   = 16;
            static const uint8_t RANGE_PARAMS      = PID_CONSTANTS + N_PID_CONSTANTS * sizeof(float);
            static const uint8_t N_RANGE_PARAMS    = 3;
            static const uint8_t TRANSMITER_TRIMS  = RANGE_PARAMS + N_RANGE_PARAMS * sizeof(float);
            static const uint8_t N_TRIMS           = 11;
            // booleans values are stored as the bits of the byte at address 0
            static const uint8_t MOSQUITO_VERSION  = 0;
            static const uint8_t POSITIONING_BOARD = 1;
            static const uint8_t CALIBRATE_ESC     = 2;
            static const uint8_t TX_CALIBRATED     = 3;


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

            virtual void handle_ESC_CALIBRATION_Request(uint8_t & protocol)
            {
                uint8_t config = EEPROM.read(GENERAL_CONFIG);
                EEPROM.put(GENERAL_CONFIG, config | (1 << CALIBRATE_ESC));
            }

            virtual void handle_SET_LEDS_Request(uint8_t  red, uint8_t  green, uint8_t  blue) override
            {
                // XXX Do not store led pins as hardcoded values
                pinMode(25, OUTPUT);
                pinMode(26, OUTPUT);
                pinMode(38, OUTPUT);
                red ? digitalWrite(25, LOW) : digitalWrite(25, HIGH);
                green ? digitalWrite(26, LOW) : digitalWrite(26, HIGH);
                blue ? digitalWrite(38, LOW) : digitalWrite(38, HIGH);
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
                roll  = _state.UAVState->eulerAngles[0];
                pitch = _state.UAVState->eulerAngles[1];
                yaw   = _state.UAVState->eulerAngles[2];
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
                _receiver->setGotNewFrame(true);
                _receiver->setBypassReceiver(true);
                _receiver->setLostSignal(false);
            }
            
            virtual void handle_LOST_SIGNAL_Request(uint8_t  flag) override
            {
                _receiver->setLostSignal(flag);
            }

            virtual void handle_SET_MOSQUITO_VERSION_Request(uint8_t version) override
            {
                uint8_t config = EEPROM.read(GENERAL_CONFIG);
                if (version)
                {
                  EEPROM.put(GENERAL_CONFIG, config | (1 << MOSQUITO_VERSION));
                  _isMosquito90 = true;
                } else {
                  EEPROM.put(GENERAL_CONFIG, config & ~(1 << MOSQUITO_VERSION));
                  _isMosquito90 = false;
                }
            }
            
            virtual void handle_SET_POSITIONING_BOARD_Request(uint8_t hasBoard) override
            {
                uint8_t config = EEPROM.read(GENERAL_CONFIG);
                if (hasBoard)
                {
                  EEPROM.put(GENERAL_CONFIG, config | (1 << POSITIONING_BOARD));
                  _hasPositioningBoard = true;
                } else {
                  EEPROM.put(GENERAL_CONFIG, config & ~(1 << POSITIONING_BOARD));
                  _hasPositioningBoard = false;
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
                EEPROM.put(PID_CONSTANTS + 12 * sizeof(float), param6);
                EEPROM.put(PID_CONSTANTS + 13 * sizeof(float), param7);
                EEPROM.put(PID_CONSTANTS + 14 * sizeof(float), param8);
                EEPROM.put(PID_CONSTANTS + 15 * sizeof(float), param9);
            }
            
            virtual void handle_SET_RANGE_PARAMETERS_Request(float rx, float ry, float rz) override
            {
              EEPROM.put(RANGE_PARAMS, rx);
              EEPROM.put(RANGE_PARAMS + 1 * sizeof(float), ry);
              EEPROM.put(RANGE_PARAMS + 2 * sizeof(float), rz);
            }
            
            virtual void handle_SET_BATTERY_VOLTAGE_Request(float  batteryVoltage) override
            {
              _state.batteryVoltage = batteryVoltage;
            }
            
            virtual void handle_GET_BATTERY_VOLTAGE_Request(float & batteryVoltage) override
            {
                batteryVoltage = _state.batteryVoltage;
            }
            
            virtual void handle_GET_MOTOR_NORMAL_Request(float & m1, float & m2, float & m3, float & m4) override
            {
                  m1 = _mixer->motorsDisarmed[0];
                  m2 = _mixer->motorsDisarmed[1];
                  m3 = _mixer->motorsDisarmed[2];
                  m4 = _mixer->motorsDisarmed[3];
            }
            
            virtual void handle_MOSQUITO_VERSION_Request(uint8_t & mosquitoVersion) override
            {
                mosquitoVersion = _isMosquito90;
            }
            
            virtual void handle_POSITION_BOARD_Request(uint8_t & hasPositionBoard) override
            {
                hasPositionBoard = _hasPositioningBoard;
            }
            
            virtual void handle_POSITION_BOARD_CONNECTED_Request(uint8_t & positionBoardConnected) override
            {
                positionBoardConnected = _positionBoardConnected;
            }
            
            virtual void handle_CHECK_PASSWORD_Request(uint8_t  c1, uint8_t  c2, uint8_t  c3, uint8_t  c4) override
            {
                uint8_t receivedPassword[PASSWORD_LENGTH] = {c1, c2, c3, c4};
                for (int k=0; k<PASSWORD_LENGTH; k++)
                {
                  if (receivedPassword[k] != _password[k])
                  {
                    _passwordValid = false;
                    return;
                  }
                }
                _passwordValid = true;
                return;
            }
            
            virtual void handle_SET_PASSWORD_Request(uint8_t  c1, uint8_t  c2, uint8_t  c3, uint8_t  c4) override
            {
                if (_passwordValid)
                {
                    EEPROM.put(PASSWORD, c1);
                    EEPROM.put(PASSWORD + 1 * sizeof(uint8_t), c2);
                    EEPROM.put(PASSWORD + 2 * sizeof(uint8_t), c3);
                    EEPROM.put(PASSWORD + 3 * sizeof(uint8_t), c4);
                }
            }
            
            virtual void handle_PASSWORD_VALIDATED_Request(uint8_t & validPassword)
            {
                validPassword = _passwordValid;
            }
            
            virtual void handle_RC_CALIBRATION_Request(uint8_t stage) override
            {
                switch (stage) {
                  case 0:
                    {
                      // Calibration logic of stage 1: T min and R, P, Y center values
                      _tx_calibration._endStage2 = true;
                      _tx_calibration._endStage1 = false;
                      int iters = 0;
                      while (!_tx_calibration._endStage1)
                      {
                          // Check whether receiver data is available
                          _receiver->pollForFrame();
                          bool newData = _receiver->getDemands(_state.UAVState->eulerAngles[AXIS_YAW] - _yawInitial, true);
                          if (newData)
                          {
                              // Update values
                              float lastVal = _receiver->getRawval(0);
                              // Throttle
                              _tx_calibration._min[0] = (lastVal < _tx_calibration._min[0]) ? lastVal : _tx_calibration._min[0];
                              // Roll
                              _tx_calibration._center[0] = (_receiver->getRawval(1) + iters * _tx_calibration._center[0]) / (iters + 1);
                              // Pitch
                              _tx_calibration._center[1] = (_receiver->getRawval(2) + iters * _tx_calibration._center[1]) / (iters + 1);
                              // Yaw
                              _tx_calibration._center[2] = (_receiver->getRawval(3) + iters * _tx_calibration._center[2]) / (iters + 1);
                              iters += 1;
                          }
                          // Store updated values in EEPROM
                          EEPROM.put(TRANSMITER_TRIMS, _tx_calibration._min[0]);
                          EEPROM.put(TRANSMITER_TRIMS + 3 * sizeof(float), _tx_calibration._center[0]);
                          EEPROM.put(TRANSMITER_TRIMS + 6 * sizeof(float), _tx_calibration._center[1]);
                          EEPROM.put(TRANSMITER_TRIMS + 9 * sizeof(float), _tx_calibration._center[2]);
                          
                          doSerialComms();
                      }
                    }
                    break;
                  case 1:
                    {
                      // Calibration logic of stage 2: T max and R, P, Y min and max
                      _tx_calibration._endStage1 = true;
                      _tx_calibration._endStage2 = false;
                      while (!_tx_calibration._endStage2)
                      {
                          // Check whether receiver data is available
                          _receiver->pollForFrame();
                          bool newData = _receiver->getDemands(_state.UAVState->eulerAngles[AXIS_YAW] - _yawInitial, true);
                          if (newData)
                          {
                              // Update max of T, R, P, Y and min of R, P, Y
                              for (int k=0; k<4; k++)
                              {
                                  float lastVal = _receiver->getRawval(k);
                                  _tx_calibration._max[k] = (lastVal > _tx_calibration._max[k]) ? lastVal : _tx_calibration._max[k];
                                  // Update min values of R, P, Y
                                  if (k!=0)
                                  {
                                      _tx_calibration._min[k] = (lastVal < _tx_calibration._min[k]) ? lastVal : _tx_calibration._min[k];
                                  }
                              }
                          }
                          // Store updated values in EEPROM
                          EEPROM.put(TRANSMITER_TRIMS + 1 * sizeof(float), _tx_calibration._max[0]);
                          EEPROM.put(TRANSMITER_TRIMS + 2 * sizeof(float), _tx_calibration._min[1]);
                          EEPROM.put(TRANSMITER_TRIMS + 4 * sizeof(float), _tx_calibration._max[1]);
                          EEPROM.put(TRANSMITER_TRIMS + 5 * sizeof(float), _tx_calibration._min[2]);
                          EEPROM.put(TRANSMITER_TRIMS + 7 * sizeof(float), _tx_calibration._max[2]);
                          EEPROM.put(TRANSMITER_TRIMS + 8 * sizeof(float), _tx_calibration._min[3]);
                          EEPROM.put(TRANSMITER_TRIMS + 10 * sizeof(float), _tx_calibration._max[3]);
                          
                          doSerialComms();
                      }

                    }
                    break;
                  case 2:
                    {
                      // Calibration logic of stage 3: End calibration
                      _tx_calibration._endStage1 = true;
                      _tx_calibration._endStage2 = true;
                      // Mark calibration as successful
                      _tx_calibration._rcCalibrationStatus = 1;
                      uint8_t config = EEPROM.read(GENERAL_CONFIG);
                      EEPROM.put(GENERAL_CONFIG, config | (1 << TX_CALIBRATED));
                    }
                    break;
                }
            }
            
            virtual void handle_RC_CALIBRATION_STATUS_Request(uint8_t & status) override
            {
                status = _tx_calibration._rcCalibrationStatus;
            }

        public:

            void init(Board * board, Receiver * receiver, Mixer * mixer, Rate * ratePid, bool armed=false)
            {  
                // Store the essentials
                _board    = board;
                _receiver = receiver;
                _mixer    = mixer;
                _ratePid  = ratePid;
                
                // XXX Now, the order of the sensors and the ESKF sensor must be the same.
                // XXX. Ideal behavior: add sensors and ensure a method (not harcoded) 
                // XXX that adds ESKF sensors depending on how they have been added in the sensors array.
                
                // Error state kalman filter
                eskf.init();
                eskf.addSensorESKF(&_imu);
                eskf.addSensorESKF(&_accelerometer);
                
                // Support for mandatory sensors
                addSensor(&_imu, board);
                addSensor(&_accelerometer, board);      

                // Last PID controller is always ratePid (rate), aux state = 0
                addPidController(_ratePid, 0);

                // Initialize state
                memset(&_state, 0, sizeof(state_t));
                _state.UAVState = &eskf.state;
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
                _lowBattery = false;
            } // init

            void setParams(bool hasPositionBoard, bool isMosquito90, bool positionBoardConnected, uint8_t password[4])
            {
                _hasPositioningBoard = hasPositionBoard;
                _isMosquito90 = isMosquito90;
                _positionBoardConnected = positionBoardConnected;
                for (int k=0; k<PASSWORD_LENGTH; k++)
                {
                    _password[k] = password[k];
                }
            }

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
                // Check Battery
                checkBattery();
                
                // Check planner
                checkPlanner();
                // Grab control signal if available
                checkReceiver();
                // Check serials for messages
                doSerialComms();

                // Estimate and correct states via the ESKF
                updateStateEstimate();
                correctStateEstimate();
                // Compute control signal
                checkFailsafe();
                updateControlSignal();
                
                // XXX Only for debuging purposes
                // readEEPROM();
            } 

    }; // class Hackflight

} // namespace
