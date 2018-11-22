/*
   mspparser.hpp: header-only implementation of MSP parsing routines

   Auto-generated code: DO NOT EDIT!

   Copyright (C) Simon D. Levy 2018

   This program is part of Hackflight

   This code is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as 
   published by the Free Software Foundation, either version 3 of the 
   License, or (at your option) any later version.

   This code is distributed in the hope that it will be useful,     
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License 
   along with this code.  If not, see <http:#www.gnu.org/licenses/>.
 */


#pragma once

#include <stdint.h>
#include <string.h>

#include <EEPROM.h>

namespace hf {

    class MspParser {

        public:

            static const uint8_t MAXMSG = 255;

        private:

            static const int INBUF_SIZE  = 128;
            static const int OUTBUF_SIZE = 128;
            
            int EEPROMindex = 0;
            bool incomingMission = 0;

            typedef enum serialState_t {
                IDLE,
                HEADER_START,
                HEADER_M,
                HEADER_ARROW,
                HEADER_SIZE,
                HEADER_CMD
            } serialState_t;

            uint8_t _checksum;
            uint8_t _inBuf[INBUF_SIZE];
            uint8_t _inBufIndex;
            uint8_t _outBuf[OUTBUF_SIZE];
            uint8_t _outBufIndex;
            uint8_t _outBufSize;
            uint8_t _command;
            uint8_t _offset;
            uint8_t _dataSize;
            uint8_t _direction;

            serialState_t  _state;

            void serialize8(uint8_t a)
            {
                _outBuf[_outBufSize++] = a;
                _checksum ^= a;
            }

            void serialize16(int16_t a)
            {
                serialize8(a & 0xFF);
                serialize8((a >> 8) & 0xFF);
            }

            void serialize32(uint32_t a)
            {
                serialize8(a & 0xFF);
                serialize8((a >> 8) & 0xFF);
                serialize8((a >> 16) & 0xFF);
                serialize8((a >> 24) & 0xFF);
            }

            void headSerialResponse(uint8_t err, uint8_t s)
            {
                serialize8('$');
                serialize8('M');
                serialize8(err ? '!' : '>');
                _checksum = 0;               // start calculating a new _checksum
                serialize8(s);
                serialize8(_command);
            }

            void headSerialReply(uint8_t s)
            {
                headSerialResponse(0, s);
            }

            void prepareToSend(uint8_t count, uint8_t size)
            {
                _outBufSize = 0;
                _outBufIndex = 0;
                headSerialReply(count*size);
            }

            void prepareToSendBytes(uint8_t count)
            {
                prepareToSend(count, 1);
            }

            void sendByte(uint8_t src)
            {
                serialize8(src);
            }

            void prepareToSendShorts(uint8_t count)
            {
                prepareToSend(count, 2);
            }

            void sendShort(short src)
            {
                int16_t a;
                memcpy(&a, &src, 2);
                serialize16(a);
            }

            void prepareToSendInts(uint8_t count)
            {
                prepareToSend(count, 4);
            }

            void sendInt(int32_t src)
            {
                int32_t a;
                memcpy(&a, &src, 4);
                serialize32(a);
            }

            void prepareToSendFloats(uint8_t count)
            {
                prepareToSend(count, 4);
            }

            void sendFloat(float src)
            {
                uint32_t a;
                memcpy(&a, &src, 4);
                serialize32(a);
            }

            static uint8_t CRC8(uint8_t * data, int n) 
            {
                uint8_t crc = 0x00;

                for (int k=0; k<n; ++k) {

                    crc ^= data[k];
                }

                return crc;
            }

            float getArgument(uint8_t k)
            {
                return (float)k; // XXX for testing only
            }

            void processMissionCommand(uint8_t command)
            {
                if (incomingMission && command != 23)
                {
                    EEPROM.write(EEPROMindex, command);
                    EEPROMindex += 1;
                    if (command != 1 && command != 2 && command != 3)
                    {
                        uint8_t commandData = readCommandData();
                        EEPROM.write(EEPROMindex, commandData);
                        EEPROMindex += 1;
                    }
                }
            }
                        
            uint8_t readCommandData(void)
            {
                return _inBuf[_inBufIndex++] & 0xff;
            }
                        
        protected:

            void init(void)
            {
                _checksum = 0;
                _outBufIndex = 0;
                _outBufSize = 0;
                _command = 0;
                _offset = 0;
                _dataSize = 0;
                _state = IDLE;
            }
            
            uint8_t availableBytes(void)
            {
                return _outBufSize;
            }

            uint8_t readByte(void)
            {
                _outBufSize--;
                return _outBuf[_outBufIndex++];
            }

            // returns true if reboot request, false otherwise
            bool parse(uint8_t c)
            {
                switch (_state) {

                    case IDLE:
                        if (c == 'R') {
                            return true; // got reboot command
                        }
                        _state = (c == '$') ? HEADER_START : IDLE;
                        break;

                    case HEADER_START:
                        _state = (c == 'M') ? HEADER_M : IDLE;
                        break;

                    case HEADER_M:
                        switch (c) {
                           case '>':
                                _direction = 1;
                                _state = HEADER_ARROW;
                                break;
                            case '<':
                                _direction = 0;
                                _state = HEADER_ARROW;
                                break;
                             default:
                                _state = IDLE;
                        }
                        break;

                    case HEADER_ARROW:
                        if (c > INBUF_SIZE) {       // now we are expecting the payload size
                            _state = IDLE;
                            return false;
                        }
                        _dataSize = c;
                        _offset = 0;
                        _checksum = 0;
                        _inBufIndex = 0;
                        _checksum ^= c;
                        _state = HEADER_SIZE;      // the command is to follow
                        break;

                    case HEADER_SIZE:
                        _command = c;
                        _checksum ^= c;
                        _state = HEADER_CMD;
                        break;

                    case HEADER_CMD:
                        if (_offset < _dataSize) {
                            _checksum ^= c;
                            _inBuf[_offset++] = c;
                        } else  {
                            if (_checksum == c) {        // compare calculated and transferred _checksum
                                if (_direction == 0) {
                                    dispatchRequestMessage();
                                }
                                else {
                                    dispatchDataMessage();
                                }
                            }
                            _state = IDLE;
                        }

                } // switch (_state)

                return false; // no reboot 

            } // parse


            void dispatchRequestMessage(void)
            {
                processMissionCommand(_command);
                switch (_command) {

                    case 102:
                    {
                        int16_t accx = 0;
                        int16_t accy = 0;
                        int16_t accz = 0;
                        int16_t gyrx = 0;
                        int16_t gyry = 0;
                        int16_t gyrz = 0;
                        int16_t magx = 0;
                        int16_t magy = 0;
                        int16_t magz = 0;
                        handle_RAW_IMU_Request(accx, accy, accz, gyrx, gyry, gyrz, magx, magy, magz);
                        prepareToSendShorts(9);
                        sendShort(accx);
                        sendShort(accy);
                        sendShort(accz);
                        sendShort(gyrx);
                        sendShort(gyry);
                        sendShort(gyrz);
                        sendShort(magx);
                        sendShort(magy);
                        sendShort(magz);
                        serialize8(_checksum);
                        } break;

                    case 121:
                    {
                        float c1 = 0;
                        float c2 = 0;
                        float c3 = 0;
                        float c4 = 0;
                        float c5 = 0;
                        float c6 = 0;
                        handle_RC_NORMAL_Request(c1, c2, c3, c4, c5, c6);
                        prepareToSendFloats(6);
                        sendFloat(c1);
                        sendFloat(c2);
                        sendFloat(c3);
                        sendFloat(c4);
                        sendFloat(c5);
                        sendFloat(c6);
                        serialize8(_checksum);
                        } break;

                    case 122:
                    {
                        float roll = 0;
                        float pitch = 0;
                        float yaw = 0;
                        handle_ATTITUDE_RADIANS_Request(roll, pitch, yaw);
                        prepareToSendFloats(3);
                        sendFloat(roll);
                        sendFloat(pitch);
                        sendFloat(yaw);
                        serialize8(_checksum);
                        } break;

                    case 123:
                    {
                        float estalt = 0;
                        float vario = 0;
                        handle_ALTITUDE_METERS_Request(estalt, vario);
                        prepareToSendFloats(2);
                        sendFloat(estalt);
                        sendFloat(vario);
                        serialize8(_checksum);
                        } break;

                    case 126:
                    {
                        float agl = 0;
                        float flowx = 0;
                        float flowy = 0;
                        handle_LOITER_Request(agl, flowx, flowy);
                        prepareToSendFloats(3);
                        sendFloat(agl);
                        sendFloat(flowx);
                        sendFloat(flowy);
                        serialize8(_checksum);
                        } break;

                    case 199:
                    {
                        int32_t value1 = 0;
                        int32_t value2 = 0;
                        handle_FAKE_INT_Request(value1, value2);
                        prepareToSendInts(2);
                        sendInt(value1);
                        sendInt(value2);
                        serialize8(_checksum);
                        } break;

                    case 215:
                    {
                        float m1 = 0;
                        memcpy(&m1,  &_inBuf[0], sizeof(float));

                        float m2 = 0;
                        memcpy(&m2,  &_inBuf[4], sizeof(float));

                        float m3 = 0;
                        memcpy(&m3,  &_inBuf[8], sizeof(float));

                        float m4 = 0;
                        memcpy(&m4,  &_inBuf[12], sizeof(float));

                        handle_SET_MOTOR_NORMAL_Request(m1, m2, m3, m4);
                        } break;

                    case 124:
                    {
                        float m1 = 0;
                        float m2 = 0;
                        float m3 = 0;
                        float m4 = 0;
                        handle_GET_MOTOR_NORMAL_Request(m1, m2, m3, m4);
                        prepareToSendFloats(4);
                        sendFloat(m1);
                        sendFloat(m2);
                        sendFloat(m3);
                        sendFloat(m4);
                        serialize8(_checksum);
                        } break;

                    case 0:
                    {
                        uint8_t code = 0;
                        handle_CLEAR_EEPROM_Request(code);
                        prepareToSendFloats(1);
                        sendFloat(code);
                        serialize8(_checksum);
                        } break;

                    case 1:
                    {
                        uint8_t code = 0;
                        handle_WP_ARM_Request(code);
                        prepareToSendFloats(1);
                        sendFloat(code);
                        serialize8(_checksum);
                        } break;

                    case 2:
                    {
                        uint8_t code = 0;
                        handle_WP_DISARM_Request(code);
                        prepareToSendFloats(1);
                        sendFloat(code);
                        serialize8(_checksum);
                        } break;

                    case 3:
                    {
                        uint8_t code = 0;
                        handle_WP_LAND_Request(code);
                        prepareToSendFloats(1);
                        sendFloat(code);
                        serialize8(_checksum);
                        } break;

                    case 4:
                    {
                        uint8_t meters = 0;
                        uint8_t code = 0;
                        handle_WP_TAKE_OFF_Request(meters, code);
                        prepareToSendFloats(2);
                        sendFloat(meters);
                        sendFloat(code);
                        serialize8(_checksum);
                        } break;

                    case 5:
                    {
                        uint8_t meters = 0;
                        uint8_t code = 0;
                        handle_WP_GO_FORWARD_Request(meters, code);
                        prepareToSendFloats(2);
                        sendFloat(meters);
                        sendFloat(code);
                        serialize8(_checksum);
                        } break;

                    case 6:
                    {
                        uint8_t meters = 0;
                        uint8_t code = 0;
                        handle_WP_GO_BACKWARD_Request(meters, code);
                        prepareToSendFloats(2);
                        sendFloat(meters);
                        sendFloat(code);
                        serialize8(_checksum);
                        } break;

                    case 7:
                    {
                        uint8_t meters = 0;
                        uint8_t code = 0;
                        handle_WP_GO_LEFT_Request(meters, code);
                        prepareToSendFloats(2);
                        sendFloat(meters);
                        sendFloat(code);
                        serialize8(_checksum);
                        } break;

                    case 8:
                    {
                        uint8_t meters = 0;
                        uint8_t code = 0;
                        handle_WP_GO_RIGHT_Request(meters, code);
                        prepareToSendFloats(2);
                        sendFloat(meters);
                        sendFloat(code);
                        serialize8(_checksum);
                        } break;

                    case 9:
                    {
                        uint8_t meters = 0;
                        uint8_t code = 0;
                        handle_WP_CHANGE_ALTITUDE_Request(meters, code);
                        prepareToSendFloats(2);
                        sendFloat(meters);
                        sendFloat(code);
                        serialize8(_checksum);
                        } break;

                    case 10:
                    {
                        uint8_t speed = 0;
                        uint8_t code = 0;
                        handle_WP_CHANGE_SPEED_Request(speed, code);
                        prepareToSendFloats(2);
                        sendFloat(speed);
                        sendFloat(code);
                        serialize8(_checksum);
                        } break;

                    case 11:
                    {
                        uint8_t seconds = 0;
                        uint8_t code = 0;
                        handle_WP_HOVER_Request(seconds, code);
                        prepareToSendFloats(2);
                        sendFloat(seconds);
                        sendFloat(code);
                        serialize8(_checksum);
                        } break;

                }
            }

            void dispatchDataMessage(void)
            {
                switch (_command) {

                    case 102:
                    {
                        int16_t accx = getArgument(0);
                        int16_t accy = getArgument(1);
                        int16_t accz = getArgument(2);
                        int16_t gyrx = getArgument(3);
                        int16_t gyry = getArgument(4);
                        int16_t gyrz = getArgument(5);
                        int16_t magx = getArgument(6);
                        int16_t magy = getArgument(7);
                        int16_t magz = getArgument(8);
                        handle_RAW_IMU_Data(accx, accy, accz, gyrx, gyry, gyrz, magx, magy, magz);
                        } break;

                    case 23:
                    {
                        incomingMission = incomingMission ? false : true;
                        uint8_t flag = 0;
                        handle_WP_MISSION_FLAG_Request(flag);
                        prepareToSendFloats(1);
                        sendFloat(flag);
                        serialize8(_checksum);
                        } break;

                    case 30:
                    {
                        uint8_t flag = 0;
                        handle_WP_MISSION_BEGIN_Request(flag);
                        prepareToSendFloats(1);
                        sendFloat(flag);
                        serialize8(_checksum);
                        } break;

                    case 50:
                    {
                        uint8_t version = 0;
                        handle_FIRMWARE_VERSION_Request(version);
                        _outBufSize = 0;
                        _outBufIndex = 0;
                        headSerialReply(1);
                        serialize8(version);
                        serialize8(_checksum);
                        } break;

                }
            }

            void dispatchDataMessage(void)
            {
                switch (_command) {

                    case 121:
                    {
                        float c1 = getArgument(0);
                        float c2 = getArgument(1);
                        float c3 = getArgument(2);
                        float c4 = getArgument(3);
                        float c5 = getArgument(4);
                        float c6 = getArgument(5);
                        handle_RC_NORMAL_Data(c1, c2, c3, c4, c5, c6);
                        } break;

                    case 122:
                    {
                        float roll = getArgument(0);
                        float pitch = getArgument(1);
                        float yaw = getArgument(2);
                        handle_ATTITUDE_RADIANS_Data(roll, pitch, yaw);
                        } break;

                    case 123:
                    {
                        float estalt = getArgument(0);
                        float vario = getArgument(1);
                        handle_ALTITUDE_METERS_Data(estalt, vario);
                        } break;

                    case 199:
                    {
                        int32_t value1 = getArgument(0);
                        int32_t value2 = getArgument(1);
                        handle_FAKE_INT_Data(value1, value2);
                        } break;

                    case 50:
                    {
                        uint8_t version = getArgument(0);
                        handle_FIRMWARE_VERSION_Data(version);
                        } break;

                }
            }

        protected:

            virtual void handle_RAW_IMU_Request(int16_t & accx, int16_t & accy, int16_t & accz, int16_t & gyrx, int16_t & gyry, int16_t & gyrz, int16_t & magx, int16_t & magy, int16_t & magz)
            {
                (void)accx;
                (void)accy;
                (void)accz;
                (void)gyrx;
                (void)gyry;
                (void)gyrz;
                (void)magx;
                (void)magy;
                (void)magz;
            }

            virtual void handle_RAW_IMU_Data(int16_t & accx, int16_t & accy, int16_t & accz, int16_t & gyrx, int16_t & gyry, int16_t & gyrz, int16_t & magx, int16_t & magy, int16_t & magz)
            {
                (void)accx;
                (void)accy;
                (void)accz;
                (void)gyrx;
                (void)gyry;
                (void)gyrz;
                (void)magx;
                (void)magy;
                (void)magz;
            }

            virtual void handle_RC_NORMAL_Request(float & c1, float & c2, float & c3, float & c4, float & c5, float & c6)
            {
                (void)c1;
                (void)c2;
                (void)c3;
                (void)c4;
                (void)c5;
                (void)c6;
            }

            virtual void handle_RC_NORMAL_Data(float & c1, float & c2, float & c3, float & c4, float & c5, float & c6)
            {
                (void)c1;
                (void)c2;
                (void)c3;
                (void)c4;
                (void)c5;
                (void)c6;
            }

            virtual void handle_ATTITUDE_RADIANS_Request(float & roll, float & pitch, float & yaw)
            {
                (void)roll;
                (void)pitch;
                (void)yaw;
            }

            virtual void handle_ATTITUDE_RADIANS_Data(float & roll, float & pitch, float & yaw)
            {
                (void)roll;
                (void)pitch;
                (void)yaw;
            }

            virtual void handle_ALTITUDE_METERS_Request(float & estalt, float & vario)
            {
                (void)estalt;
                (void)vario;
            }

            virtual void handle_ALTITUDE_METERS_Data(float & estalt, float & vario)
            {
                (void)estalt;
                (void)vario;
            }

            virtual void handle_LOITER_Request(float & agl, float & flowx, float & flowy)
            {
                (void)agl;
                (void)flowx;
                (void)flowy;
            }

            virtual void handle_LOITER_Data(float & agl, float & flowx, float & flowy)
            {
                (void)agl;
                (void)flowx;
                (void)flowy;
            }

            virtual void handle_SET_ARMED_Request(uint8_t  flag)
            {
                (void)flag;
            }

            virtual void handle_FAKE_INT_Request(int32_t & value1, int32_t & value2)
            {
                (void)value1;
                (void)value2;
            }

            virtual void handle_FAKE_INT_Data(int32_t & value1, int32_t & value2)
            {
                (void)value1;
                (void)value2;
            }

            virtual void handle_SET_MOTOR_NORMAL_Data(float  m1, float  m2, float  m3, float  m4)
            {
                (void)m1;
                (void)m2;
                (void)m3;
                (void)m4;
            }

            virtual void handle_GET_MOTOR_NORMAL_Request(float & m1, float & m2, float & m3, float & m4)
            {
                (void)m1;
                (void)m2;
                (void)m3;
                (void)m4;
            }

            virtual void handle_GET_MOTOR_NORMAL_Data(float & m1, float & m2, float & m3, float & m4)
            {
                (void)m1;
                (void)m2;
                (void)m3;
                (void)m4;
            }

            virtual void handle_CLEAR_EEPROM_Request(uint8_t & code)
            {
                (void)code;
            }

            virtual void handle_CLEAR_EEPROM_Data(uint8_t & code)
            {
                (void)code;
            }

            virtual void handle_WP_ARM_Request(uint8_t & code)
            {
                (void)code;
            }

            virtual void handle_WP_ARM_Data(uint8_t & code)
            {
                (void)code;
            }

            virtual void handle_WP_DISARM_Request(uint8_t & code)
            {
                (void)code;
            }

            virtual void handle_WP_DISARM_Data(uint8_t & code)
            {
                (void)code;
            }

            virtual void handle_WP_LAND_Request(uint8_t & code)
            {
                (void)code;
            }

            virtual void handle_WP_LAND_Data(uint8_t & code)
            {
                (void)code;
            }

            virtual void handle_WP_TAKE_OFF_Request(uint8_t & meters, uint8_t & code)
            {
                (void)meters;
                (void)code;
            }

            virtual void handle_WP_TAKE_OFF_Data(uint8_t & meters, uint8_t & code)
            {
                (void)meters;
                (void)code;
            }

            virtual void handle_WP_GO_FORWARD_Request(uint8_t & meters, uint8_t & code)
            {
                (void)meters;
                (void)code;
            }

            virtual void handle_WP_GO_FORWARD_Data(uint8_t & meters, uint8_t & code)
            {
                (void)meters;
                (void)code;
            }

            virtual void handle_WP_GO_BACKWARD_Request(uint8_t & meters, uint8_t & code)
            {
                (void)meters;
                (void)code;
            }

            virtual void handle_WP_GO_BACKWARD_Data(uint8_t & meters, uint8_t & code)
            {
                (void)meters;
                (void)code;
            }

            virtual void handle_WP_GO_LEFT_Request(uint8_t & meters, uint8_t & code)
            {
                (void)meters;
                (void)code;
            }

            virtual void handle_WP_GO_LEFT_Data(uint8_t & meters, uint8_t & code)
            {
                (void)meters;
                (void)code;
            }

            virtual void handle_WP_GO_RIGHT_Request(uint8_t & meters, uint8_t & code)
            {
                (void)meters;
                (void)code;
            }

            virtual void handle_WP_GO_RIGHT_Data(uint8_t & meters, uint8_t & code)
            {
                (void)meters;
                (void)code;
            }

            virtual void handle_WP_CHANGE_ALTITUDE_Request(uint8_t & meters, uint8_t & code)
            {
                (void)meters;
                (void)code;
            }

            virtual void handle_WP_CHANGE_ALTITUDE_Data(uint8_t & meters, uint8_t & code)
            {
                (void)meters;
                (void)code;
            }

            virtual void handle_WP_CHANGE_SPEED_Request(uint8_t & speed, uint8_t & code)
            {
                (void)speed;
                (void)code;
            }

            virtual void handle_WP_CHANGE_SPEED_Data(uint8_t & speed, uint8_t & code)
            {
                (void)speed;
                (void)code;
            }

            virtual void handle_WP_HOVER_Request(uint8_t & seconds, uint8_t & code)
            {
                (void)seconds;
                (void)code;
            }

            virtual void handle_WP_HOVER_Data(uint8_t & seconds, uint8_t & code)
            {
                (void)seconds;
                (void)code;
            }

            virtual void handle_WP_TURN_CW_Request(uint8_t & degrees, uint8_t & code)
            {
                (void)degrees;
                (void)code;
            }

            virtual void handle_WP_TURN_CW_Data(uint8_t & degrees, uint8_t & code)
            {
                (void)degrees;
                (void)code;
            }

            virtual void handle_WP_TURN_CCW_Request(uint8_t & degrees, uint8_t & code)
            {
                (void)degrees;
                (void)code;
            }

            virtual void handle_WP_TURN_CCW_Data(uint8_t & degrees, uint8_t & code)
            {
                (void)degrees;
                (void)code;
            }

            virtual void handle_WP_MISSION_FLAG_Request(uint8_t & flag)
            {
                (void)flag;
            }

            virtual void handle_WP_MISSION_FLAG_Data(uint8_t & flag)
            {
                (void)flag;
            }

            virtual void handle_WP_MISSION_BEGIN_Request(uint8_t & flag)
            {
                (void)flag;
            }

            virtual void handle_WP_MISSION_BEGIN_Data(uint8_t & flag)
            {
                (void)flag;
            }

            virtual void handle_FIRMWARE_VERSION_Request(uint8_t & version)
            {
                (void)version;
            }

            virtual void handle_FIRMWARE_VERSION_Data(uint8_t & version)
            {
                (void)version;
            }

        public:

            static uint8_t serialize_RAW_IMU_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 102;
                bytes[5] = 102;

                return 6;
            }

            static uint8_t serialize_RAW_IMU(uint8_t bytes[], int16_t  accx, int16_t  accy, int16_t  accz, int16_t  gyrx, int16_t  gyry, int16_t  gyrz, int16_t  magx, int16_t  magy, int16_t  magz)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 18;
                bytes[4] = 102;

                memcpy(&bytes[5], &accx, sizeof(int16_t));
                memcpy(&bytes[7], &accy, sizeof(int16_t));
                memcpy(&bytes[9], &accz, sizeof(int16_t));
                memcpy(&bytes[11], &gyrx, sizeof(int16_t));
                memcpy(&bytes[13], &gyry, sizeof(int16_t));
                memcpy(&bytes[15], &gyrz, sizeof(int16_t));
                memcpy(&bytes[17], &magx, sizeof(int16_t));
                memcpy(&bytes[19], &magy, sizeof(int16_t));
                memcpy(&bytes[21], &magz, sizeof(int16_t));

                bytes[23] = CRC8(&bytes[3], 20);

                return 24;
            }

            static uint8_t serialize_RC_NORMAL_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 121;
                bytes[5] = 121;

                return 6;
            }

            static uint8_t serialize_RC_NORMAL(uint8_t bytes[], float  c1, float  c2, float  c3, float  c4, float  c5, float  c6)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 24;
                bytes[4] = 121;

                memcpy(&bytes[5], &c1, sizeof(float));
                memcpy(&bytes[9], &c2, sizeof(float));
                memcpy(&bytes[13], &c3, sizeof(float));
                memcpy(&bytes[17], &c4, sizeof(float));
                memcpy(&bytes[21], &c5, sizeof(float));
                memcpy(&bytes[25], &c6, sizeof(float));

                bytes[29] = CRC8(&bytes[3], 26);

                return 30;
            }

            static uint8_t serialize_ATTITUDE_RADIANS_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 122;
                bytes[5] = 122;

                return 6;
            }

            static uint8_t serialize_ATTITUDE_RADIANS(uint8_t bytes[], float  roll, float  pitch, float  yaw)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 12;
                bytes[4] = 122;

                memcpy(&bytes[5], &roll, sizeof(float));
                memcpy(&bytes[9], &pitch, sizeof(float));
                memcpy(&bytes[13], &yaw, sizeof(float));

                bytes[17] = CRC8(&bytes[3], 14);

                return 18;
            }

            static uint8_t serialize_ALTITUDE_METERS_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 123;
                bytes[5] = 123;

                return 6;
            }

            static uint8_t serialize_ALTITUDE_METERS(uint8_t bytes[], float  estalt, float  vario)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 8;
                bytes[4] = 123;

                memcpy(&bytes[5], &estalt, sizeof(float));
                memcpy(&bytes[9], &vario, sizeof(float));

                bytes[13] = CRC8(&bytes[3], 10);

                return 14;
            }

            static uint8_t serialize_LOITER_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 126;
                bytes[5] = 126;

                return 6;
            }

            static uint8_t serialize_LOITER(uint8_t bytes[], float  agl, float  flowx, float  flowy)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 12;
                bytes[4] = 126;

                memcpy(&bytes[5], &agl, sizeof(float));
                memcpy(&bytes[9], &flowx, sizeof(float));
                memcpy(&bytes[13], &flowy, sizeof(float));

                bytes[17] = CRC8(&bytes[3], 14);

                return 18;
            }

            static uint8_t serialize_FAKE_INT_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 199;
                bytes[5] = 199;

                return 6;
            }

            static uint8_t serialize_FAKE_INT(uint8_t bytes[], int32_t  value1, int32_t  value2)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 8;
                bytes[4] = 199;

                memcpy(&bytes[5], &value1, sizeof(int32_t));
                memcpy(&bytes[9], &value2, sizeof(int32_t));

                bytes[13] = CRC8(&bytes[3], 10);

                return 14;
            }

            static uint8_t serialize_SET_MOTOR_NORMAL(uint8_t bytes[], float  m1, float  m2, float  m3, float  m4)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 1;
                bytes[4] = 216;

                memcpy(&bytes[5], &flag, sizeof(uint8_t));

                bytes[6] = CRC8(&bytes[3], 3);

                return 7;
            }

            static uint8_t serialize_SET_MOTOR_NORMAL(uint8_t bytes[], float  m1, float  m2, float  m3, float  m4)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 16;
                bytes[4] = 215;

                memcpy(&bytes[5], &m1, sizeof(float));
                memcpy(&bytes[9], &m2, sizeof(float));
                memcpy(&bytes[13], &m3, sizeof(float));
                memcpy(&bytes[17], &m4, sizeof(float));

                bytes[21] = CRC8(&bytes[3], 18);

                return 22;
            }

            static uint8_t serialize_GET_MOTOR_NORMAL_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 124;
                bytes[5] = 124;

                return 6;
            }

            static uint8_t serialize_GET_MOTOR_NORMAL(uint8_t bytes[], float  m1, float  m2, float  m3, float  m4)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 16;
                bytes[4] = 124;

                memcpy(&bytes[5], &m1, sizeof(float));
                memcpy(&bytes[9], &m2, sizeof(float));
                memcpy(&bytes[13], &m3, sizeof(float));
                memcpy(&bytes[17], &m4, sizeof(float));

                bytes[21] = CRC8(&bytes[3], 18);

                return 22;
            }

            static uint8_t serialize_CLEAR_EEPROM_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 0;
                bytes[5] = 0;

                return 6;
            }

            static uint8_t serialize_CLEAR_EEPROM(uint8_t bytes[], uint8_t  code)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 1;
                bytes[4] = 0;

                memcpy(&bytes[5], &code, sizeof(uint8_t));

                bytes[6] = CRC8(&bytes[3], 3);

                return 7;
            }

            static uint8_t serialize_WP_ARM_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 1;
                bytes[5] = 1;

                return 6;
            }

            static uint8_t serialize_WP_ARM(uint8_t bytes[], uint8_t  code)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 1;
                bytes[4] = 1;

                memcpy(&bytes[5], &code, sizeof(uint8_t));

                bytes[6] = CRC8(&bytes[3], 3);

                return 7;
            }

            static uint8_t serialize_WP_DISARM_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 2;
                bytes[5] = 2;

                return 6;
            }

            static uint8_t serialize_WP_DISARM(uint8_t bytes[], uint8_t  code)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 1;
                bytes[4] = 2;

                memcpy(&bytes[5], &code, sizeof(uint8_t));

                bytes[6] = CRC8(&bytes[3], 3);

                return 7;
            }

            static uint8_t serialize_WP_LAND_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 3;
                bytes[5] = 3;

                return 6;
            }

            static uint8_t serialize_WP_LAND(uint8_t bytes[], uint8_t  code)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 1;
                bytes[4] = 3;

                memcpy(&bytes[5], &code, sizeof(uint8_t));

                bytes[6] = CRC8(&bytes[3], 3);

                return 7;
            }

            static uint8_t serialize_WP_TAKE_OFF_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 4;
                bytes[5] = 4;

                return 6;
            }

            static uint8_t serialize_WP_TAKE_OFF(uint8_t bytes[], uint8_t  meters, uint8_t  code)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 2;
                bytes[4] = 4;

                memcpy(&bytes[5], &meters, sizeof(uint8_t));
                memcpy(&bytes[6], &code, sizeof(uint8_t));

                bytes[7] = CRC8(&bytes[3], 4);

                return 8;
            }

            static uint8_t serialize_WP_GO_FORWARD_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 5;
                bytes[5] = 5;

                return 6;
            }

            static uint8_t serialize_WP_GO_FORWARD(uint8_t bytes[], uint8_t  meters, uint8_t  code)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 2;
                bytes[4] = 5;

                memcpy(&bytes[5], &meters, sizeof(uint8_t));
                memcpy(&bytes[6], &code, sizeof(uint8_t));

                bytes[7] = CRC8(&bytes[3], 4);

                return 8;
            }

            static uint8_t serialize_WP_GO_BACKWARD_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 6;
                bytes[5] = 6;

                return 6;
            }

            static uint8_t serialize_WP_GO_BACKWARD(uint8_t bytes[], uint8_t  meters, uint8_t  code)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 2;
                bytes[4] = 6;

                memcpy(&bytes[5], &meters, sizeof(uint8_t));
                memcpy(&bytes[6], &code, sizeof(uint8_t));

                bytes[7] = CRC8(&bytes[3], 4);

                return 8;
            }

            static uint8_t serialize_WP_GO_LEFT_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 7;
                bytes[5] = 7;

                return 6;
            }

            static uint8_t serialize_WP_GO_LEFT(uint8_t bytes[], uint8_t  meters, uint8_t  code)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 2;
                bytes[4] = 7;

                memcpy(&bytes[5], &meters, sizeof(uint8_t));
                memcpy(&bytes[6], &code, sizeof(uint8_t));

                bytes[7] = CRC8(&bytes[3], 4);

                return 8;
            }

            static uint8_t serialize_WP_GO_RIGHT_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 8;
                bytes[5] = 8;

                return 6;
            }

            static uint8_t serialize_WP_GO_RIGHT(uint8_t bytes[], uint8_t  meters, uint8_t  code)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 2;
                bytes[4] = 8;

                memcpy(&bytes[5], &meters, sizeof(uint8_t));
                memcpy(&bytes[6], &code, sizeof(uint8_t));

                bytes[7] = CRC8(&bytes[3], 4);

                return 8;
            }

            static uint8_t serialize_WP_CHANGE_ALTITUDE_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 9;
                bytes[5] = 9;

                return 6;
            }

            static uint8_t serialize_WP_CHANGE_ALTITUDE(uint8_t bytes[], uint8_t  meters, uint8_t  code)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 2;
                bytes[4] = 9;

                memcpy(&bytes[5], &meters, sizeof(uint8_t));
                memcpy(&bytes[6], &code, sizeof(uint8_t));

                bytes[7] = CRC8(&bytes[3], 4);

                return 8;
            }

            static uint8_t serialize_WP_CHANGE_SPEED_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 10;
                bytes[5] = 10;

                return 6;
            }

            static uint8_t serialize_WP_CHANGE_SPEED(uint8_t bytes[], uint8_t  speed, uint8_t  code)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 2;
                bytes[4] = 10;

                memcpy(&bytes[5], &speed, sizeof(uint8_t));
                memcpy(&bytes[6], &code, sizeof(uint8_t));

                bytes[7] = CRC8(&bytes[3], 4);

                return 8;
            }

            static uint8_t serialize_WP_HOVER_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 11;
                bytes[5] = 11;

                return 6;
            }

            static uint8_t serialize_WP_HOVER(uint8_t bytes[], uint8_t  seconds, uint8_t  code)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 2;
                bytes[4] = 11;

                memcpy(&bytes[5], &seconds, sizeof(uint8_t));
                memcpy(&bytes[6], &code, sizeof(uint8_t));

                bytes[7] = CRC8(&bytes[3], 4);

                return 8;
            }

            static uint8_t serialize_WP_TURN_CW_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 12;
                bytes[5] = 12;

                return 6;
            }

            static uint8_t serialize_WP_TURN_CW(uint8_t bytes[], uint8_t  degrees, uint8_t  code)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 2;
                bytes[4] = 12;

                memcpy(&bytes[5], &degrees, sizeof(uint8_t));
                memcpy(&bytes[6], &code, sizeof(uint8_t));

                bytes[7] = CRC8(&bytes[3], 4);

                return 8;
            }

            static uint8_t serialize_WP_TURN_CCW_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 13;
                bytes[5] = 13;

                return 6;
            }

            static uint8_t serialize_WP_TURN_CCW(uint8_t bytes[], uint8_t  degrees, uint8_t  code)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 2;
                bytes[4] = 13;

                memcpy(&bytes[5], &degrees, sizeof(uint8_t));
                memcpy(&bytes[6], &code, sizeof(uint8_t));

                bytes[7] = CRC8(&bytes[3], 4);

                return 8;
            }

            static uint8_t serialize_WP_MISSION_FLAG_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 23;
                bytes[5] = 23;

                return 6;
            }

            static uint8_t serialize_WP_MISSION_FLAG(uint8_t bytes[], uint8_t  flag)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 1;
                bytes[4] = 23;

                memcpy(&bytes[5], &flag, sizeof(uint8_t));

                bytes[6] = CRC8(&bytes[3], 3);

                return 7;
            }

            static uint8_t serialize_WP_MISSION_BEGIN_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 30;
                bytes[5] = 30;

                return 6;
            }

            static uint8_t serialize_WP_MISSION_BEGIN(uint8_t bytes[], uint8_t  flag)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 1;
                bytes[4] = 30;

                memcpy(&bytes[5], &flag, sizeof(uint8_t));

                bytes[6] = CRC8(&bytes[3], 3);

                return 7;
            }

            static uint8_t serialize_FIRMWARE_VERSION_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 50;
                bytes[5] = 50;

                return 6;
            }

            static uint8_t serialize_FIRMWARE_VERSION(uint8_t bytes[], uint8_t  version)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 1;
                bytes[4] = 50;

                memcpy(&bytes[5], &version, sizeof(uint8_t));

                bytes[6] = CRC8(&bytes[3], 3);

                return 7;
            }

    }; // class MspParser

} // namespace hf
