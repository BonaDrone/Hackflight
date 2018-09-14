/*
   LoiterBoard.ino : Sketch allowing Butterfly breakout board to provide
   optical flow and above-ground-level messages to a flight controller

   Additional libraries required:

      https://github.com/simondlevy/CrossPlatformDataBus
      https://github.com/simondlevy/VL53L1X
      https://github.com/bitcraze/Bitcraze_PMW3901

   Copyright (c) 2018 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modiflowy
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

#include <Wire.h>

#include <VL53L1X.h>
#include <Bitcraze_PMW3901.h>

#include "MSPPG.h"

static uint16_t FLOW_UPDATE_HZ = 20;

static const uint8_t VCC_PIN = A0;
static const uint8_t GND_PIN = A1;

static const uint8_t CS_PIN  = 10;

static VL53L1X _distanceSensor;

static Bitcraze_PMW3901 _flowSensor(CS_PIN);

static uint32_t _flowUpdateMicros = 1000000 / FLOW_UPDATE_HZ;

static void powerPin(uint8_t pin, uint8_t value)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, value);
}

static void error(const char * sensorName)
{
    while (true) {
        Serial.print(sensorName);
        Serial.println(" offline!");
    }
}

void setup(void)
{
    // Use digital pins for VL53L1 power, ground
    powerPin(GND_PIN, LOW);
    powerPin(VCC_PIN, HIGH);

    // Debugging
    Serial.begin(115200);

    delay(200);

    Wire.begin(TWI_PINS_6_7);

    // Output to flight controller
    //Serial1.begin(115200);

    if (!_distanceSensor.begin()) {
        error("VL53L1X");
    }

    if (!_flowSensor.begin()) {
        error("PMW3901");
    }
}

void loop(void)
{
    // Declare measurement variables static so they'll persist between calls to loop()
    static uint16_t distance;
    static int16_t flowx, flowy;

    // Read distance sensor when its data is available
    if (_distanceSensor.newDataReady()) {
        distance = _distanceSensor.getDistance(); //Get the result of the measurement from the sensor
    }

    // Read flow sensor periodically
    static uint32_t _time;
    uint32_t time = micros();
    if (time-_time > _flowUpdateMicros) {
        _flowSensor.readMotionCount(&flowx, &flowy);
        _time = time;
    }

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" mm; Flow: ");
    Serial.print(flowx);
    Serial.print(" ");
    Serial.println(flowy);

    /*
    static uint8_t c;
    Serial1.write(c);
    c = (c+1) % 0xFF;
    */
}
