/*
   esp8266.hpp : ESP8266 support for Arduino flight controllers

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

#include<ESP8266WiFi.h>
#include "mspparser2.hpp"

namespace hf {

    class ESP8266_Receiver : public Receiver {

        private:

            static constexpr char * SSID     = "Esp8266TestNet";
            static constexpr char * PASSWORD = "Esp8266Test"; // has to be longer than 7 chars

            WiFiServer _server = WiFiServer(80);
            WiFiClient _client;
            bool _haveClient;

            MspParser2 parser;

        protected:

            void begin(void)
            {
                WiFi.mode(WIFI_AP);
                //WiFi.softAP(SSID, PASSWORD, 1, 1); // password required
                WiFi.softAP(SSID); // no password
                _server.begin();
                _haveClient = false;

                parser.init();
            }

            bool gotNewFrame(void)
            {
                if (_haveClient) {

                    if (_client.connected()) {

                        while (_client.available()) {
                            if (parser.parse(_client.read()) == MspParser2::RESULT_DONE) {
                                float c1=0, c2=0, c3=0, c4=0, c5=0, c6=0;

                                parser.get_SET_RC_NORMAL(c1, c2, c3, c4, c5, c6);
                                Debug::printf("%+2.2f %+2.2f %+2.2f %+2.2f %+2.2f %+2.2f\n", 
                                        c1, c2, c3, c4, c5, c6);
                            }
                        }
                    }

                    else {
                        _haveClient = false;
                    }
                }

                else {
                    _client = _server.available();
                    if (_client) {
                        Serial.println("Connected!");
                        _haveClient = true;
                    } 
                    else {
                        Serial.println("Waiting for client ...");
                    }
                }

                return false; //rx.gotNewFrame();
            }

            void readRawvals(void)
            {
                //rx.getChannelValuesNormalized(rawvals, CHANNELS);
            }

            bool lostSignal(void)
            {
                return false; //rx.timedOut(micros());
            }

        public:

        ESP8266_Receiver(const uint8_t channelMap[6]) : Receiver(channelMap) 
        { 
        }

    }; // class ESP8266_Receiver

} // namespace hf
