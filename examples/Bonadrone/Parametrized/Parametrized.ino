#include <Arduino.h>
#include "main.hpp"

hf::HackflightWrapper hw;

void setup() {
  // put your setup code here, to run once:
  hw.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  hw.h.update();
}
