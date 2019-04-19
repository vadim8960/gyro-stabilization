#include "AX12A.h"

void setup() {
  Serial.begin(9600);
  ax12a.begin(1000000, 2, &Serial1);
  ax12a.setEndless(1, true);
}

void loop() {
//  ax12a.moveSpeed(1, 512, 200);
  ax12a.turn(1, true, 100);
  Serial.println("loop");
}
