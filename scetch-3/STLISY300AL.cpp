#include "STLISY300AL.h"

float LISY300AL::make_cardinal(float _position) {
	if(_position > 360.00)
      	_position = _position-360.00;
    else if(_position < 0)
      	_position = 360+_position;
    return _position;
}

LISY300AL::LISY300AL(int pin) {
	this->pin = pin;
	position = 0.0;
}

void LISY300AL::setupLISY300AL() {
	zero_position = analogRead(pin);
}

float LISY300AL::getPosition() {
	int change = 0;
	unsigned long last_time = millis();
	int gyro_x = analogRead(pin);
	if (abs(gyro_x - zero_position) > 3)
		change = gyro_x - zero_position;
	float time = ( float(millis()) - float(last_time) ) / 100.0;
	float degchange = change * 0.1 * ANALOG_SENSITIVITY / GYRO_SENSITIVITY;
	position = position + degchange;
	position = make_cardinal(position);
  return position;
}

void LISY300AL::clear() {
  position = 0.0;
}
