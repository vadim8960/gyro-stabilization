#ifndef _STLISY300AL_H_
#define _STLISY300AL_H_

#include <Arduino.h>

#define GYRO_SENSITIVITY   0.0033
#define ANALOG_SENSITIVITY 0.0047

class LISY300AL {
	float position;
	int pin;
	int zero_position;
	float make_cardinal(float _position);
public:
	LISY300AL(int pin);
	void setupLISY300AL();
	float getPosition();
  void clear();
};

#endif /*_STLISY300AL_H_*/
