
//#ifndef PM25_H
#ifndef PM25_H
#define PM25_H 

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#define SAMPLINGTIME 280
#define DELTATIME    40
#define SLEEPTIME    9680

class PM25 {

private:
	uint8_t _measurePin, _ledPowerPin;

	float dustDensityMg;
	float dustDensityNum;
public:
	PM25(uint8_t measurePin, uint8_t ledPowerPin);
	void begin(void);
	float getDustDensityMg() {return dustDensityMg; };
	float getDustDensityNum() {return dustDensityNum; };
	void setDustDensity(boolean valueType);

};
#endif