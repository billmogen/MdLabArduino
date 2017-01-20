

#include "PM25.h"

PM25::PM25(uint8_t measurePin, uint8_t ledPowerPin){
	_measurePin = measurePin;
	_ledPowerPin = ledPowerPin;
	dustDensityMg = 0;
	dustDensityNum = 0;
	
}

void PM25::begin(void) {
	//pinMode(_measurePin, INPUT_PULLUP);
	pinMode(_ledPowerPin, OUTPUT);
	digitalWrite(_ledPowerPin, LOW);
}

/*
	valueType: true : mg/m3 format  such as: 0.5mg/m3
			   false: Number format such as: 1001
 */
void PM25::setDustDensity(boolean valueType) {
	float voMeasured = 0;
	float calcVoltage = 0;

	digitalWrite(_ledPowerPin, HIGH);
	delayMicroseconds(SAMPLINGTIME);
	voMeasured = analogRead(_measurePin);

	delayMicroseconds(DELTATIME);
	digitalWrite(_ledPowerPin, LOW);
	delayMicroseconds(SLEEPTIME);

	Serial.print("Voltage: ");
	Serial.print(voMeasured);
	Serial.println();
	if(valueType){
		if(voMeasured > 120.47){
			calcVoltage = voMeasured * (5000.0 / 1024.0);
			dustDensityMg = 0.17 * calcVoltage - 0.1;
    
		} else {
			dustDensityMg = 0;
		}
				/*
		Serial.print("RAW Signal Value (0-1024) : ");
		Serial.print(voMeasured);
		Serial.print(" - Voltage: ");
		Serial.print(calcVoltage);
		Serial.print(" - Dust Density: ");
		Serial.println(dustDensityMg);    
		*/
	} else {
		if(voMeasured>36.455) 
			dustDensityNum = (float(voMeasured/1024)-0.0356)*120000*0.035;
		else
			dustDensityNum = 0;

		// Serial.println();
		// Serial.print("RAW Signal Value (0-1023) : ");
		// Serial.print(voMeasured);
		// Serial.print(" - Dust Density NumberFormat: ");
		// Serial.println(dustDensityNum);
		
	}

	
}