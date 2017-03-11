
#include "WeatherDev.h"
#include <Wire.h>

WeatherDev::WeatherDev(uint8_t sunPin, uint8_t rainPin, uint8_t windDirectionPin, uint8_t windSpeedPin, uint8_t uvPin) {
	_sunPin = sunPin;
	_rainPin = rainPin;
	_windDirectionPin = windDirectionPin;
	_windSpeedPin = windSpeedPin;
	_uvPin = uvPin;

	sunShine = 0;
	rainCollect = 0;
	_windDirection = east;
	windSpeed = 0;
	windSignalCount = 0;
	//oldWindSignalCount = 0;
	//currentWindSignalCount = 0;
	uvLight = 0;
	sunLux = 0;

}
void WeatherDev::begin() {

	Wire.begin();
	//pinMode(_sunPin, INPUT_PULLUP);
	pinMode(_rainPin, INPUT_PULLUP);
	pinMode(_windSpeedPin, INPUT_PULLUP); //pd2 int0 or pd3 int1

}
/*
0 - 700 ; the step is 7
 */
uint8_t WeatherDev::getSunShine() {
	uint16_t tSunShine = 0;
	tSunShine = analogRead(_sunPin);
	
	// Serial.print("Sample: ");
	// if(tSunShine == 0){
	//     Serial.print("0");
	// }else {
	// 	Serial.print(tSunShine);	
	// }
	
	// Serial.print(' ');
	
	if(tSunShine > 700){
		sunShine = 100;
	    
	} else {
		sunShine = tSunShine/7;
	}
	
	return sunShine;
}

/*
north : 507 - 513     
northweast: 423 - 428 
weast: 731 - 736      
southweast: 543 - 550 
south: 691 - 698      
southeast: 457 - 462  
east: 582 - 588       
northeast: 368 - 372  

northeast 400 northweast 440 southeast 485 north 530 southweast 570 east 640 south 715 weast
 */

uint8_t WeatherDev::getWindDirection() {
	uint16_t tSampleDirection = analogRead(_windDirectionPin);
	if(tSampleDirection < 400){
	    _windDirection = northeast;
	} else if(tSampleDirection < 440) {
		_windDirection = northwest;
	} else if(tSampleDirection < 485) {
		_windDirection = southeast;
	} else if(tSampleDirection < 530) {
		_windDirection = north;
	} else if(tSampleDirection < 570) {
		_windDirection = southwest;
	} else if(tSampleDirection < 640) {
		_windDirection = east;
	} else if(tSampleDirection < 715) {
		_windDirection = south;
	} else {
		_windDirection = west;
	}

	return (uint8_t) _windDirection;

}

void WeatherDev::setWindSpeed(uint16_t signalCount) {
	
	//oldWindSignalCount  = currentWindSignalCount;
	//currentWindSignalCount = windSignalCount;

	windSpeed = HARFPERIMETER * (signalCount);

}

uint8_t WeatherDev::getUvLight() {
	uvLight = analogRead(_uvPin);
}

void WeatherDev::BH1750_Init(uint8_t address) {
	Wire.beginTransmission(address);
	Wire.write(0x10);//1lx reolution 120ms
  	Wire.endTransmission();
}

uint16_t WeatherDev::getSunLux() {
	uint8_t buff[2];
	int i=0;

	BH1750_Init(BH1750ADDR);

  	Wire.beginTransmission(BH1750ADDR);
	Wire.requestFrom(BH1750ADDR, 2);
	while(Wire.available()) {

    	buff[i] = Wire.read();  // receive one byte
    	i++;
  	}
	Wire.endTransmission();
	if(i == 2){
	    sunLux = ((buff[0]<<8)|buff[1])/1.2;
	}

	return sunLux;
}