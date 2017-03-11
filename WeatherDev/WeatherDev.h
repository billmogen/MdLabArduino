
#ifndef WEATHERDEV_H
#define WEATHERDEV_H
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#define HARFPERIMETER 14
#define BH1750ADDR    0x23
class WeatherDev {
private:
	uint8_t _sunPin, _rainPin, _windDirectionPin, _windSpeedPin, _uvPin;
	uint8_t sunShine;
	uint8_t rainCollect;
	uint16_t windSignalCount; // running value;
	//uint16_t oldWindSignalCount; //last circle record value
	//uint16_t currentWindSignalCount;  //this circle record value
	uint16_t windSpeed;
	uint16_t uvLight;
	uint16_t sunLux;

	enum windDirection {
		east,
		west,
		south,
		north,
		southeast,
		southwest,
		northeast,
		northwest
	} _windDirection;
	

	void getWindCount();
public:
	WeatherDev(uint8_t sunPin, uint8_t rainPin, uint8_t windDirectionPin, uint8_t windSpeedPin, uint8_t uvPin);
	void begin();
	void setRainCollect() { rainCollect++; };
	void clearRainCollect() { rainCollect = 0;};
	//void setWindDirection();
	void setWindSpeed(uint16_t	signalCount);
	void setWindSignalCount() { windSignalCount++; };
	//void recordWindSignalCount();
	void clearWindSignalCount() {
		windSignalCount = 0;
		//oldWindSignalCount = 0;
		//currentWindSignalCount = 0;
	 };

	uint8_t getSunShine();
	uint8_t getRainCollect() { return rainCollect; };
	uint8_t getWindDirection();
	uint16_t getWindSpeed() { return windSpeed; };
	uint16_t getWindSignalCount() { return windSignalCount; };
	uint8_t getUvLight();
	void BH1750_Init(uint8_t address);
	uint16_t getSunLux();

};

#endif