#include <SoftwareSerial.h>

//WeatherStation.ino

#include "./DHT/DHT.h"
#include "./PM25/PM25.h"
#include "./WeatherDev/WeatherDev.h"
#include "./BMP280_IIC_SPI/Adafruit_BMP280.h"

#include <MsTimer2.h>
//#include "DHT.h"
//#include "PM25.h"
//#include "WeatherDev.h"

#define DHTPIN      7   // D7: PD7  what pin we're connected to
#define PM25DPIN    4   // D4: PD4  PM2.5 led power control pin
#define PM25APIN    0   // A0: PC0  PM2.5 voltage sample pin 
#define SUNPIN      1   // A2: PC2  Weatherdev sunshine sample pin 
#define RAINPIN     3   // D3: PD3  Weatherdev raincollect sample pin 
#define WINDDIRPIN  6   // A6: PC6  Weatherdev winddirection sample pin 
#define WINDSPDPIN  2   // D2: PD2  Weatherdev windspeed sample pin 
#define UVPIN       2	// A1: PC1  Weatherdev uvLight sample pin 

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11 

#define MSGMAXRCVLEN 6
#define MSGMAXSENTLEN 20

SoftwareSerial mySerial(8, 9); //rx tx

DHT dht(DHTPIN, DHTTYPE);
PM25 pm25(PM25APIN, PM25DPIN);
WeatherDev weatherDev(SUNPIN, RAINPIN, WINDDIRPIN, WINDSPDPIN, UVPIN);
Adafruit_BMP280 bmp280;


//uint8_t rcvMsgBuf[MSGMAXRCVLEN];
uint8_t sentMsgBuf[MSGMAXSENTLEN];
uint8_t updateFlag = 0;
unsigned long sentTime = 0;

struct WeatherStationMsg
{
	uint8_t sVoltage;
	uint8_t sTemperature;
	uint8_t sHumidity;
	float sPressure;
	uint16_t sWindSpeed;
	uint8_t sWindDirection;
	uint8_t sRainCollect;
	uint8_t sSunShine;
	float sPM25;
	uint8_t sUV;
	//uint8_t sLedState;
	// uint8_t sSwitchState;
	// uint8_t sCarMode;
	uint8_t sWarnValue;
	uint8_t sFault;

	} weatherStationMsg;

	void setup() {
		Serial.begin(9600);
		mySerial.begin(115200);
		Serial.println("WeatherStation test!");
		analogReference(DEFAULT);
		dht.begin();
		pm25.begin();
		weatherDev.begin();
		if(!bmp280.begin()){
			Serial.println("bmp280 dev connect failed, pls check!");
		}
		sentTime = millis();
		attachInterrupt(0, addWindSignalCount, RISING);
		attachInterrupt(1, rainSignal, RISING);
		MsTimer2::set(1000, updateWindSpeed);
		MsTimer2::start();

		pinMode(13, OUTPUT);
		digitalWrite(13, LOW);
	}

	void loop() {
  // Wait a few seconds between measurements.


	  // Reading temperature or humidity takes about 250 milliseconds!
	  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)

	  // sent interval min 1s
	  
	  unsigned long nowTime = millis();
	  if(abs(nowTime - sentTime) > 1000){
	  	updateWeatherMsg();
	  	if(updateFlag == 1){
	  	  // unsigned long nowTime = millis();
	  	  encodeMsg();
	  	  mySerial.write(sentMsgBuf, MSGMAXSENTLEN);
	  	  updateFlag = 0;
	  	  sentTime = millis();

	  	} 
	  }
	  

	  if(mySerial.available()){
	  	uint8_t rcvLen = mySerial.available();
	  	uint8_t rcvMsgBuf[MSGMAXRCVLEN];
	  	if(rcvLen == MSGMAXRCVLEN){
	  		mySerial.readBytes(rcvMsgBuf, rcvLen);
	  		if(decodeMsg(rcvMsgBuf)) {

	  	  			//updateWeatherMsg();


	  	  			encodeMsg();
	  	  			mySerial.write(sentMsgBuf, MSGMAXSENTLEN);
	  	  			}else {

	  	  				//sent the resent msg to android app
	  	  			}  
	  	  		} 

	  	  	}

	  	  	pm25.setDustDensity(false);
	  	  	delay(300);
	  	  }

	  	  void addWindSignalCount() {
	  	  	delayMicroseconds(200);
	  	  	if(digitalRead(WINDSPDPIN)) {
	  	  		weatherDev.setWindSignalCount();
	  	  	}
	  	  }

	  	  void rainSignal() {
	  	  	delayMicroseconds(200);
	  	  	if(digitalRead(RAINPIN)){
	  	  		weatherDev.setRainCollect();

	  	  	}
	  	  }

	  	  void updateWindSpeed() {

	  	  	weatherDev.setWindSpeed(weatherDev.getWindSignalCount());
	  	  	weatherDev.clearWindSignalCount();


	  	  }

	  	  void updateWeatherMsg() {

	  // updateWindSpeed();
	  weatherStationMsg.sVoltage = 0;
	  weatherStationMsg.sWarnValue = 0;
	  weatherStationMsg.sFault = 0;

	  uint8_t tUpdateFlag = 0;
	  uint8_t h = dht.readHumidity();
	  if(isnan(h)){
	  	h = 0;
	  }
	  if(h != weatherStationMsg.sHumidity){
	  	weatherStationMsg.sHumidity = h;
	  	tUpdateFlag = 1;
	  }
	  // Read temperature as Celsius
	  uint8_t t = dht.readTemperature();
	  if(isnan(t)){
	  	t = 0;
	  }
	  if(t != weatherStationMsg.sTemperature){
	  	weatherStationMsg.sTemperature = t;
	  	tUpdateFlag = 1;
	  }


	  // Compute heat index
	  // Must send in temp in Fahrenheit!
	  //float hi = dht.computeHeatIndex(f, h);

	  // Serial.print("Humidity: "); 
	  // Serial.print(h);
	  // Serial.print(" %\t");
	  // Serial.print("Temperature: "); 
	  // Serial.print(t);
	  // Serial.print(" *C ");
	  //Serial.print(f);
	  //Serial.print(" *F\t");
	  //Serial.print("Heat index: ");
	  //Serial.print(hi);
	  //Serial.print(" *F");

	  pm25.setDustDensity(true);
	  float dustDensity =  pm25.getDustDensityMg();
	  if(dustDensity != weatherStationMsg.sPM25){
	  	weatherStationMsg.sPM25 = dustDensity;
	  	tUpdateFlag = 1;
	  }


	  Serial.print("PM25: ");
	  Serial.print(dustDensity); 
	  Serial.print(' ');
	  
	  float tPressure = bmp280.readPressure();
	  if(tPressure != weatherStationMsg.sPressure){
	  	weatherStationMsg.sPressure = tPressure;
	  	tUpdateFlag = 1;
	      //Serial.println(weatherStationMsg.sPressure);
	  }
	  // Serial.print("Pressure: ");
	  // Serial.print(tPressure);
	  // Serial.print(' ');
	  uint8_t tSunshine = weatherDev.getSunShine();
	  if(tSunshine != weatherStationMsg.sSunShine){
	  	weatherStationMsg.sSunShine = tSunshine;

	  	tUpdateFlag = 1;
	  }
	  Serial.print("Sunshine: ");
	  Serial.println(weatherStationMsg.sSunShine);
	  //uint16_t tSunshine = analogRead(2);
	  // Serial.print("SunShine: ");
	  // Serial.print(tSunshine);
	  // Serial.print(' ');
	  uint8_t tRainCollect = weatherDev.getRainCollect();
	  Serial.print(tRainCollect);
	  Serial.println();
	  if(tRainCollect != weatherStationMsg.sRainCollect){
	  	weatherStationMsg.sRainCollect = tRainCollect;
	  	tUpdateFlag = 1;
	  }
	  // Serial.print("Rain: ");
	  // Serial.print(tRainCollect);
	  // Serial.print(' ');
	  uint8_t tWindDirection = weatherDev.getWindDirection();
	  if(tWindDirection != weatherStationMsg.sWindDirection){
	  	weatherStationMsg.sWindDirection = tWindDirection;
	  	tUpdateFlag = 1;
	  }
	  // Serial.print("WindDirection: ");
	  // Serial.print(tWindDirection);
	  // Serial.print(' ');
	  uint16_t tWindSpeed = weatherDev.getWindSpeed();
	  if(tWindSpeed != weatherStationMsg.sWindSpeed){
	  	weatherStationMsg.sWindSpeed = tWindSpeed;
	  	tUpdateFlag = 1;
	  }
	  // Serial.print("WindSpeed: ");
	  // Serial.print(tWindSpeed);
	  // Serial.print(' ');

	  uint8_t tUvLight = weatherDev.getUvLight();
	  weatherStationMsg.sUV = tUvLight;
	  // Serial.print("UVLight: ");
	  // Serial.println(tUvLight);
	  //Serial.println(tUpdateFlag);
	  if(tUpdateFlag == 1){
	  	updateFlag = 1;
	  }
	}

//Format: Header(ffff) cmd(1B) sn(1B) flags(1B) action(1B) dev_statue(18B) chksum(1B)
void encodeMsg() {

	sentMsgBuf[0] = 0xff;
	sentMsgBuf[1] = 0xff;  // msg header
	sentMsgBuf[2] = 0x05; //cmd 
	sentMsgBuf[3] = 0x00; //flags
	sentMsgBuf[4] = 0x04; //actions
	sentMsgBuf[5] = weatherStationMsg.sVoltage; //voltage
	sentMsgBuf[6] = weatherStationMsg.sTemperature;
	sentMsgBuf[7] = weatherStationMsg.sHumidity;
	uint16_t tVal = (uint16_t) (weatherStationMsg.sPressure * 100);
	
	sentMsgBuf[8] = (uint8_t)(tVal >> 8);   // pressure high bits
	sentMsgBuf[9] = (uint8_t)(tVal & 0xff); // pressure low bits
	sentMsgBuf[10] = (uint8_t)(weatherStationMsg.sWindSpeed >> 8);
	sentMsgBuf[11] = (uint8_t)(weatherStationMsg.sWindSpeed & 0xff);
	sentMsgBuf[12] = weatherStationMsg.sWindDirection;
	sentMsgBuf[13] = weatherStationMsg.sRainCollect;
	sentMsgBuf[14] = weatherStationMsg.sSunShine;
	tVal = (uint16_t) (weatherStationMsg.sPM25 * 100);
	sentMsgBuf[15] = (uint8_t)(tVal >> 8);
	sentMsgBuf[16] = (uint8_t)(tVal & 0xff);
	sentMsgBuf[17] = weatherStationMsg.sUV;
	sentMsgBuf[18] = weatherStationMsg.sWarnValue;
	sentMsgBuf[19] = weatherStationMsg.sFault;

}
//Format: Header(0xffff) cmd(1B) sn(1B) flags(1B) action(1B) Attr_flags(2B) Attr_vals(12B) 
boolean decodeMsg(uint8_t rcvMsgBuf[]) {


	for(int i=0; i< MSGMAXRCVLEN; i++){
		Serial.print(rcvMsgBuf[i]);
		Serial.print(' ');
	}
	Serial.println();
	//check the msg header
	if(rcvMsgBuf[0] != 0xff ){
		Serial.print("check header failed");
		return false;
	}
	if(rcvMsgBuf[1] != 0xff ){
		Serial.print("check header failed");
		return false;
	}

	Serial.println("header");
	//check the cmd
	switch (rcvMsgBuf[2]) {
		case 0x03:
	      // do something
	      break;
	      default:

	      return false;
	  }
	  Serial.println("cmd");
	//check the flags
	if(rcvMsgBuf[3] != 0x01){
		return false;
	}
	Serial.println("flags");
	//check the actions
	switch (rcvMsgBuf[4]) {
		case 0x02:
		Serial.println("actions");
	    	//get the device statue from application
	    	if(bitRead(rcvMsgBuf[5], 0) == 1){
	    		Serial.println("decode success");
	    		updateWeatherMsg();

	    	}

	    	break;
	    	case 0x01:
	      // do something
	      break;
	      default:
	      return false;
	  }

	  return true;
	}