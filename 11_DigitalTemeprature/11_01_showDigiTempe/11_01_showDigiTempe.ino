
#include <OneWire.h> //https://github.com/PaulStoffregen/OneWire
#include <DallasTemperature.h> // https://github.com/milesburton/Arduino-Temperature-Control-Library
//ref to example/DallasTemperature/single.ino

#define PIN_NUM_TEMPE 33
#define NUM_BIT_WIDTH_TEMPE 12

OneWire myOneWire(PIN_NUM_TEMPE); 
DallasTemperature myTempSenses(&myOneWire);
DeviceAddress myAryAddressTemp;

void setup(void) 
{ 
	Serial.begin(115200); 
  
	myTempSenses.begin(); 
  myTempSenses.getAddress(myAryAddressTemp, 0);
	myTempSenses.setResolution(myAryAddressTemp,NUM_BIT_WIDTH_TEMPE);
  if( myTempSenses.getDeviceCount()!=1 || myTempSenses.getResolution(myAryAddressTemp)!=12) Serial.println("status error");

} 
void loop(void) 
{ 
 myTempSenses.requestTemperatures();
 float fTempe = myTempSenses.getTempC(myAryAddressTemp);
 Serial.println(fTempe);
 delay(1000); 
} 
