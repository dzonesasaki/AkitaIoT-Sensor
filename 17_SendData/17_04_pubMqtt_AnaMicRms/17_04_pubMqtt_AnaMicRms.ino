#include <WiFi.h>
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient

char *ssidC="SSID";
char *passC="PASSWORD";
char* servC="192.168.1.1";

const char* topic = "myTopic";
const String clientId = "myMqttPub" ;

WiFiClient myWifiClient;
const int mqttPort = 1883;
PubSubClient mqttClient(myWifiClient);

uint32_t gu32Count=0;

#define NUM_1SEC_MICROSEC (1000000)
#define NUM_SEC_WDTIMER (10)
#define NUM_WDTIMER_MICROSEC (NUM_SEC_WDTIMER*NUM_1SEC_MICROSEC)
#define NUM_MAX_WATCHDOG (6) // 60sec 
hw_timer_t * handleTimer2 = NULL; 
portMUX_TYPE timer2Mux = portMUX_INITIALIZER_UNLOCKED; 
volatile int gviCountWatchDog=0;




#define PIN_MIC_IN 33 
#define MICROSEC_INTERRUPT_INTERVAL 45 //22kHz=45
#define MAX_SIZE  16*1024 //131072 // 128*1024
#define NUM_PREREAD_SIZE 2048
hw_timer_t *handletimer0 = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timer0Mux = portMUX_INITIALIZER_UNLOCKED; //rer to https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/

unsigned int guiStream[MAX_SIZE];
volatile unsigned int guiCountReadAdc=0;
volatile unsigned int guiPreRead=0;



void IRAM_ATTR isr_timer0() {
    portENTER_CRITICAL_ISR(&timer0Mux);

    guiStream[guiCountReadAdc] = analogRead(PIN_MIC_IN);

    if (guiCountReadAdc >= (MAX_SIZE-1) )//
    {
        //guiCountReadAdc = 0;
    }
    else
    {
        guiCountReadAdc += 1;
    }

    portEXIT_CRITICAL_ISR(&timer0Mux);
    xSemaphoreGiveFromISR(timerSemaphore, NULL);
}


void IRAM_ATTR irq_Timer2()
{
  portENTER_CRITICAL_ISR(&timer2Mux);
  gviCountWatchDog++;
  if (gviCountWatchDog > NUM_MAX_WATCHDOG)
  {
    selfRestartProcess();
  }

  portEXIT_CRITICAL_ISR(&timer2Mux);
}//irq_Timer02()


void selfRestartProcess(void)
{
  //memory copy to flash
  ESP.restart(); // reboot
}//selfRestartProcess()

void init_TimerInterrupt2()
{
  gviCountWatchDog =0;
  handleTimer2 = timerBegin(2, 80, true); // Num_timer ,  counter per clock 
  timerAttachInterrupt(handleTimer2, &irq_Timer2, true);
  timerAlarmWrite(handleTimer2, NUM_WDTIMER_MICROSEC, true); //[us] per 80 clock @ 80MHz
  timerAlarmEnable(handleTimer2);
}//init_TimerInterrupt()


void init_TimerInterrupt0()
{
	timerSemaphore = xSemaphoreCreateBinary();
	handletimer0 = timerBegin(0, 80, true);
	timerAttachInterrupt(handletimer0, &isr_timer0, true);
	timerAlarmWrite(handletimer0,  MICROSEC_INTERRUPT_INTERVAL , true);//us
	timerAlarmEnable(handletimer0);
}//init_TimerInterrupt()


void initWifiClient(void){
  Serial.print("Connecting to ");
  Serial.println(ssidC);
  uint16_t tmpCount =0;
  
  WiFi.begin( ssidC, passC);
  while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.print(".");
  tmpCount++;
  if(tmpCount>128)
  {
    //gFlagWiFiConnectFailure = true;
    Serial.println("failed  ");
    return;
  }
}
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void connectMqtt()
{
  mqttClient.setServer(servC, mqttPort);
  while( ! mqttClient.connected() )
  {
    Serial.println("Connecting to MQTT Broker...");
    //String clientId = "myMqttPub" ;
    if ( mqttClient.connect(clientId.c_str()) )
    {
      Serial.println("connected"); 
    }
  }
}

void doPubMqtt(char * cValue)
{
  if(!mqttClient.connected())
  { 
    if (mqttClient.connect(clientId.c_str()) )
    {
      Serial.println("reconnected to MQTT broker"); 
    }
    else
    {
      Serial.println("failure to reconnect to MQTT broker");
      return;
    }
  }
  //mqttClient.loop();
  boolean bFlagSucceed=mqttClient.publish(topic,cValue);
  //free(cValue);
  if(bFlagSucceed)
   {
    Serial.print("done: ");
    Serial.println(cValue);
  }
  else
    Serial.println("failure");
  // run to check: mosquitto_sub -t myTopic
}

void preread()
{
    unsigned int uiFlag=0;
    while(uiFlag==0)
    {
      if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
          portENTER_CRITICAL(&timer0Mux);
          if(guiCountReadAdc>NUM_PREREAD_SIZE){
            uiFlag=1;
            guiPreRead=1;
          }
          portEXIT_CRITICAL(&timer0Mux);
      }//endif xSemaphore
    }
}

float calc_rms(){
	uint32_t tmpSum=0;
	float tmpVal=0;
	for( uint16_t uilp=0;uilp<(MAX_SIZE-1);uilp++){
		tmpSum += guiStream[uilp];
	}
	float fDcOffset = (float)tmpSum/(MAX_SIZE-1);

	for( uint16_t uilp=0;uilp<(MAX_SIZE-1);uilp++){
		tmpVal = (float)guiStream[uilp]-fDcOffset;
		tmpSum += tmpVal*tmpVal;
	}
	float fMean = (float)tmpSum/(MAX_SIZE-1);
	return(sqrt(fMean));
}

void setup() {
  Serial.begin(115200);
  gviCountWatchDog=0; 
  init_TimerInterrupt2();
  init_TimerInterrupt0();
  initWifiClient();
  connectMqtt();
  
  guiCountReadAdc=0;
  guiPreRead=0;
  preread();

}

void loop(){
 Serial.println("start loop");
  portENTER_CRITICAL(&timer2Mux);
    gviCountWatchDog=0; //clear Counter
  portEXIT_CRITICAL(&timer2Mux);

  float fSensValue=gu32Count;
  portENTER_CRITICAL(&timer0Mux);
    unsigned int uiCounterVal=guiCountReadAdc;
  portEXIT_CRITICAL(&timer0Mux);
 
  if ( uiCounterVal >= (MAX_SIZE-1) ){
     fSensValue = calc_rms();
     char *cValue;
     cValue = (char *)malloc(12);
     sprintf(cValue, "%1.5e", fSensValue);
     doPubMqtt(cValue);
     free(cValue);
     guiCountReadAdc=0;
  }

  //char cValue[11];
  
  //vTaskDelay(2000);
  gu32Count++;

}
