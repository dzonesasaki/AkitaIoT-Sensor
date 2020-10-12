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
#define NUM_SEC_TIMER (10)
#define NUM_TIMER_MICROSEC (NUM_SEC_TIMER*NUM_1SEC_MICROSEC)
hw_timer_t * handleTimer = NULL; 

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; 

volatile int gviCountWatchDog=0;
#define NUM_MAX_WATCHDOG (6) // 60sec 


void IRAM_ATTR irq_Timer01()
{
  portENTER_CRITICAL_ISR(&timerMux);
  gviCountWatchDog++;
  if (gviCountWatchDog > NUM_MAX_WATCHDOG)
  {
    selfRestartProcess();
  }

  portEXIT_CRITICAL_ISR(&timerMux);
}//irq_Timer01()

void selfRestartProcess(void)
{
  //memory copy to flash
  ESP.restart(); // reboot
}//selfRestartProcess()

void init_TimerInterrupt()
{
  gviCountWatchDog =0;
  handleTimer = timerBegin(0, 80, true); // Num_timer ,  counter per clock 
  timerAttachInterrupt(handleTimer, &irq_Timer01, true);
  timerAlarmWrite(handleTimer, NUM_TIMER_MICROSEC, true); //[us] per 80 clock @ 80MHz
  timerAlarmEnable(handleTimer);
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

void setup() {
  Serial.begin(115200);
  gviCountWatchDog=0; 
  init_TimerInterrupt();
  initWifiClient();
  connectMqtt();

}

void loop(){
 Serial.println("start loop");
  portENTER_CRITICAL(&timerMux);
    gviCountWatchDog=0; //clear Counter
  portEXIT_CRITICAL(&timerMux);

  float fSensValue=gu32Count;

  //char cValue[11];
  char *cValue;
  cValue = (char *)malloc(12);
  sprintf(cValue, "%1.5e", fSensValue);

  doPubMqtt(cValue);
  free(cValue);
  
  delay(2000);
  gu32Count++;

}
