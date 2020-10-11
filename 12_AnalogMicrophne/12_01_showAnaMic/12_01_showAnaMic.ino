#define PIN_MIC_IN 33 
#define MICROSEC_INTERRUPT_INTERVAL 45 //22kHz=45
#define MAX_SIZE  16*1024 //131072 // 128*1024
#define NUM_PREREAD_SIZE 2048
unsigned int guiStream[MAX_SIZE];
volatile unsigned int guiCountReadAdc=0;
volatile unsigned int guiPreRead=0;


hw_timer_t *timer1 = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer1() {
    portENTER_CRITICAL_ISR(&timerMux);

    guiStream[guiCountReadAdc] = analogRead(PIN_MIC_IN);

    if (guiCountReadAdc >= (MAX_SIZE-1) )//
    {
        //guiCountReadAdc = 0;
    }
    else
    {
        guiCountReadAdc += 1;
    }

    portEXIT_CRITICAL_ISR(&timerMux);
    xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_MIC_IN, INPUT);

    timerSemaphore = xSemaphoreCreateBinary();
    timer1 = timerBegin(0, 80, true);
    timerAttachInterrupt(timer1, &onTimer1, true);
    timerAlarmWrite(timer1,  MICROSEC_INTERRUPT_INTERVAL , true);//us
    timerAlarmEnable(timer1);

    guiCountReadAdc=0;
    guiPreRead=0;

    unsigned int uiFlag=0;
    while(uiFlag==0)
    {
      if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
          portENTER_CRITICAL(&timerMux);
          if(guiCountReadAdc>NUM_PREREAD_SIZE){
            uiFlag=1;
            guiPreRead=1;
          }
          portEXIT_CRITICAL(&timerMux);
      }//endif xSemaphore
    }
    
    Serial.println("start sampling");

}

void loop()
{
    unsigned int uiCounterVal=0;
     
    if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
        portENTER_CRITICAL(&timerMux);
        uiCounterVal = guiCountReadAdc;
        portEXIT_CRITICAL(&timerMux);
    }//endif xSemaphore

    if ( uiCounterVal >= (MAX_SIZE-1) ){
        for(unsigned int uilp =0;uilp<MAX_SIZE;uilp++){
            Serial.println(guiStream[uilp]);
        }//end for
        Serial.println("end of data");
        timerAlarmDisable(timer1);
        guiCountReadAdc=0;
    }//endif counter
}
