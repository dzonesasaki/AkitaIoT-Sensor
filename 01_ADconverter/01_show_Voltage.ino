void setup() {
  Serial.begin(115200);
}

const float FACT_VOLTAGE = 3.3/(float)4096;
const float WEIGHT_AVERAGE = 1/(float)10;
const uint32_t NLOOP = 1000;
float gfAveragedValue =0;

void loop() {
  float currentValue = 0;

  for(int i=0;i<NLOOP;i++)
  {
    currentValue = analogRead(33)*FACT_VOLTAGE;
    gfAveragedValue = currentValue * WEIGHT_AVERAGE + gfAveragedValue * (1-WEIGHT_AVERAGE); 
    delay(1);
  }  
  Serial.println(gfAveragedValue);
  delay(1);
}
