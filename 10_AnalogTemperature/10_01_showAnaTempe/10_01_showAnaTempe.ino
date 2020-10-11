float gfAveragedTemperature;

void setup() {
  Serial.begin(115200);
  pinMode(33,INPUT);
  gfAveragedTemperature = 0;
}

void loop() {
  float fCurrentVoltage = analogRead(33) / (float)4096 * 3.3 ;
  float fCurrentTemperature = (fCurrentVoltage - 0.424) * 160;
  gfAveragedTemperature = fCurrentTemperature  * 0.1 + gfAveragedTemperature * (1-0.1); 
  Serial.println(gfAveragedTemperature);
  delay(10);
}
