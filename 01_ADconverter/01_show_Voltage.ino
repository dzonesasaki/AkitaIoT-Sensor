float gfAveragedValue;

void setup() {
  Serial.begin(115200);
  gfAveragedValue = 0;
}

void loop() {
  gfAveragedValue = analogRead(33) / (float)4096 * 3.3  * 0.1 + gfAveragedValue * (1-0.1); 
  Serial.println(gfAveragedValue);
  delay(10);
}
