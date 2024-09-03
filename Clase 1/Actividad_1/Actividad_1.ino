unsigned long tiempo;
int sensor = 14;
int val = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long tiempo_inicial=micros();
  Serial.println(map(analogRead(sensor),0,1023,0,280));
  unsigned long tiempo_final=micros();
  delayMicroseconds(10000-(tiempo_final-tiempo_inicial));
}
