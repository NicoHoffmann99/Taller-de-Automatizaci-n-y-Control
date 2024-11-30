#include "TimerOne.h"

#define PWMoutput 9
#define PWMperiod 10000 //En micro segundos => 10ms

int duty_max=21;
int duty_min=265;
int sensor=14;

void setup()
{
  pinMode(PWMoutput, OUTPUT);
  Timer1.initialize(PWMperiod);              // initialize timer1, and set period in microseconds
  Timer1.pwm(PWMoutput, 0);                  // setup pwm, X duty cycle
  Timer1.attachInterrupt(timer_callback);    // attaches callback() as a timer overflow interrupt

  Serial.begin(115200);
}

void loop(){
  unsigned long tiempo_inicial=millis();
  
  int pote = analogRead(sensor);
  float duty=map(pote,0,1023,duty_min,duty_max);
  Serial.println(map(analogRead(sensor),0,1023,0,280));
  //Timer1.pwm(PWMoutput,duty);
  
  //Serial.println("duty servo: %f ",duty);
  //Serial.println("lectura pote: %d", pote);
  
  Serial.println(pote);
  
  unsigned long tiempo_final=millis();
  delay(10-(tiempo_final-tiempo_inicial));
}

void timer_callback()
{
}
