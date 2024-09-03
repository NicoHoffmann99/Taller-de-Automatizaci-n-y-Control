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
  float duty=map(analogRead(sensor),0,1023,duty_min,duty_max);
  Timer1.pwm(PWMoutput,duty);
  unsigned long tiempo_final=millis();
  delay(10-(tiempo_final-tiempo_inicial));
}

void timer_callback()
{
}
