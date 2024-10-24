#include "TimerOne.h"

#define PWMoutput 9
#define PWMperiod 10000 //En micro segundos => 10ms

int duty_max=21;
int duty_min=265;
int centro = (duty_max + duty_min)/2; //es para ver el 0 del servo


void setup() {
  // put your setup code here, to run once:
  pinMode(PWMoutput, OUTPUT);
  Timer1.initialize(PWMperiod);              // initialize timer1, and set period in microseconds
  Timer1.pwm(PWMoutput, 0);                  // setup pwm, X duty cycle
  Timer1.attachInterrupt(timer_callback);    // attaches callback() as a timer overflow interrupt

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long tiempo_inicial=millis();
  //float duty=map(,0,1023,duty_min,duty_max);
  Timer1.pwm(PWMoutput,centro);
  unsigned long tiempo_final=millis();
  delay(10-(tiempo_final-tiempo_inicial));
}
