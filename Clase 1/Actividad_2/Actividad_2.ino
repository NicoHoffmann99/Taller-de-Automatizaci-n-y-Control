#include "TimerOne.h"

#define PWMoutput 9
#define PWMperiod 10000 //En micro segundos => 10ms

//int duty_max=21;
//int duty_min=265;
int duty_min=98;
int duty_max=198;
int pote_50=392; //pwm 117
int pote_menos_50=742; //pwm 218
//Los límites de phi son -50 y 50 y estan asociados correspondientemente a duty_max y duty_min.
int sensor=14;
int centro = (duty_max + duty_min)/2;
int angulo_1 = 50;
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
  //float duty=map(analogRead(sensor),0,1023,,198);

  float duty_ang = map(angulo_1, 50,-50,duty_min,duty_max);
  Timer1.pwm(PWMoutput,duty_ang);
  float angulo =map(analogRead(sensor),pote_menos_50,pote_50,-50,50);//basicamente queremos nuestro angulo limite de -50 a 50 y vimos la correspondencia con el valor q indica el pote ya armado en el pendulo.
  float pos_pote = analogRead(sensor);
  //float angulo = map(angulo, -90, 90, 60, 260);
  static int cambiar=0;
  if (cambiar == 700) {
    if (angulo_1==-50){
      angulo_1=50;
    }
    else{
      angulo_1=-50;
    }
    cambiar = 0;
  }
  cambiar++;

  Serial.println(angulo);
  Serial.println(angulo_1);
  Serial.println(duty_ang);
  
  Serial.println(pos_pote);
  
  unsigned long tiempo_final=millis();


  delay(10-(tiempo_final-tiempo_inicial));
}

void timer_callback()
{
}
