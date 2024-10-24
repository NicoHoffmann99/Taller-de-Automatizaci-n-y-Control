#include "TimerOne.h"

// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

//defines de pwm pines y periodo
#define PWMoutput 9
#define PWMperiod 10000 //En micro segundos => 10ms


//Condiciones inciales

//La condicion inicial del giroscopio no sería necesaria
float angulo_giro_x=0;
float angulo_ace_x=0;
float angulo_complementario=0;
float alpha=0.1;
float periodo=0.01;
float pi=3.1415;


//int duty_max=21;
//int duty_min=265;
int duty_min=117;
int duty_max=218;
int pote_50=566; //pwm 117
int pote_menos_50=232; //pwm 218

//Los límites de phi son -50 y 50 y estan asociados correspondientemente a duty_max y duty_min.
int sensor=14;
int centro = (duty_max + duty_min)/2;
int angulo_referencia = 0;
void setup()
{ 
  
  Serial.begin(115200);
  //setup pwm
  pinMode(PWMoutput, OUTPUT);
  Timer1.initialize(PWMperiod);              // initialize timer1, and set period in microseconds
  Timer1.pwm(PWMoutput, 0);                  // setup pwm, X duty cycle
  Timer1.attachInterrupt(timer_callback);    // attaches callback() as a timer overflow interrupt

  //setup imu
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  delay(100);
}

void loop(){
  unsigned long tiempo_inicial=millis();
  
  //movimiento brazo
  float duty_ang = map(angulo_referencia, 50,-50,duty_min,duty_max);
  float phi_brazo =map(analogRead(sensor),235,567,-50,50);//basicamente queremos nuestro angulo limite de -50 a 50 y vimos la correspondencia con el valor q indica el pote ya armado en el pendulo.
  float pos_pote = analogRead(sensor);
  //float angulo = map(angulo, -90, 90, 60, 260);
  static int cambiar=0;
  if (cambiar == 500) {
    if (angulo_referencia==0){
      angulo_referencia=30;
    }
    else{
      angulo_referencia=0;
    }
    cambiar = 0;
  }
  cambiar++;
  Timer1.pwm(PWMoutput,duty_ang);
  //Serial.println(phi_brazo);
  //Serial.println(angulo_referencia);
  //Serial.println(pos_pote);
  


  //imu
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  angulo_giro_x= angulo_complementario + periodo*g.gyro.x;
  angulo_ace_x= atan2(a.acceleration.y,a.acceleration.z);
  angulo_complementario= (1-alpha)*angulo_giro_x + angulo_ace_x*alpha;


  //matlab_send(angulo_referencia,phi_brazo,(180*angulo_complementario)/pi,0,0,0,0);
  
  
  unsigned long tiempo_final=millis();
  
  Serial.println(tiempo_final-tiempo_inicial);
  delay(10-(tiempo_final-tiempo_inicial));
}

void matlab_send(float ace1, float ace2, float ace3, float giro1, float giro2, float giro3, float temp){
  Serial.write("abcd");
  byte * b = (byte *) &ace1;
  Serial.write(b,4);
  b = (byte *) &ace2;
  Serial.write(b,4);
  b = (byte *) &ace3;
  Serial.write(b,4);
  b = (byte *) &giro1;
  Serial.write(b,4);
  b = (byte *) &giro2;
  Serial.write(b,4);
  b = (byte *) &giro3;
  Serial.write(b,4);
  b = (byte *) &temp;
  Serial.write(b,4);
  //etc con mas datos tipo float. Tambien podría pasarse como parámetro a esta funcion un array de floats.
}

void timer_callback()
{
}
