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
float b;

//La condicion inicial del giroscopio no sería necesaria
float angulo_giro_x=0;
float angulo_ace_x=0;
float angulo_complementario=0;
float alpha=0.1;
float periodo=0.01;
float pi=3.1415;


//int duty_max=21;
//int duty_min=265;
int duty_min=106;
int duty_max=230;
int pote_50=606; //pwm 117
int pote_menos_50=203; //pwm 218


//Los límites de phi son -50 y 50 y estan asociados correspondientemente a duty_max y duty_min.
int sensor=14;
int centro = (duty_max + duty_min)/2;
int angulo_referencia = 0;
static float Ts=0.01;
//PI - Back Kp=0.1 - Ki=5
//PI - tust Kp = 0.25 - Ki = 0.05;

//PD - Back y -Tust Kp = 0.5 - Kd = 0.0002 
//PD - sin gomita oscila menos con Kp = 0.3 y Kd = 0.0001. Bajarlo un toque mas.
//Controlador
float Kp = 0.3;
float Ki = 0;
float Kd = 0.0005;


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
  
  b=get_bias();
  delay(100);
}



void loop(){
  unsigned long tiempo_inicial=millis();
  //imu
  sensors_event_t a, g, temp;
  
  mpu.getEvent(&a, &g, &temp);
  static float u = 0;
  //movimiento brazo
  float duty_ang = map(angulo_referencia, 50,-50,duty_min,duty_max);
  float phi_brazo =map(analogRead(sensor),pote_menos_50,pote_50,-50,50);//basicamente queremos nuestro angulo limite de -50 a 50 y vimos la correspondencia con el valor q indica el pote ya armado en el pendulo.
  float pos_pote = analogRead(sensor);

  /*  
  static int cambiar=0;
  if (cambiar == 1700) {
    if (angulo_referencia==0){
      angulo_referencia=30;
      Serial.println(phi_brazo);
      Serial.println(pos_pote);
      
    }
    else{
      angulo_referencia=0;
      Serial.println(phi_brazo);
      Serial.println(pos_pote);
    }
    cambiar = 0;
  }
  cambiar++;
  */
  Timer1.pwm(PWMoutput,map(u,50,-50,duty_min,duty_max));
  
  //Serial.print('.');


 

  angulo_giro_x= angulo_complementario + periodo*g.gyro.x;
  angulo_ace_x= atan2(a.acceleration.y,a.acceleration.z);
  angulo_complementario= (1-alpha)*angulo_giro_x + angulo_ace_x*alpha;

  

  
  //Control
  
  //Proporcional
  static float e;
  
  static float e_prev = 0;
  static float u_prev = 0;
  static float e_acum = 0;
  e = angulo_referencia - (angulo_complementario*(180/pi)-b);
  //u = e * Kp; 

  //PI y PD
  //Back
  
  e_acum = e_acum + Ts * e;
  //u = Kp * e + Ki * e_acum + Kd * (e - e_prev)/Ts;

  //tusting
  //u = u_prev + (Kp + (Ts * (Ki/(Kp*2)))) * e + ((Ts * Ki / (Kp*2)) - Kp) * e_prev;
  static float I_acum = 0;
  static float D_acum = 0;
  
  u = Kp * e + Ki *(I_acum+ (Ts * e)/2 + (Ts * e_prev)/2) + Kd * (2*(e - e_prev)/Ts - D_acum);
  I_acum = I_acum + (Ts * e)/2 + (Ts * e_prev)/2;
  D_acum = 2*(e - e_prev)/Ts - D_acum;
  



  u_prev = u;
  e_prev = e;
  
  matlab_send(u,e,(180*angulo_complementario)/pi - b,0,0,0,0);

  unsigned long tiempo_final=millis();
  static int cont = 0;
  if(cont == 10){
    //Serial.println('.');
    //Serial.println(u);
    //Serial.println(e);
    //Serial.println(angulo_complementario*(180/pi)-b);
    //Serial.println(map(u, 50,-50,duty_min,duty_max));
    //Serial.println(map(analogRead(sensor),pote_menos_50,pote_50,-50,50));
    //Serial.println(tiempo_final-tiempo_inicial);
    cont = 0;
  }
  cont++;

  
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

float get_bias(){
  float bias=0;
  for(int i=0;i<=100;i++){
    unsigned long tiempo_inicial=millis();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    angulo_giro_x= angulo_complementario + periodo*g.gyro.x;
    angulo_ace_x= atan2(a.acceleration.y,a.acceleration.z);
    bias += (1-alpha)*angulo_giro_x + angulo_ace_x*alpha;
    unsigned long tiempo_final=millis();
    delay(10-(tiempo_final-tiempo_inicial));
  }
  Serial.println(bias*(180/pi));
  return (bias*(180/pi))/float(10);
}
