// Basic demo for accelerometer readings from Adafruit MPU6050
#include "TimerOne.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <stdint.h>

Adafruit_MPU6050 mpu;
//defines de pwm pines y periodo
#define PWMoutput 9
#define PWMperiod 10000 //En micro segundos => 10ms
int sensor=14;

//Condiciones inciales
float bi;

//Matrices Digitializadas del Modelo
float Ad[4][4] = {{1.0, 0, 0.01, 0},{0, 1, 0, 1},{-0.5340, -0.0026, 2.7199, 0.2185},{0, -2.4570, 0, 0.6878}};
float Bd[4] = {0, 0, -1.7199, 2.4570};
float Cd[2][4] = {{1, 0, 0, 0},{0, 1, 0, 0}};
float Dd[2]= {0, 0};
float L[4][2] = {{2.2683, 0.2185},{0.0, 0.2362},{389.9529, 42.7388},{0.0, -9.4742}};

//constantes del pote
int duty_min=106;
int duty_max=230;
int pote_50=606; //pwm 117
int pote_menos_50=203; //pwm 218

//La condicion inicial del giroscopio no sería necesaria
float angulo_giro_x=0;
float angulo_ace_x=0;
float angulo_complementario=0;
float velocidad_pendulo = 0;

float velocidad_brazo = 0;
float angulo_brazo_ant = 0;

//inicializado de estimadores
float vang_pendulo_k = 0;
float vang_pendulo_k1 = 0;
float ang_pendulo_k = 0;
float ang_pendulo_k1 = 0;
float ang_brazo_k = 0;
float ang_brazo_k1 = 0;
float vang_brazo_k = 0;
float vang_brazo_k1 = 0; 

//valores para filtro complementario
float alpha=0.1;
float Ts=0.01;
float pi=3.1415;

void setup(void) {
  
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
  
  bi=get_bias();
  delay(100);;
}

void loop() {
  //Tiempo de lectura de datos + transformacion
  //static unsigned long dif;
  unsigned long t_inicial=micros();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  //angulo
  angulo_giro_x= angulo_complementario + Ts*g.gyro.x;
  angulo_ace_x= atan2(a.acceleration.y,a.acceleration.z);
  angulo_complementario= ((1-alpha)*angulo_giro_x + angulo_ace_x*alpha);

  //float duty_ang = map(angulo_referencia, 50,-50,duty_min,duty_max);
  float angulo_brazo_act =map(analogRead(sensor),pote_menos_50,pote_50,-50,50);//basicamente queremos nuestro angulo limite de -50 a 50 y vimos la correspondencia con el valor q indica el pote ya armado en el pendulo.
  //float pos_pote = analogRead(sensor);

  //velocidad
  velocidad_pendulo = (g.gyro.x)*(180 / pi);
  velocidad_brazo = (angulo_brazo_act - angulo_brazo_ant) / Ts;
  angulo_brazo_ant = angulo_brazo_act; 

  //observador
  ang_pendulo_k1 = Ad[0][0] * ang_pendulo_k + Ad[0][1] * ang_brazo_k + Ad[0][2] * vang_pendulo_k + Ad[0][3] * vang_brazo_k + L[0][0] * (angulo_complementario - ang_pendulo_k) + L[0][1] * (angulo_brazo_act - ang_brazo_k); //sin Bd pq no controlamos todavia
  ang_brazo_k1 = Ad[1][0] * ang_pendulo_k + Ad[1][1] * ang_brazo_k + Ad[1][2] * vang_pendulo_k + Ad[1][3] * vang_brazo_k + L[1][0] * (angulo_complementario - ang_pendulo_k) + L[1][1] * (angulo_brazo_act - ang_brazo_k);
  vang_pendulo_k1 = Ad[2][0] * ang_pendulo_k + Ad[2][1] * ang_brazo_k + Ad[2][2] * vang_pendulo_k + Ad[2][3] * vang_brazo_k + L[2][0] * (angulo_complementario - ang_pendulo_k) + L[2][1] * (angulo_brazo_act - ang_brazo_k);
  vang_brazo_k1 = Ad[3][0] * ang_pendulo_k + Ad[3][1] * ang_brazo_k + Ad[3][2] * vang_pendulo_k + Ad[3][3] * vang_brazo_k + L[3][0] * (angulo_complementario - ang_pendulo_k) + L[3][1] * (angulo_brazo_act - ang_brazo_k);
  
  //actualizacion estados
  ang_pendulo_k = ang_pendulo_k1;
  ang_brazo_k = ang_brazo_k1;
  vang_pendulo_k = vang_pendulo_k1;
  vang_brazo_k = vang_brazo_k1;

  //mediciones a rad/s
  //float ang_comp = angulo_complementario * 180 / pi;  
  //float vel_ang  = velocidad_pendulo * 180 / pi;

  //estimaciones a rad/s
  //float angulo_k = ang_k * 180/pi;
  //float vel_ang_k  = vang_k * 180 / pi;


  matlab_send(angulo_complementario*(180/pi), ang_pendulo_k, velocidad_pendulo, vang_pendulo_k, angulo_brazo_act, ang_brazo_k1, velocidad_brazo, vang_brazo_k);

  unsigned long t_final= micros();
  //dif = t_final - t_inicial;
  delayMicroseconds(10000-(t_final-t_inicial));
}

void matlab_send(float ace1, float ace2, float ace3, float giro1, float giro2, float giro3, float temp1, float temp2){
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
  b = (byte *) &temp1;
  Serial.write(b,4);
  b = (byte *) &temp2;
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
    angulo_giro_x= angulo_complementario + Ts*g.gyro.x;
    angulo_ace_x= atan2(a.acceleration.y,a.acceleration.z);
    bias += (1-alpha)*angulo_giro_x + angulo_ace_x*alpha;
    unsigned long tiempo_final=millis();
    delay(10-(tiempo_final-tiempo_inicial));
  }
  Serial.println(bias*(180/pi));
  return (bias*(180/pi))/float(10);
}

