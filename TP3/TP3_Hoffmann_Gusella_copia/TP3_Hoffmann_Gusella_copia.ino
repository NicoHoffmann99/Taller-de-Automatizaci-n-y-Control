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

//Matrices Digitializadas del Modelo
float Ad[4][4] = {{0.9973, 0.0077, 0.01, 0.0010},{0, 0.9890, 0, 0.0086},{-0.5328, 1.4558, 0.9948, 0.1942},{0, -2.0846, 0, 0.7219}};
float Bd[4] = {-0.0077, 0.0110, -1.4558, 2.0846};
float Cd[2][4] = {{1, 0, 0, 0},{0, 1, 0, 0}};
float Dd[2]= {0, 0};
float L[4][2] = {{0.215, 0.13},{0.097, 0.0113},{1.6, 0.3},{-0.7800, -0.57}};
//float L[4][2] = {{0.3184, 0.0132},{0.0921, 0.0161},{3.6270, -2.1184},{-0.7462, -0.6125}};
//Matrices de Control por realimentación de estados
float K[4] = {0.6021 ,  0.65051 ,  -0.1067 ,  -0.0990};
//Matriz de feedfordward
float F[2] = {0.0000, 0.2925};
//Accion integral
float H = 0.725;

//constantes del pote
int duty_min=103;
int duty_max=230;
int pote_50=606; //pwm 117
int pote_menos_50=203; //pwm 218

//La condicion inicial del giroscopio
float angulo_giro_x=0;
float angulo_ace_x=0;
float angulo_complementario=0;
float velocidad_pendulo = 0;
float angulo_theta = 0;
float bi_pendulo = 0;
float bi_vel = 0;
float velocidad_brazo = 0;
float angulo_brazo_ant = 0;
float angulo_brazo_act = 0;
float angulo_referencia = 10;
float tita_referencia = 0;
float acc_u = 0;

//inicializado de estimadores
float vang_pendulo_k = 0;
float vang_pendulo_k1 = 0;
float ang_pendulo_k = 0;
float ang_pendulo_k1 = 0;
float ang_brazo_k = 0;
float ang_brazo_k1 = 0;
float vang_brazo_k = 0;
float vang_brazo_k1 = 0; 
float q_k = 0;
float q_k1 = 0;
//valores para filtro complementario
float alpha=0.1;
float Ts=0.01;
float pi=3.1415;

sensors_event_t a, g, temp;

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
  bi_pendulo=get_bias_angulo_IMU();
  bi_vel = get_bias_gyro_IMU();
  delay(100);
}



void loop() {
  //Tiempo de lectura de datos + transformacion
  unsigned long t_inicial=micros();

  angulo_theta = get_angulo_IMU();
  angulo_brazo_act = map(analogRead(sensor),pote_menos_50,pote_50,50,-50);
  //Serial.println(angulo_theta);
  //velocidad
  velocidad_pendulo = (g.gyro.x)*(180 / pi) - bi_vel;
  //Serial.println(angulo_theta);
  velocidad_brazo = (angulo_brazo_act - angulo_brazo_ant) / Ts;
  angulo_brazo_ant = angulo_brazo_act; 
  //angulo_referencia = angulo_brazo_act;

  //observador
  
  ang_pendulo_k1 = Ad[0][0] * ang_pendulo_k + Ad[0][1] * ang_brazo_k + Ad[0][2] * vang_pendulo_k + Ad[0][3] * vang_brazo_k + L[0][0] * (angulo_theta - ang_pendulo_k) + L[0][1] * (angulo_brazo_act - ang_brazo_k) + Bd[0] * angulo_referencia;
  ang_brazo_k1 =   Ad[1][0] * ang_pendulo_k + Ad[1][1] * ang_brazo_k + Ad[1][2] * vang_pendulo_k + Ad[1][3] * vang_brazo_k + L[1][0] * (angulo_theta - ang_pendulo_k) + L[1][1] * (angulo_brazo_act - ang_brazo_k) + Bd[1] * angulo_referencia;
  vang_pendulo_k1 =Ad[2][0] * ang_pendulo_k + Ad[2][1] * ang_brazo_k + Ad[2][2] * vang_pendulo_k + Ad[2][3] * vang_brazo_k + L[2][0] * (angulo_theta - ang_pendulo_k) + L[2][1] * (angulo_brazo_act - ang_brazo_k) + Bd[2] * angulo_referencia;
  vang_brazo_k1 =  Ad[3][0] * ang_pendulo_k + Ad[3][1] * ang_brazo_k + Ad[3][2] * vang_pendulo_k + Ad[3][3] * vang_brazo_k + L[3][0] * (angulo_theta - ang_pendulo_k) + L[3][1] * (angulo_brazo_act - ang_brazo_k) + Bd[3] * angulo_referencia;
  

  /*
  //realimentación de estados
  ang_pendulo_k1 = Ad[0][0] * ang_pendulo_k + Ad[0][1] * ang_brazo_k + Ad[0][2] * vang_pendulo_k + Ad[0][3] * vang_brazo_k + L[0][0] * (angulo_theta - ang_pendulo_k) + L[0][1] * (angulo_brazo_act - ang_brazo_k) + Bd[0]*(acc_u);
  ang_brazo_k1 = Ad[1][0] * ang_pendulo_k + Ad[1][1] * ang_brazo_k + Ad[1][2] * vang_pendulo_k + Ad[1][3] * vang_brazo_k + L[1][0] * (angulo_theta - ang_pendulo_k) + L[1][1] * (angulo_brazo_act - ang_brazo_k) + Bd[1] * (acc_u);
  vang_pendulo_k1 = Ad[2][0] * ang_pendulo_k + Ad[2][1] * ang_brazo_k + Ad[2][2] * vang_pendulo_k + Ad[2][3] * vang_brazo_k + L[2][0] * (angulo_theta - ang_pendulo_k) + L[2][1] * (angulo_brazo_act - ang_brazo_k) + Bd[2] * (acc_u);
  vang_brazo_k1 = Ad[3][0] * ang_pendulo_k + Ad[3][1] * ang_brazo_k + Ad[3][2] * vang_pendulo_k + Ad[3][3] * vang_brazo_k + L[3][0] * (angulo_theta - ang_pendulo_k) + L[3][1] * (angulo_brazo_act - ang_brazo_k) + Bd[3] * (acc_u);

  //acc_u = K[0]*ang_pendulo_k1 + K[1] * ang_brazo_k1 + K[2] * vang_pendulo_k1 + K[3]*vang_brazo_k1;
  //FeedFordward
  //acc_u  = K[0]*ang_pendulo_k1 + K[1]*ang_brazo_k1 + K[2]*vang_pendulo_k1 + K[3]*vang_brazo_k1 + F[1]*angulo_referencia + F[0]*tita_referencia;
  
  //Acción integral
  q_k1 = q_k + Ts*(angulo_referencia - ang_brazo_k1);
  acc_u  = K[0]*ang_pendulo_k1 + K[1]*ang_brazo_k1 + K[2]*vang_pendulo_k1 + K[3]*vang_brazo_k1 + H*q_k;

  float duty_acc = map(acc_u,-50,50,duty_min,duty_max);
  Timer1.pwm(PWMoutput,duty_acc);
  */
  

  //actualizacion estados
  ang_pendulo_k = ang_pendulo_k1;
  ang_brazo_k = ang_brazo_k1;
  vang_pendulo_k = vang_pendulo_k1;
  vang_brazo_k = vang_brazo_k1;
  q_k = q_k1;
  //Serial.println(ang_pendulo_k);
  

  //Rutina Movimiento Brazo
  
  static int cambiar=0;
  if (cambiar == 2000) {
    if (angulo_referencia==-15){
      angulo_referencia=10;
    }
    else if (angulo_referencia == 10){
      angulo_referencia=-15;
    }
    //else{
     // angulo_referencia=20;
    //}
    cambiar = 0;
  }
  cambiar++;
  
  float duty_ang = map(angulo_referencia,-50,50,duty_min,duty_max);
  Timer1.pwm(PWMoutput,duty_ang);
  

  matlab_send(angulo_theta, ang_pendulo_k, velocidad_pendulo, vang_pendulo_k, angulo_brazo_act, ang_brazo_k, velocidad_brazo, vang_brazo_k, angulo_referencia);
  unsigned long t_final= micros();
  delayMicroseconds(10000-(t_final-t_inicial));
}

void matlab_send(float ace1, float ace2, float ace3, float giro1, float giro2, float giro3, float temp1, float temp2, float phi_ref){
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
  b = (byte *) &phi_ref;
  Serial.write(b,4);
}

void timer_callback()
{
}

float get_bias_angulo_IMU(){
  float bias=0;
  for(int i=0;i<=100;i++){
    unsigned long tiempo_inicial=millis();
    mpu.getEvent(&a, &g, &temp);
    angulo_giro_x= angulo_complementario + Ts*g.gyro.x;
    angulo_ace_x= atan2(a.acceleration.y,a.acceleration.z);
    bias += (1-alpha)*angulo_giro_x + angulo_ace_x*alpha;
    unsigned long tiempo_final=millis();
    delay(10-(tiempo_final-tiempo_inicial));
  }
  return (bias*(180/pi))/float(10);
}

float get_angulo_IMU(){
  //sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  angulo_giro_x= (angulo_complementario + (Ts*g.gyro.x)*(180/pi));
  angulo_ace_x= atan2(a.acceleration.y,a.acceleration.z)*(180/pi);
  angulo_complementario= ((1-alpha)*angulo_giro_x + angulo_ace_x*alpha) ;
  return angulo_complementario - bi_pendulo;
}

float get_bias_gyro_IMU(){
  float bias=0;
  for(int i=0;i<=100;i++){
    unsigned long tiempo_inicial=millis();
    mpu.getEvent(&a, &g, &temp);
    float giro = g.gyro.x;
    bias += giro;
    unsigned long tiempo_final=millis();
    delay(10-(tiempo_final-tiempo_inicial));
  }
  return (bias*(180/pi))/float(100);
}

