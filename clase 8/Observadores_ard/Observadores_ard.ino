// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <stdint.h>

Adafruit_MPU6050 mpu;


//EJERCICIO 1
//cargo valores matrices digitales
/*
float Ad[2][2] = {{1.0, 0.01},{-0.5337,0.9974}};
float Cd[2] = {1, 0};
float L[2] = {0.1416, 0.014};
*/

//EJERCICIO 2

float Ad[3][3] = {{1.0, 0.01, 0},{-0.5337,0.9974, 0}, {0, 0, 1}};
float Cd[2][3] = {{1, 0, 0}, {0, 1, 1}};
float L[3][2] = {{0.2271, -0.0012},{0.3008, 0.0218},{-0.8470, 0.2022}};

//La condicion inicial del giroscopio no sería necesaria
float angulo_giro_x=0;
float angulo_ace_x=0;
float angulo_complementario=0;
float velocidad_pendulo = 0;
float angulo_theta = 0;
float bi_pendulo = 0;
float bi_vel = 0;
float bias = 10;
//inicializado de estimadores
float vang_k = 0;
float vang_k1 = 0;
float ang_k = 0;
float ang_k1 = 0;
float bias_k = 0;
float bias_k1 = 0;

//valores para filtro compl
float alpha=0.1;
float Ts=0.01;
float pi=3.1415;

sensors_event_t a, g, temp;

void setup(void) {
  Serial.begin(115200);
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
  static unsigned long dif;
  unsigned long t_inicial=micros();
  angulo_theta = get_angulo_IMU();
  //Serial.println(angulo_theta);
  //velocidad
  velocidad_pendulo = g.gyro.x*(180/pi) - bi_vel ;
  //Serial.println(velocidad_pendulo);


  //rad/s
  float ang_comp = angulo_theta;  
  float vel_ang  = velocidad_pendulo;

  //EJERCICIO 1
  /*
  ang_k1 = Ad[0][0] * ang_k + Ad[0][1] * vang_k + L[0] * ( angulo_theta- ang_k); //sin Bd pq no controlamos todavia
  vang_k1 = Ad[1][0] * ang_k + Ad[1][1] * vang_k + L[1] * (angulo_theta - ang_k);
  */

  //EJERCICIO 2
  
  ang_k1 = Ad[0][0] * ang_k + Ad[0][1] * vang_k + Ad[0][2] * bias_k + L[0][0] * (angulo_theta - ang_k) + L[0][1] * (velocidad_pendulo + bias - vang_k - bias_k);
  vang_k1 = Ad[1][0] * ang_k + Ad[1][1] * vang_k + Ad[1][2] * bias_k + L[1][0] * (angulo_theta - ang_k) + L[1][1] * (velocidad_pendulo + bias - vang_k - bias_k);
  bias_k1 = Ad[2][0] * ang_k + Ad[2][1] * vang_k + Ad[2][2] * bias_k + L[2][0] * (angulo_theta - ang_k) + L[2][1] * (velocidad_pendulo + bias - vang_k - bias_k);
  

  //actualizacion estados
  ang_k = ang_k1;
  //Serial.println(ang_k);
  vang_k = vang_k1;
  //Serial.println(vang_k);
  bias_k = bias_k1;
  //Serial.println(bias_k);

  //traduzco a rad/s
  float angulo_k = ang_k ;
  float vel_ang_k  = vang_k ;
  float bias_grad_k = bias_k ;


  matlab_send(ang_comp,angulo_k,vel_ang,vel_ang_k,bias,bias_grad_k,0);

  
  //Temperatura: temp.temperature
  unsigned long t_final= micros();
  //dif = t_final - t_inicial;
  //Serial.print(10000-dif);
  delayMicroseconds(10000-(t_final-t_inicial));

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


float get_angulo_IMU(){
  //sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  angulo_giro_x= (angulo_complementario + (Ts*g.gyro.x)*(180/pi));
  angulo_ace_x= atan2(a.acceleration.y,a.acceleration.z)*(180/pi);
  angulo_complementario= ((1-alpha)*angulo_giro_x + angulo_ace_x*alpha) ;
  return angulo_complementario - bi_pendulo;
}
