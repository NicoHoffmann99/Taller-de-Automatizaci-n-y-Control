// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <stdint.h>

Adafruit_MPU6050 mpu;


//cargo valores matrices digitales

float Ad[2][2] = {{1.0, 0.01},{-0.5337,0.9974}};

float Cd[2] = {1, 0};
float L[2] = {0.1416, 0.014};

//La condicion inicial del giroscopio no sería necesaria
float angulo_giro_x=0;
float angulo_ace_x=0;
float angulo_complementario=0;
float velocidad_pendulo = 0;

//inicializado de estimadores
float vang_k = 0;
float vang_k1 = 0;
float ang_k = 0;
float ang_k1 = 0;

//valores para filtro compl
float alpha=0.1;
float periodo=0.01;
float pi=3.1415;

void setup(void) {
  Serial.begin(115200);
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  delay(100);
}

void loop() {
  //Tiempo de lectura de datos + transformacion
  static unsigned long dif;
  unsigned long t_inicial=micros();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  //angulo
  angulo_giro_x= angulo_complementario + periodo*g.gyro.x;
  angulo_ace_x= atan2(a.acceleration.y,a.acceleration.z);
  angulo_complementario= (1-alpha)*angulo_giro_x + angulo_ace_x*alpha;

  //velocidad
   velocidad_pendulo = g.gyro.x ;

  //rad/s
  float ang_comp = (180*angulo_complementario)/pi;  
  float vel_ang  = velocidad_pendulo * 180 / pi;

  ang_k1 = Ad[0][0] * ang_k + Ad[0][1] * vang_k + L[0] * ( angulo_complementario- ang_k); //sin Bd pq no controlamos todavia
  vang_k1 = Ad[1][0] * ang_k + Ad[1][1] * vang_k + L[1] * (angulo_complementario - ang_k);

  


  //actualizacion estados
  ang_k = ang_k1;
  vang_k = vang_k1;

  //traduzco a rad/s
  float angulo_k = ang_k * 180/pi;
  float vel_ang_k  = vang_k * 180 / pi;


  matlab_send(ang_comp,angulo_k,vel_ang,vel_ang_k,0,0,0);

  
  //Temperatura: temp.temperature
  unsigned long t_final= micros();
  dif = t_final - t_inicial;
  Serial.print(10000-dif);
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

