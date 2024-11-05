// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <stdint.h>

Adafruit_MPU6050 mpu;


//cargo valores matrices digitales
//X = [tita , phi , tita_p , phi_p]^T
float Ad[4][4] = {{1.0000,0,0.0100,0},{0.0,1.0,0,0.01},{-0.5340,-0.0026,2.7199,0.2185},{0,-2.4570,0,0.6878}}; //col 1 tita, col 2 phi, col 3 tita_p, col 4 phi_p (estos son pesos que multiplican a los estimados)
float Cd[2][4] = {{1, 0,0,0},{0,1,0,0}}; // aca fila 1 es para tita (angulo pendulo) y la fila 2 es para phi (angulo brazo)
float L[2][4] = {{1.8658,0,320.3755,0},{0.2185, -0.1663, 33.9425, 2.7556}}; //ver que onda con el valor grande

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

//pin lectura pote
int sensor=14;
int pote_50=606; //pwm 117
int pote_menos_50=203; //pwm 218

//pruebo con notacion vectorial
float tita = 0;float phi = 0;float tita_p = 0;float phi_p = 0;
float x_k[4] = {tita,phi,tita_p,phi_p}; 
float x_k1[4] = {tita,phi,tita_p,phi_p};
float y_m[4] = {tita,phi,tita_p,phi_p}; 

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

  //puedo inferir la velocidad del brazo?


  //lectura imu angulo
  angulo_giro_x= angulo_complementario + periodo*g.gyro.x;
  angulo_ace_x= atan2(a.acceleration.y,a.acceleration.z);
  angulo_complementario= (1-alpha)*angulo_giro_x + angulo_ace_x*alpha;

  //lectura imu vel
   velocidad_pendulo = g.gyro.x ;

  //rad/s
  float ang_comp = (180*angulo_complementario)/pi;  
  float vel_ang  = velocidad_pendulo * 180 / pi;


  //cargo valores actuales, medidios
  y_m[0] = (180*angulo_complementario)/pi; //cargo tita 
  y_m[2] = velocidad_pendulo * 180 / pi; // cargo tita_p

  //lectura pote y mapeo a angulo phi
  float pos_pote = analogRead(sensor);
  y_m[1] =map(pos_pote,pote_menos_50,pote_50,-50,50);//basicamente queremos nuestro angulo limite de -50 a 50 y vimos la correspondencia con el valor q indica el pote ya armado en el pendulo.
  //nose si se puede inferir phi_p de alguna forma



  //la estimacion con luenberg es x_k+1 = Ad * Xk + L * (y-yk) sin usar matriz B de control
  //en C no existen las matrices sino arreglos de vectores, para hacer multiplicacion matricial debo recorrer con un for anidado
  for(int i = 0;i<2;i++){
      x_k1[i] = Ad[i][i] * x_k[i]  + Ad[i][i+2] * x_k[i+2] + L[i][i] * ( y_m[i] - x_k[i]); //estimacion variable de estado 'a'
      x_k1[i+2] = Ad[i+2][i] * x_k[i]  + Ad[i+2][i+2] * x_k[i] + L[i][i+2] * ( y_m[i] - x_k[i]); //estimacion variable de estado 'a_punto'
    //chequear bien lo de L[i][i+2] a cual corresponde
  }
  /*ejemplo para tita y phi
  tita x1[0] = Ad[0][0] * x[0] (angulo tita est) + Ad[0][2] * x[2] + L[0][0] * (y[0]-x[0])
  tita_p x1[2] = Ad[2][0] * x[0] (angulo tita est) + Ad[2][2] * x[2] + L[0][2] * (y[0]-x[0])
  phi x1[1] = Ad[1][1] * x[1] (angulo phi) + Ad[1][3] * x[3] + L[1][1] * (y[1]-x[1])
  phi_p x1[3] = Ad[3][1] * x[1] (angulo phi) + Ad[3][3] * x[3] + L[1][3] * (y[1]-x[1])

  creo asi seria todo a manopla, nose como corroborar los indices de L que corresponden a cada valor
  
  */

  


  //actualizacion estados
  x_k[0] = x_k1[0];x_k[1] = x_k1[1];x_k[2] = x_k1[2];x_k[3] = x_k1[3];

  //traduzco a rad/s si quiero ver las salidas y sus derivadas estimadas (defino las variables internamente, no necesito declararlas antes)
  float tita_k = x_k[0] * 180/pi;
  float tita_kp  = x_k[2] * 180 / pi;
  float phi_k = x_k[1] * 180/pi;
  float phi_kp  = x_k[3] * 180 / pi;


  matlab_send(y_m[0],tita_k,y_m[1],phi_k,y_m[2],tita_kp,0); //esto despues ver de acomodar segun lo q necesitemos

  
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

