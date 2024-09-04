// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  delay(100);
}

void loop() {

  static unsigned long dif;
  unsigned long t_inicial=micros();
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values 
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  */

  matlab_send(a.acceleration.x,a.acceleration.y,a.acceleration.z,g.gyro.x,g.gyro.y,g.gyro.z,dif);
  //matlab_send(a.acceleration.x,a.acceleration.y,a.acceleration.z,g.gyro.x,g.gyro.y,g.gyro.z,temp.temperature);
  unsigned long t_final= micros();
  dif = t_final - t_inicial;
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