#include "TimerOne.h"


//ctes:
const float k = -4;
const float a = 1;
const float b = 0.00237;

typedef union{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

void setup()
{
  Serial.begin(115200);
  //valores iniciales de instante anterior error:

}

void loop()
{
  // Ajustar condiciones iniciales de trabajo
  static float u_0=0.5, h_ref=0.4, h=0.45, u;
  static float Ts=1;
  FLOATUNION_t aux;
  static float sampling_period_ms = 1000*Ts;
  //=========================
  // Definir parametros y variables del control

  //=========================

  if (Serial.available() >= 4) {
 
    aux.number = getFloat();
    h = aux.number;
    //aux.number = getFloat();
  }
  
  //=========================
  //CONTROL
  static float e_prev_forw = 0;
  static float e_prev_tust = 0;
  static float e_prev_back = 0;

  //valor inicial accion de control instante anterior:
  static float u_prev_forw = 0;
  static float u_prev_tust = 0;
  static float u_prev_back = 0;
  //float e_back = (h_ref - h);
  float e_forw = (h_ref - h);

  /*back
  u = u_prev_back - k * a * e_prev_back + k * (a + b * Ts) * e_back ;

  //actualizo el valor previo
  e_prev_back = e_back;
  u_prev_back = u;
  */
  //u = u_prev_forw + k * e_prev_forw * (b * Ts - a) + k * a * e_forw;
  float val_act = e_forw * k + (b*Ts-1) * e_prev_forw * k + u_prev_forw;
  u=val_act +u_0;
  //u = u_prev_forw + e_forw * k - e_prev_forw* k + k * b * Ts

  //actualizo el valor previo
  e_prev_forw = e_forw;
  u_prev_forw = val_act;
  
  //=========================
    
  matlab_send(u);
  delay(sampling_period_ms);
}

void matlab_send(float u){
  Serial.write("abcd");
  byte * b = (byte *) &u;
  Serial.write(b,4);
 
}

float getFloat(){
    int cont = 0;
    FLOATUNION_t f;
    while (cont < 4 ){
        f.bytes[cont] = Serial.read() ;
        cont = cont +1;
    }
    return f.number;
}
