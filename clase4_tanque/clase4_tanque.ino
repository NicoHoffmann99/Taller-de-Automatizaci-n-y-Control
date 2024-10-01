
//defino tiempo de sampleo constante
const float T_s = 1.0;
//ctes:
const float k = -4;
const float a = 1;
const float b = 0.00237;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
// put your main code here, to run repeatedly:
//considero recibo un valor de señal de error. 'e'
//valores iniciales de instante anterior error:
static float e_prev_forw = 0;
static float e_prev_tust = 0;
static float e_prev_back = 0;

//valor inicial accion de control instante anterior:
static float u_prev_forw = 0;
static float u_prev_tust = 0;
static float u_prev_back = 0;




//calculos accion instante actual

//forward
u_forw = u_prev_forw + k * e_prev_forw * (b * T_s - a) + k * a * e_forw;

//actualizo el valor previo
e_prev_forw = e_forw;
u_prev_forw = u_forw;

//backward

u_back = u_prev_back - k * a * e_prev_back + k * (a + b * T_s) * e_back;

//actualizo el valor previo
e_prev_back = e_back;
u_prev_back = u_back;

//tustin

u_tust = u_prev_tust + (k * a + (T_s * b)/2) * e_tust + ((T_s * b)/2 - k * a) * e_prev_tust;

//actualizo el valor previo
e_prev_tust = e_tust;
u_prev_tust = u_tust;


}
