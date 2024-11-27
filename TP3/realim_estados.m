%realimentacion por variables de estados
Q = 1/2;
w = 15.61;
h0 = 1;
s = tf('s');

coef_a = 20;
coef_a_i = 0.8;
coef_b = 1;

%Polos brazos
p1 =- w/(2*Q) + w*sqrt((1/(2*Q))^2-1) * coef_b ;
p2 = - w/(Q*2) - w*sqrt((1/(2*Q))^2-1) * coef_b;

%Polos pendulo

p_1 = -0.1280 * coef_a + 1i*7.3024*coef_a_i;
p_2 = -0.1280 * coef_a - 1i*7.3024*coef_a_i;

%-5.1200 + 7.3064i
%{
 -31.6266 + 0.0000i
  -1.5919 + 8.1915i
  -1.5919 - 8.1915i
  -3.0436 + 0.0000i
%}

polos=[p1 p2 p_1 p_2];


A=[0 0 1 0; 0 0 0 1; -53.4 170.57047 -0.256 21.854; 0 -243.6721 0 -31.22];
B=[0 0 -170.57047 243.6721];
C=[1 0 0 0; 0 1 0 0];
D = [0 0];
sys = ss(A,B',C,0);
Ts=0.01;
sys_d=c2d(sys,Ts,'zoh');
A_d = sys_d.A;
B_d = sys_d.B;
C_d = sys_d.C;
D_d = sys_d.D;

% polos obs   -7.3075   -7.3075  -15.6100  -15.6100

p_realim = exp(polos*Ts);

K = acker(A_d,-B_d,p_realim);
%K=[0.4266 0.6005 0.0010 0.0030];

%float K[4] = {0.5 ,  0.5 ,  -0.07 ,  -0.07};

avas_realim = eig(A_d + B_d*K);
p_cont = log(avas_realim)/Ts;
F = pinv(C_d * inv(eye(size(A_d)) - (A_d + B_d * K)) * B_d); %es sensible a variaciones de planta y modelo

%Accion integral
C_cont_i = [0 1 0 0];
I = eye(size(C_cont_i, 1));
A_integral = [A_d, zeros(size(A_d, 1), size(I, 1));-C_cont_i*Ts, I];
B_integral = [B_d; 0];
%{
-29.6112 + 0.0000i
  -1.7209 + 8.3459i
  -1.7209 - 8.3459i
  -2.4943 + 0.0000i
float K[4] = {0.5421 ,  0.59551 ,  -0.0700 ,  -0.0620};
%}

polos_integral = [-4.4943 -4.6112 -1.7209-8.3459i -1.7209+8.3459i -7];
polos_integral_discretos = exp(polos_integral*Ts);
H = acker(A_integral, -B_integral, polos_integral_discretos);
ava_integral = eig(A_integral +B_integral * H);


