%control planta
close all;
%Parametros
Q = 1/2;
w = 15.61;
h0 = 1;
s = tf('s');

%Polos brazos
p1 =- w/(2*Q) + w*sqrt((1/(2*Q))^2-1);
p2 = - w/(Q*2) - w*sqrt((1/(2*Q))^2-1);
%Polos pendulo
p_1 = -0.13247559170053874+1i*7.302437935150943;
p_2 = -0.13247559170053874-1i*7.302437935150943;

polos=[p1 p2 p_1 p_2];

%Ganancia
k = w^2*0.7;
Ts=0.01;
%Planta total
P=zpk([0 0], polos, -k);

A=[0 0 1 0; 0 0 0 1; -53.4 170.57047 -0.256 21.854; 0 -243.6721 0 -31.22];
polos = eig(A);
B=[0 0 -170.57047 243.6721];
C=[1 0 0 0; 0 1 0 0];
D = [0 0];
sys = ss(A,B',C,0);
t=linspace(0,1000*0.01,1001);
[salida,tiempo] = initial(sys,[15,0,0,0],t);
plot(tiempo,salida);


figure();
sys_d=c2d(sys,Ts,'zoh');
A_d = sys_d.A;
B_d = sys_d.B;
C_d = sys_d.C;
D_d = sys_d.D;

step(20*sys_d,t);

polos_observador = real(polos(1:2))*40 + 1i * imag(polos(1:2)) ;
polos_observador_imaginario = imag(polos(1:2))*1i;
%pendulo = 1, brazp =1.5
factor_polos_pendulo = 10;
factor_polos_brazo = 10;
p_obs_c = [polos_observador(1) polos_observador(2) p1 p2];
%p_obs_c = [-abs(polos(1)) -abs(polos(2)) p1 p2];
p_obs_c(1:2) = p_obs_c(1:2)*factor_polos_pendulo; 
p_obs_c(3:4) = p_obs_c(3:4)*factor_polos_brazo; 
%polos(1:2)=polos(1:2)*factor_polos_pendulo;
%polos(3:4)=polos(3:4)*factor_polos_brazo;
P_Z = exp(p_obs_c.*Ts);
L = place(A_d',C_d',P_Z)';

O = (A_d - L*C_d);
ava =eig(O);

OBS = obsv(A,C);
rango = rank(OBS);

