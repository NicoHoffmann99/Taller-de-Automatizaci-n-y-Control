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

[num, den] = tfdata(P, 'v');
[A, B, C, D] = tf2ss(num, den)