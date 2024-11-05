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

A=[0 0 1 0; 0 0 0 1; -53.4 -0.256 171.99 21.85; 0 -245.7 0 -31.22];
B=[0 0 -171.99 245.7];
C=[1 0 0 0; 0 1 0 0];
D = [0 0];

A_d = eye(size(A,1))+A.*Ts;
B_d = B.*Ts;
C_d= C;
D_d = D;

polos_observador_real = real([p1 p2 p_1 p_2]);
polos_observador_imaginario = imag([p1 p2 p_1 p_2])*1i;

factor_polos = 5;
P_Z = exp(factor_polos*polos_observador_real.*Ts);
L = place(A_d',C',P_Z)';

O = (A_d - L*C_d);
ava =eig(O);