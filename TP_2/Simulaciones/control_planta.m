%control planta

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
k = w^2*3/5;

%Planta total
P=zpk([0 0], polos, -k);

figure();
bode(P);
title('Planta')
grid on;

Kp=0.1;
C=-Kp;
L = C*P;
figure();
bode(L);
title('L con Proporcional')
grid on;

%integrador

Kp = 0.07;
Ki = 1;
C_i = -(Kp+Ki/s);
L_i = C_i*P;

figure();

bode(L_i);
title('L con Integrador');
grid on;

%proporcional derivativo

Kp = 0.09;
Kd = 0.003;%bajo pq deriva el ruido q es algo muy oscilante/de picos abruptos
C_d = - (Kp + Kd*s); 
L_d = C_d * P;
figure();
bode(L_d);
title('L con derivador');
grid on;

