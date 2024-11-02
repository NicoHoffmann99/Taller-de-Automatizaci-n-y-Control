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
figure();
bode(P);
title('Bode - Planta')
grid on;

%Rlocus
figure();
rlocus(-P);
title('Diseño de Controlador P - Root Locus de la Planta');

%Retraso de media muestra
D=exp(-Ts*s/2);
Dp=(1-(Ts/4)*s)/(1+(Ts/4)*s);



%Controlador Proporcional
Kp=0.3;
C=-Kp;
L = C*P*D;
%{
figure();
bode(L);
title('L con Proporcional')
grid on;
%}

delta = 100; 
% Crear la señal de impulso
t = 0:0.01:10;
impulso = zeros(size(t)); % Inicializar el vector con ceros
impulso(70:70+10) = delta; % Asignar el valor del impulso en t = 0
% Graficar la señal de impulso

figure();
theta_real_p = out.d3(3325:4250);
N_p = length(theta_real_p);
t2 = linspace(0,N_p*0.01,N_p);
theta_sim_p = lsim(L/(L+1), impulso, t);
plot(t, theta_sim_p); hold on;
plot(t2, theta_real_p);
legend('Real','Simulado');
title('Respuesta al impulso para controlador P');
xlabel('tiempo [s]');
ylabel('theta [grados]');
grid on;

%integrador
Kp = 0.1;
Ki = 3;
C_i = -(Kp+Ki/s);
L_i = C_i*P*D;

%{
figure();
bode(L_i);
title('L con Integrador');
grid on;
%}
t = 0:0.01:5;

T_i = L_i/(1+L_i);
r_sim = 5 * heaviside(t);
theta_sim_p = lsim(T_i, r_sim, t);
figure();
plot(t, theta_sim_p);
title('Salida ante respuesta al escalón con PI');
grid on;
xlabel('tiempo [s]');
ylabel('theta [grados]');


CS_i = C_i/(1+L_i);
r_sim = 5 * heaviside(t);
u_sim_pi = lsim(CS_i, r_sim, t);
figure(); 
plot(t, u_sim_pi);
title('Acción de control ante respuesta al escalón con PI');
grid on;
xlabel('tiempo [s]');
ylabel('u [grados]');


%proporcional derivativo

Kp = 0.3;
Kd = 0.0005;%bajo pq deriva el ruido q es algo muy oscilante/de picos abruptos
C_d = - (Kp + Kd*s); 
L_d = C_d * P * D;
%{
figure();
bode(L_d);
title('L con derivador');
grid on;
%}

delta = 300; 
% Crear la señal de impulso
t = 0:0.01:10;
impulso = zeros(size(t)); % Inicializar el vector con ceros
impulso(100:100+5) = delta; % Asignar el valor del impulso en t = 0
% Graficar la señal de impulso

figure();
theta_real_d = out.d3(730:1600);
N_d = length(theta_real_d);
t3 = linspace(0,N_d*0.01,N_d);
theta_sim_p = lsim(L_d/(L_d+1), impulso, t);
plot(t3,theta_real_d);hold on;
plot(t, theta_sim_p);
legend('Real','Simulado');
title('Respuesta al impulso para controlador PD');
xlabel('tiempo [s]');
ylabel('theta [grados]');
grid on;

