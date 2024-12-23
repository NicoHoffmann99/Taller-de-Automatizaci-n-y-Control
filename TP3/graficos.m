phi = out.phi;
phi_obs = out.phi_obs;
theta = out.theta_real;
theta_obs = out.theta_obs;
vel_phi = out.vel_phi;
vel_phi_obs = out.vel_phi_obs;
vel_theta = out.vel_theta;
vel_theta_obs = out.vel_theta_obs;
tiempo = out.tout;

% Crear una figura
figure();
% Primer subgráfico con varias curvas
subplot(4, 1, 1); 
plot(tiempo, phi, 'r', 'LineWidth', 1.5); 
hold on; 
plot(tiempo, phi_obs, 'b', 'LineWidth', 1); 
hold off; 
title('Angulo del brazo $\phi$ real vs observado', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
ylabel('$\phi$ [grados]', 'Interpreter', 'latex');
legend('$\phi$ real', '$\phi$ observado', 'Interpreter', 'latex');
% Segundo subgráfico con una curva
subplot(4, 1, 2); 
plot(tiempo, theta, 'r', 'LineWidth', 1.5); 
hold on; 
plot(tiempo, theta_obs, 'b', 'LineWidth', 1); 
hold off; 
title('Angulo del pendulo $\theta$ real vs observado', 'Interpreter', 'latex');
ylabel('$\theta [grados]$', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
legend('$\theta$ real', '$\theta$ observado', 'Interpreter', 'latex');
% Tercer subgráfico
subplot(4, 1, 3); 
plot(tiempo, vel_phi, 'r', 'LineWidth', 1.5);
hold on; 
plot(tiempo, vel_phi_obs, 'b', 'LineWidth', 1); 
hold off; 
title('Velocidad angular del brazo $\dot{\phi}$ real vs observado', 'Interpreter', 'latex');
ylabel('$\dot{\phi}$ [grados/s]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
legend('$\dot{\phi}$ real', '$\dot{\phi}$ observado', 'Interpreter', 'latex');
% Cuarto subgráfico
subplot(4, 1, 4); 
plot(tiempo, vel_theta, 'r', 'LineWidth', 1.5); 
hold on; 
plot(tiempo, vel_theta_obs, 'b', 'LineWidth', 1); 
hold off;
ylabel('$\dot{\theta}$ [grados/s]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
legend('$\dot{\theta}$ real', '$\dot{\theta}$ observado', 'Interpreter', 'latex');
title('Velocidad angular del pendulo $\dot{\theta}$ real vs observado', 'Interpreter', 'latex');

%15 grados
phi = out.phi(1950:3950);
phi_obs = out.phi_obs(1950:3950);
theta = out.theta_real(1950:3950);
theta_obs = out.theta_obs(1950:3950);
vel_phi = out.vel_phi(1950:3950);
vel_phi_obs = out.vel_phi_obs(1950:3950);
vel_theta = out.vel_theta(1950:3950);
vel_theta_obs = out.vel_theta_obs(1950:3950);
tiempo = out.tout(1950:3950);


% Crear una figura
figure();
% Primer subgráfico con varias curvas
subplot(4, 1, 1); 
plot(tiempo, phi, 'r', 'LineWidth', 1.5); 
hold on; 
plot(tiempo, phi_obs, 'b', 'LineWidth', 1); 
hold off; 
title('Angulo del brazo $\phi$ para un step de 15', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
ylabel('$\phi$ [grados]', 'Interpreter', 'latex');
legend('$\phi$ real', '$\phi$ observado', 'Interpreter', 'latex');
xlim([tiempo(1) tiempo(2000)]);
% Segundo subgráfico con una curva
subplot(4, 1, 2); 
plot(tiempo, theta, 'r', 'LineWidth', 1.5); 
hold on; 
plot(tiempo, theta_obs, 'b', 'LineWidth', 1); 
hold off; 
title('Angulo del pendulo $\theta$ para un step de 15', 'Interpreter', 'latex');
ylabel('$\theta [grados]$', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
legend('$\theta$ real', '$\theta$ observado', 'Interpreter', 'latex');
xlim([tiempo(1) tiempo(2000)]);
% Tercer subgráfico
subplot(4, 1, 3); 
plot(tiempo, vel_phi, 'r', 'LineWidth', 1.5);
hold on; 
plot(tiempo, vel_phi_obs, 'b', 'LineWidth', 1);
hold off; 
title('Velocidad angular del brazo $\dot{\phi}$ para un step de 15', 'Interpreter', 'latex');
ylabel('$\dot{\phi}$ [grados/s]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
legend('$\dot{\phi}$ real', '$\dot{\phi}$ observado', 'Interpreter', 'latex');
xlim([tiempo(1) tiempo(2000)]);
% Cuarto subgráfico
subplot(4, 1, 4); 
plot(tiempo, vel_theta, 'r', 'LineWidth', 1.5); 
hold on; 
plot(tiempo, vel_theta_obs, 'b', 'LineWidth', 1); 
hold off;
ylabel('$\dot{\theta}$ [grados/s]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
legend('$\dot{\theta}$ real', '$\dot{\theta}$ observado', 'Interpreter', 'latex');
title('Velocidad angular del pendulo $\dot{\theta}$ para un step de 15', 'Interpreter', 'latex');
xlim([tiempo(1) tiempo(2000)]);




%15 grados
phi = out.phi(3980:6280);
phi_obs = out.phi_obs(3980:6280);
theta = out.theta_real(3980:6280);
theta_obs = out.theta_obs(3980:6280);
vel_phi = out.vel_phi(3980:6280);
vel_phi_obs = out.vel_phi_obs(3980:6280);
vel_theta = out.vel_theta(3980:6280);
vel_theta_obs = out.vel_theta_obs(3980:6280);
tiempo = out.tout(3980:6280);


% Crear una figura
figure();
% Primer subgráfico con varias curvas
subplot(4, 1, 1); 
plot(tiempo, phi, 'r', 'LineWidth', 1.5); 
hold on; 
plot(tiempo, phi_obs, 'b', 'LineWidth', 1); 
hold off; 
title('Angulo del brazo $\phi$ para un step de -10', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
ylabel('$\phi$ [grados]', 'Interpreter', 'latex');
legend('$\phi$ real', '$\phi$ observado', 'Interpreter', 'latex');
xlim([tiempo(1) tiempo(2000)]);
% Segundo subgráfico con una curva
subplot(4, 1, 2); 
plot(tiempo, theta, 'r', 'LineWidth', 1.5); 
hold on; 
plot(tiempo, theta_obs, 'b', 'LineWidth', 1); 
hold off; 
title('Angulo del pendulo $\theta$ para un step de -10', 'Interpreter', 'latex');
ylabel('$\theta [grados]$', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
legend('$\theta$ real', '$\theta$ observado', 'Interpreter', 'latex');
xlim([tiempo(1) tiempo(2000)]);
% Tercer subgráfico
subplot(4, 1, 3); 
plot(tiempo, vel_phi, 'r', 'LineWidth', 1.5);
hold on; 
plot(tiempo, vel_phi_obs, 'b', 'LineWidth', 1);
hold off; 
title('Velocidad angular del brazo $\dot{\phi}$ para un step de -10', 'Interpreter', 'latex');
ylabel('$\dot{\phi}$ [grados/s]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
legend('$\dot{\phi}$ real', '$\dot{\phi}$ observado', 'Interpreter', 'latex');
xlim([tiempo(1) tiempo(2000)]);
% Cuarto subgráfico
subplot(4, 1, 4); 
plot(tiempo, vel_theta, 'r', 'LineWidth', 1.5); 
hold on; 
plot(tiempo, vel_theta_obs, 'b', 'LineWidth', 1); 
hold off;
ylabel('$\dot{\theta}$ [grados/s]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
legend('$\dot{\theta}$ real', '$\dot{\theta}$ observado', 'Interpreter', 'latex');
title('Velocidad angular del pendulo $\dot{\theta}$ para un step de -10', 'Interpreter', 'latex');
xlim([tiempo(1) tiempo(2000)]);


%% SIMULACIONES


%importar datos simulink
t_inicial = ;
t_final = ;
phi = out.phi(t_inicial:t_final);
phi_obs = out.phi_obs(t_inicial:t_final);
theta = out.theta_real(t_inicial:t_final);
theta_obs = out.theta_obs(t_inicial:t_final);
vel_phi = out.vel_phi(t_inicial:t_final);
vel_phi_obs = out.vel_phi_obs(t_inicial:t_final);
vel_theta = out.vel_theta(t_inicial:t_final);
vel_theta_obs = out.vel_theta_obs(t_inicial:t_final);
tiempo = out.tout(t_inicial:t_final);



%Simulaciones por realimentacion de estado
Ts=0.01;

A=[0 0 1 0; 0 0 0 1; -53.4 170.57047 -0.256 21.854; 0 -243.6721 0 -31.22];
polos = eig(A);
B=[0 0 -170.57047 243.6721];
C=[1 0 0 0; 0 1 0 0];
D = [0 0];

sys = ss(A,B',C,0);
sys_d=c2d(sys,Ts,'zoh');
A_d = sys_d.A;
B_d = sys_d.B;
C_d = sys_d.C;
D_d = sys_d.D;

p_realim = [-8.6112 + 0.0000i -1.509 + 7.3459i -1.509 - 7.3459i -8.4943 + 0.0000i];
%float K[4] = {0.2217 ,  0.6697 ,  -0.0078 ,  0.0415};
%p_realim = [-10.6112 + 0.0000i -1.909 + 7.3459i -1.909 - 7.3459i -10.4943 + 0.0000i];
p_realim_disc = exp(p_realim.*Ts);
K = acker(A_d,-B_d,p_realim_disc);
K = [0.65, 0.5, -0.07, -0.06];
A_realim_est = A_d + B_d*K;
sys_impulso = ss(A_realim_est,B_d,eye(4),0,Ts);
t=linspace(0,500*0.01,501);
%initial con un impulso a la velocidad theta
vel_theta = -150;
[salida_1, tiempo_1] = initial(sys_impulso(1),[0,0,vel_theta,0],t);
[salida_2, tiempo_2] = initial(sys_impulso(2),[0,0,vel_theta,0],t);
[salida_3, tiempo_3] = initial(sys_impulso(3),[0,0,vel_theta,0],t);
[salida_4, tiempo_4] = initial(sys_impulso(4),[0,0,vel_theta,0],t);

figure();
subplot(2, 2, 1);
plot(tiempo_1,salida_1,'Color', [0.1, 0.2, 0.5],'LineWidth', 1.5);
grid on;
title('Angulo $\theta$','Color', [0.1, 0.2, 0.5], 'Interpreter', 'latex');
ylabel('$\theta$ [grados]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
xlim([0,5]);
subplot(2, 2, 2);
plot(tiempo_2,salida_2,'Color', [0.1, 0.2, 0.5], 'LineWidth', 1.5);
title('Angulo $\phi$', 'Interpreter', 'latex');
ylabel('$\phi$ [grados]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
xlim([0,5]);
grid on;
subplot(2, 2, 3);
plot(tiempo_3,salida_3,'Color', [0.1, 0.2, 0.5], 'LineWidth', 1.5);
title('Velocidad angular $\dot{\theta}$', 'Interpreter', 'latex');
ylabel('$\dot{\theta}$ [grados/s]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
xlim([0,5]);
grid on;
subplot(2, 2, 4);
plot(tiempo_4,salida_4,'Color', [0.1, 0.2, 0.5] , 'LineWidth', 1.5 );
title('Velocidad angular $\dot{\phi}$', 'Interpreter', 'latex');
ylabel('$\dot{\phi}$ [grados/s]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
xlim([0,5]);
grid on;

sgtitle('Respuesta al impulso - Realimentacion por variables de estado', 'Interpreter', 'latex');

%Simulación sumando la matriz de feedfordward
F = pinv(C_d * inv(eye(size(A_d)) - (A_d + B_d * K)) * B_d);
sys_feedfordward = ss(A_realim_est,B_d*F(2),eye(4),0,Ts);

[salida_1, tiempo_1] = step(20*sys_feedfordward(1),t);
[salida_2, tiempo_2] = step(20*sys_feedfordward(2),t);
[salida_3, tiempo_3] = step(20*sys_feedfordward(3),t);
[salida_4, tiempo_4] = step(20*sys_feedfordward(4),t);

figure();
subplot(2, 2, 1);
plot(tiempo_1,salida_1,'Color', [0.1, 0.2, 0.5],'LineWidth', 1.5);
grid on;
title('Angulo $\theta$','Color', [0.1, 0.2, 0.5], 'Interpreter', 'latex');
ylabel('$\theta$ [grados]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
xlim([0,5]);
subplot(2, 2, 2);
plot(tiempo_2,salida_2,'Color', [0.1, 0.2, 0.5], 'LineWidth', 1.5);
title('Angulo $\phi$', 'Interpreter', 'latex');
ylabel('$\phi$ [grados]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
xlim([0,5]);
grid on;
subplot(2, 2, 3);
plot(tiempo_3,salida_3,'Color', [0.1, 0.2, 0.5], 'LineWidth', 1.5);
title('Velocidad angular $\dot{\theta}$', 'Interpreter', 'latex');
ylabel('$\dot{\theta}$ [grados/s]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
xlim([0,5]);
grid on;
subplot(2, 2, 4);
plot(tiempo_4,salida_4,'Color', [0.1, 0.2, 0.5] , 'LineWidth', 1.5 );
title('Velocidad angular $\dot{\phi}$', 'Interpreter', 'latex');
ylabel('$\dot{\phi}$ [grados/s]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
xlim([0,5]);
grid on;

sgtitle('Respuesta al escalon con $\phi_{ref}=20^{\circ}$ - Feedfordward', 'Interpreter', 'latex');

%Simulacion de acción integral
C_cont_i = [0 1 0 0];
I = eye(size(C_cont_i, 1));
A_integral = [A_d, zeros(size(A_d, 1), size(I, 1));-C_cont_i*Ts, I];
B_integral = [B_d; 0];

polos_integral = [-23.7862 + 0.0000i -2.2154 + 8.0974i -2.2154 - 8.0974i -5.4994 + 0.0000i -1];
%polos_integral = [-12.7862 + 0.0000i -2.2154 + 8.0974i -11.2154 - 8.0974i -3.4994 + 0.0000i -2];
polos_integral_discretos = exp(polos_integral*Ts);
H = acker(A_integral, -B_integral, polos_integral_discretos);

A_integral_simulacion = A_integral + B_integral*H;
sys_acc_integral = ss(A_integral_simulacion,[B_integral [0; 0; 0; 0; 1]*Ts],[1 0 0 0 0;0 1 0 0 0;0 0 1 0 0; 0 0 0 1 0], 0, Ts); 

%Respuesta al escalón con acción integral
[salida_1, tiempo_1] = step(20*sys_acc_integral(1,1),t);
[salida_2, tiempo_2] = step(20*sys_acc_integral(2,2),t);
[salida_3, tiempo_3] = step(20*sys_acc_integral(3,1),t);
[salida_4, tiempo_4] = step(20*sys_acc_integral(4,2),t);

figure();
subplot(2, 2, 1);
plot(tiempo_1,salida_1,'Color', [0.1, 0.2, 0.5],'LineWidth', 1.5);
grid on;
title('Angulo $\theta$', 'Interpreter', 'latex');
ylabel('$\theta$ [grados]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
xlim([0,5]);
subplot(2, 2, 2);
plot(tiempo_2,salida_2,'Color', [0.1, 0.2, 0.5], 'LineWidth', 1.5);
title('Angulo $\phi$', 'Interpreter', 'latex');
ylabel('$\phi$ [grados]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
xlim([0,5]);
grid on;
subplot(2, 2, 3);
plot(tiempo_3,salida_3,'Color', [0.1, 0.2, 0.5], 'LineWidth', 1.5);
title('Velocidad angular $\dot{\theta}$', 'Interpreter', 'latex');
ylabel('$\dot{\theta}$ [grados/s]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
xlim([0,5]);
grid on;
subplot(2, 2, 4);
plot(tiempo_4,salida_4,'Color', [0.1, 0.2, 0.5] , 'LineWidth', 1.5 );
title('Velocidad angular $\dot{\phi}$', 'Interpreter', 'latex');
ylabel('$\dot{\phi}$ [grados/s]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
xlim([0,5]);
grid on;

sgtitle('Respuesta al escalon con $\phi_{ref}=20^{\circ}$ - Accion Integral', 'Interpreter', 'latex');


[salida_1, tiempo_1] = initial(sys_acc_integral(1,1),[0,0,vel_theta,0, 0],t);
[salida_2, tiempo_2] = initial(sys_acc_integral(2,1),[0,0,vel_theta,0, 0],t);
[salida_3, tiempo_3] = initial(sys_acc_integral(3,1),[0,0,vel_theta,0, 0],t);
[salida_4, tiempo_4] = initial(sys_acc_integral(4,1),[0,0,vel_theta,0, 0],t);

figure();

subplot(2, 2, 1);
plot(tiempo_1,salida_1,'Color',[0.1, 0.2, 0.5],'LineWidth', 1.5);
grid on;
title('Angulo $\theta$', 'Interpreter', 'latex');
ylabel('$\theta$ [grados]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
xlim([0,5]);
subplot(2, 2, 2);
plot(tiempo_2,salida_2,'Color', [0.1, 0.2, 0.5], 'LineWidth', 1.5);
title('Angulo $\phi$', 'Interpreter', 'latex');
ylabel('$\phi$ [grados]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
xlim([0,5]);
grid on;
subplot(2, 2, 3);
plot(tiempo_3,salida_3,'Color', [0.1, 0.2, 0.5], 'LineWidth', 1.5);
title('Velocidad angular $\dot{\theta}$', 'Interpreter', 'latex');
ylabel('$\dot{\theta}$ [grados/s]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
xlim([0,5]);
grid on;
subplot(2, 2, 4);
plot(tiempo_4,salida_4,'Color', [0.1, 0.2, 0.5] , 'LineWidth', 1.5 );
title('Velocidad angular $\dot{\phi}$', 'Interpreter', 'latex');
ylabel('$\dot{\phi}$ [grados/s]', 'Interpreter', 'latex');
xlabel('tiempo [s]', 'Interpreter', 'latex');
xlim([0,5]);
grid on;

sgtitle('Respuesta a una perturbación tipo impulso - Accion Integral', 'Interpreter', 'latex');






