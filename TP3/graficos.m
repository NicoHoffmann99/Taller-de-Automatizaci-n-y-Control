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




