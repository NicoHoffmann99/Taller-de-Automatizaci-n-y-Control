%Item a - NO LINEAL
figure();
subplot(2,1,1); 
plot(h_no_lineal_item_a.time,h_no_lineal_item_a.signals.values);
title('Variación de la altura de la planta no lineal');
xlabel('Tiempo[s]');
ylabel('Altura[m]');
grid on;
% Crea el segundo gráfico en el subplot inferior
subplot(2,1,2); 
plot(u_no_lineal_item_a.time,u_no_lineal_item_a.signals.values);
title('Variación de la acción de control no lineal');
xlabel('Tiempo[s]');
ylabel('u');
grid on;

%Item a - LINEAL
figure();
subplot(2,1,1); 
plot(h_lineal_item_a.time,h_lineal_item_a.signals.values);
title('Variación de la altura de la planta lineal');
xlabel('Tiempo[s]');
ylabel('Altura[m]');
grid on;
% Crea el segundo gráfico en el subplot inferior
subplot(2,1,2); 
plot(u_lineal_item_a.time,u_lineal_item_a.signals.values);
title('Variación de la acción de control lineal');
xlabel('Tiempo[s]');
ylabel('u');
grid on;

%Item b a 0.35- NO LINEAL
figure();
subplot(2,1,1); 
plot(h_no_lineal_item_b_c.time(1:10000),h_no_lineal_item_b_c.signals.values(1:10000));
title('Respuesta ante un decremento de altura a 0.35m - NO LINEAL');
xlabel('Tiempo[s]');
ylabel('Altura[m]');
grid on;
% Crea el segundo gráfico en el subplot inferior
subplot(2,1,2); 
plot(u_no_lineal_item_b_c.time(1:10000),u_no_lineal_item_b_c.signals.values(1:10000));
title('Accion de control ante un decremento de altura a 0.35m - NO LINEAL');
xlabel('Tiempo[s]');
ylabel('u');
grid on;

%Item b a 0.35- LINEAL
figure();
subplot(2,1,1); 
plot(h_lineal_item_b_c.time(1:10000),h_lineal_item_b_c.signals.values(1:10000));
title('Respuesta ante un decremento de altura a 0.35m - LINEAL');
xlabel('Tiempo[s]');
ylabel('Altura[m]');
grid on;
% Crea el segundo gráfico en el subplot inferior
subplot(2,1,2); 
plot(u_linal_item_b_c.time(1:10000),u_linal_item_b_c.signals.values(1:10000));
title('Accion de control ante un decremento de altura a 0.35m - LINEAL');
xlabel('Tiempo[s]');
ylabel('u');
grid on;


%Item b a 0.55- NO LINEAL
figure();
subplot(2,1,1); 
plot(h_no_lineal_item_b_c.time(20000:34601),h_no_lineal_item_b_c.signals.values(20000:34601));
title('Respuesta ante un incremento de altura a 0.55m - NO LINEAL');
xlabel('Tiempo[s]');
ylabel('Altura[m]');
grid on;
% Crea el segundo gráfico en el subplot inferior
subplot(2,1,2); 
plot(u_no_lineal_item_b_c.time(20000:34601),u_no_lineal_item_b_c.signals.values(20000:34601));
title('Accion de control ante un incremento de altura a 0.55m - NO LINEAL');
xlabel('Tiempo[s]');
ylabel('u');
grid on;

%Item b a 0.55- LINEAL
figure();
subplot(2,1,1); 
plot(h_lineal_item_b_c.time(20000:34601),h_lineal_item_b_c.signals.values(20000:34601));
title('Respuesta ante un incremento de altura a 0.55m - LINEAL');
xlabel('Tiempo[s]');
ylabel('Altura[m]');
grid on;
% Crea el segundo gráfico en el subplot inferior
subplot(2,1,2); 
plot(u_linal_item_b_c.time(20000:34601),u_linal_item_b_c.signals.values(20000:34601));
title('Accion de control ante un incremento de altura a 0.55m - LINEAL');
xlabel('Tiempo[s]');
ylabel('u');
grid on;


%Item c - NO LINEAL
figure();
subplot(2,1,1); 
plot(h_no_lineal_item_b_c.time(34601:end),h_no_lineal_item_b_c.signals.values(34601:end));
title('Respuesta ante variación del caudal de entrada');
xlabel('Tiempo[s]');
ylabel('Altura[m]');
grid on;
% Crea el segundo gráfico en el subplot inferior
subplot(2,1,2); 
plot(u_no_lineal_item_b_c.time(34601:end),u_no_lineal_item_b_c.signals.values(34601:end));
title('Accion de control ante variación del caudal de entrada');
xlabel('Tiempo[s]');
ylabel('u');
grid on;