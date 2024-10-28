%ploteo de step.

Q = 1/2;
w = 15.61;
h0 = 1;
s = tf('s');


%pendulo
m_p = out.d3(8580:10200) - out.d3(10121);
N_p = length(m_p);
time_p = linspace(0,N_p*0.01,N_p);


m_b = out.d2(8502:8600) - out.d2(8500);
N = length(m_b);
time = linspace(0,N*0.01,N);


H_1 = (h0*w^2)/(s^2+s*(w/Q)+w^2);
opt = stepDataOptions('StepAmplitude', 30);
[salida,tiempo] = step(H_1,opt);
figure();
hold on;
plot(tiempo,salida);
plot(time,m_b);

p1 =- w/(2*Q) + w*sqrt((1/(2*Q))^2-1);
p2 = - w/(Q*2) - w*sqrt((1/(2*Q))^2-1);
polos_brazo = [p1 p2];

p_1 = -0.13247559170053874+1i*7.302437935150943;
p_2 = -0.13247559170053874-1i*7.302437935150943;
polos_pend = [p_1 p_2];
k = 3/5;
H_2 = zpk([0 0], polos_pend,k);

H_total = H_1 * H_2;
[salida_total,tiempo_total] = step(H_total,opt);

 %saco data y armo sistema.
 [NUM_PEND,DEN_PEND]=tfdata(H_total,'v');
 sys_h = ss(tf(NUM_PEND,DEN_PEND));
 

[salida_2,tiempo_2] = initial(sys_h,[15,0,0,0],time_p);

figure();
hold on;
grid on;
plot(tiempo_total,salida_total);
plot(time_p,m_p);
a = 1;
plot(tiempo_2(a:end),salida_2(a:end),'--');

legend({'estimado','medido'},'Location','southwest');




