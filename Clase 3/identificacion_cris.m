% Extraemos las mediciones de simulink
% Extraemos las mediciones de simulink

%exporto a csv
%d3  = out.d3+bias;
bias = 1.9;
writematrix(out.d2(8500:8600), 'd3_mediciones.csv');

% Leer el archivo CSV como una matriz (sin encabezados)
data = readtable('d3_mediciones.csv');

%bias=0.23;
%theta = out.d3(1430:2200)+bias;



% Mostrar los primeros datos
%time = data.Var1(240:end);
theta = data.Var1;

N=length(theta);
time=linspace(0,N*0.01,N);

[max_theta,argmax] = max(theta);

% Calculamos los paramteros del polinomio de la transferencia discreta
Y = theta(3:end);
X = [theta(2:end-1),theta(1:end-2)];
a = pinv(X)*Y;

% Trasnferencia discreta
Td = tf(1,[1 -a(1) -a(2)],time(2)-time(1));

% Polos discretos y continuos
pd = pole(Td);
pc = log(pd)/(time(2) - time(1));

% Armo la forma canonica del controlador de la planta
den = poly(pc); 
A = [0 1; -den(3) -den(2)];
B = [0;1];
C = [1 0];
D = 0;
sys = ss(A,B,C,D);

% Grafico las mediciones y la respuesta teorica a condiciones iniciales
figure(); hold on
plot(time(argmax:end)-time(argmax),theta(argmax:end),'r');
initial(sys,[max_theta;0],time(argmax:end)-time(argmax));
legend({'Mediciones','Respuesta teorica'})

save('pendulo_id','A')