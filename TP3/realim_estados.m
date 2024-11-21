%realimentacion por variables de estados
%run matrices_y_observador.m %cargo las matrices de los observadores al workspace

%En vez de usar la salida se usa las estimaciones de los estados. Por eso
%usamos lo hallado con luerenberg (no uso matriz C):

%creo los polos que debemos de luego  buscar ser mas rapidos, salen de la
%ecuacion det(sI-(A-BK))=0 ese K es el que se agrega para llegar a los
%deseados en lazo cerrado por lo tanto sin la K deberia tener los polos "reales"

%polos deseados del script anterior P_Z son 20 veces mas rapido. Podria ver
%de reducir la parte imaginaria para usar polos complejos y asi obtener una
%respuesta mas rapida (¿?) aunque oscilatoria. (me mejora en velocidad, empeora oscil, mejor rise time, Tsettling)
%para verificar:
poles_rr = pole(sys);

%[K,prec] = place(A_d,B_d,P_Z); %me guardo la precisión. no comprendo place
%pq se queja.

%preguntar si hay q usar P_Z o buscar con otros polos deseados.
K = acker(A_d,B_d,P_Z);

avas_realim = eig(A_d - B_d.*K); %deberian estar dentro del circulo unitario


%asi me queda K = -56.6150    4.2427    4.9443    4.0210
% avas =  0.0441   0.0441  0.9747   0.9747 que son los polos P_Z pero nose
% pq ordenado diferente.

%para la matriz de forward se usa la sig ecuacion

F = pinv(C_d * inv(eye(size(A_d)) - (A_d + B_d * K)) * B_d); %es sensible a variaciones de planta y modelo

%con esta puedo seguir referencias, preguntarle a claudio como la aplico en
%mi control. Si en las diapos F == H y resulta U = [K H] [x q]'. Pero va
%para accion integral.