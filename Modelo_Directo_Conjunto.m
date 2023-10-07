

%%%%%%% MODELO DIRECTO DEL CONJUNTO %%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%CAMBIANDO LOS VALORES DE LOS GRADOS DE LIBERTAD DE CADA ARTICULACION
%OBTENEMOS EL VECTOR POSICIÓN DE CADA PIE EN EL SISTEMA MUNDO, QUE SERÁ EL
%FIJO, ASÍ COMO EL VECTOR POSICION DE LA BASE DE LA PINZA DEL BRAZO. EL SISTEMA B Y M ESTÁN RELACIONADOS MEDIANTE RB Y vB_M QUE HASTA NUEVA
%ORDEN CONSIDERAMOS, RESPECTIVAMENTE, IDENTIDAD Y 0. EL SISTEMA B2 ES UN
%SISTEMA EN EL CENTRO DE UN CUADRILÁTERO FORMADO POR LOS 4 PIES, ESTE
%SISTEMA GIRA CON EL A1.


%Grados de libertad.
q1_L1_i = 0; q2_L1_i = 0; q3_L1_i = 0; %Legs iniciales
q1_L2_i = 0; q2_L2_i = 0; q3_L2_i = 0;
q1_L3_i = 0; q2_L3_i = 0; q3_L3_i = 0;
q1_L4_i = 0; q2_L4_i = 0; q3_L4_i = 0;
q1_i = 0; q2_i = 0; q3_i = 0; q4_i = 0; q5_i = 0; q6_i = 0; %Los del brazo iniciales

q1_L1 = 0; q2_L1 = 0; q3_L1 = 0; %Leg1 = trasera derecha
q1_L2 = 0; q2_L2 = 0; q3_L2 = 0; %Leg2 = delantera derecha
q1_L3 = 0; q2_L3 = 0; q3_L3 = 0; %Leg3 = trasera izquierda
q1_L4 = 0; q2_L4 = 0; q3_L4 = 0; %Leg4 = delantera izquierda
q1 = 0; q2 = 0; q3 = 0; q4 = 0; q5 = 0; q6 = 0; %Los del brazo

%Situación inicial del sistema B = centro lomo A1 con sistema M = Mundo
h = 415; %Altura de la base B respecto de las patas. Lo considero constante. h = 415 => estiradas completamente 
RB_inicial = eye(3); %Rotacion inicial del A1 respecto de M. Si se cambia hay que incluirlo en la entrada de la funcion modelodirectopierna R_inicial*R. Por comodidad supondré que siempre comenzará con la misma orientación de ejes que M.
vB_M_inicial = [0 h 0]'; %El A1 está sobre el sistema mundo, el cual está en el suelo. y=altura de B=h.


%Las longitudes de los links brazos no los ponemos porque: El modelo del
%brazo se ha hecho con D-H y requiere más operaciones matriciales, las
%cuales se han hecho a parte y se han transladado a este script únicamente
%las ecuaciones.

%Longitudes links piernas en Base B 
s0_L1_B = [-125 0 95]'; %Para la pierna 1
s1_L1_B = [-55 0 55]';
s2_L1_B = [0 -200 0]';
s3_L1_B = [0 -215 0]';

s0_L2_B = [125 0 95]'; %Para la pierna 2 cambia la componente x de s0 con respecto a pierna 1 en sentido
s1_L2_B = [55 0 55]'; %En la pierna 2, delantera, s1 es en direccion de x creciente. z tambien creciente pq es el lado derecho.
s2_L2_B = [0 -200 0]';
s3_L2_B = [0 -215 0]';

s0_L3_B = [-125 0 -95]'; %Para la pierna 3 cambia la componente z de s0 con respecto a la 1
s1_L3_B = [-55 0 -55]'; %En la pierna 3 cambia la componente z de s1 a negativa pq está a la izquierda
s2_L3_B = [0 -200 0]';
s3_L3_B = [0 -215 0]';

s0_L4_B = [125 0 -95]'; %Comoponente x de s0 positiva. Componente z negativa
s1_L4_B = [55 0 -55]'; %Componente x positiva por estar delante. Componente z negativa por estar a la izquierda.
s2_L4_B = [0 -200 0]';
s3_L4_B = [0 -215 0]';


%Vector posición de los pies en el sistema mundo inicialmente. Es aquí donde considero el
%desplazamiento inicial de B respecto de M y la rotacion inicial en caso de
%haberlos. Voy a suponer que siempre se inicia con un R_inicial=I;
v1_M = vB_M_inicial + ModeloDirectoPierna(RB_inicial, s0_L1_B, s1_L1_B, s2_L1_B, s3_L1_B, q1_L1_i, q2_L1_i, q3_L1_i); %Vector posición inicial de las patas.
v2_M = vB_M_inicial + ModeloDirectoPierna(RB_inicial, s0_L1_B, s1_L1_B, s2_L1_B, s3_L1_B, q1_L2_i, q2_L2_i, q3_L2_i);
v3_M = vB_M_inicial + ModeloDirectoPierna(RB_inicial, s0_L1_B, s1_L1_B, s2_L1_B, s3_L1_B, q1_L3_i, q2_L3_i, q3_L3_i);
v4_M = vB_M_inicial + ModeloDirectoPierna(RB_inicial, s0_L1_B, s1_L1_B, s2_L1_B, s3_L1_B, q1_L4_i, q2_L4_i, q3_L4_i);
vB_M = vB_M_inicial;


%%%%%   EN CASO DE HABER BUCLE PARA ACTUALIZAR LAS Q, A PARTIR DE AQUI   %%%%%%%%

%Cálculo Matriz rotación A1 respecto M. Aquí tiene que ser eye(3) si o si
v1 = ModeloDirectoPierna(eye(3), s0_L1_B, s1_L1_B, s2_L1_B, s3_L1_B, q1_L1, q2_L1, q3_L1); % B y M coincidentes, solo para sacar valores. Es calcular las distancias desde B
v2 = ModeloDirectoPierna(eye(3), s0_L2_B, s1_L2_B, s2_L2_B, s3_L2_B, q1_L2, q2_L2, q3_L2);
v3 = ModeloDirectoPierna(eye(3), s0_L3_B, s1_L3_B, s2_L3_B, s3_L3_B, q1_L3, q2_L3, q3_L3);
v4 = ModeloDirectoPierna(eye(3), s0_L4_B, s1_L4_B, s2_L4_B, s3_L4_B, q1_L4, q2_L4, q3_L4);
pc =  0.25*(v1 + v2 + v3 + v4); %Centro del rectáncgulo que formen los pies en el suelo. En base "B"
a =  0.5*(v2 + v4); %Vector auxiliar para calcular base B2 = nx, ny, nz.
nx = (a - pc)/norm(a-pc) ;
b = (v2 - pc)/norm(v2 - pc); %Vector auxiliar
ny = cross(b,nx)/norm(cross(b,nx));
nz = cross(nx,ny);
R = [nx ny nz]'; %Matriz rotacion A1 respecto M. 

%Suponemos régimen estacionario en el que despues de desplazarse se para y podemos calcular la nueva posicion en el mundo.
v1_M = vB_M + ModeloDirectoPierna(R, s0_L1_B, s1_L1_B, s2_L1_B, s3_L1_B, q1_L1, q2_L1, q3_L1)
v2_M = vB_M + ModeloDirectoPierna(R, s0_L2_B, s1_L2_B, s2_L2_B, s3_L2_B, q1_L2, q2_L2, q3_L2)
v3_M = vB_M + ModeloDirectoPierna(R, s0_L3_B, s1_L3_B, s2_L3_B, s3_L3_B, q1_L3, q2_L3, q3_L3)
v4_M = vB_M + ModeloDirectoPierna(R, s0_L4_B, s1_L4_B, s2_L4_B, s3_L4_B, q1_L4, q2_L4, q3_L4)

[v,Rp] = ModeloDirectoBrazo(q1,q2,q3,q4,q5,q6); %Vector y matriz rotacion el brazo. Ambos respecto a la base del brazo. 
Rp = Rp*R; %Rotacion de la pinza en el sistema M. Ya tiene corregido el cambio de ejes.

vp_M = vB_M + [0 90 0]' + v  %Vector posición de la base de la pinza en el sistema mundo. Es aquí donde sumo el desplazamiento de la base del brazo con respecto de B = acoplador.En altura y ¿en largo?

pc_M =  0.25*(v1_M + v2_M + v3_M + v4_M); %Punto medio del rectángulo que forman los pies, en el sistema M.
vB_M = pc_M - pc; %Actualización de la separación de la base B respecto de M = desplazamiento del robot.

%%%%%%%%%  FIN BUCLE  %%%%%%%%%%%%
