
%%%%% SE APLICA EL MÉTODO DE DESACOPLO CINEMÁTICO %%%%

% Los valores que definen la posición de pm es el vector p_m. Mientras que
% T orienta y posiciona al extremo, al ser modelo inverso se supone T
% conocida.
% Todas las ecuaciones tienen dos soluciones, he cogido siempre la primera
% porque son +- rad p.ej: 3.16rad y -0.02rad. Es decir, la misma solución.

T = [0 0 1 431; 0 -1 0 0.01; 1 0 0 354; 0 0 0 1]; %Los valores de rotacion y posicion del extremo serán conocidos. Podría hacerse una función y pasarlos como parámetros.

% Generales
syms x y z q1 q2 q3 q4 q5 q6;
syms pim; 

% Longitudes links
% L1 = 104.43;
% L21 = 250.00;
% L22 = 50.00;
% L3 = 171.80;
% L4 = 78.2;
% L5 = 65.00;
% L6 = 66.00; 
syms L1 L21 L22 L3 L4 L5 L6 ;

% Valores útiles
%pm = [; ; ]; %Solo depende de q1 q2 q3
arc = atan(L22/L21);
d = sqrt(L21^2 + L22^2); %Distancia para DH

% Parámetros Denavit-Hartenberg
theta1 = q1;
theta2 = pim - arc + q2;
theta3 = arc + q3;
theta4 = q4;
theta5 = q5;
theta6 = q6;

d1 = L1;
d2 = 0;
d3 = 0;
d4 = L3 + L4;
d5 = 0;
d6 = L5 + L6;

a1 = 0;
a2 = d;
a3 = 0;
a4 = 0;
a5 = 0;
a6 = 0;

alpha1 = pim;
alpha2 = 0;
alpha3 = pim;
alpha4 = -pim;
alpha5 = pim;
alpha6 = 0;


% Obtención de q1,q2,q3. Vamos despejando de T = A1*A2*A3
A1 = genera_A(theta1,d1,a1,alpha1);
R1trans = A1(1:3,1:3).';
A1inv = [ R1trans, -R1trans*A1(1:3,4); 0 0 0 1]; %Mirar apuntes robotica para hacer inversa de matrices transformación homogénea.

A2 = genera_A(theta2,d2,a2,alpha2);
R2trans = A2(1:3,1:3).';
A2inv = [ R2trans, -R2trans*A2(1:3,4); 0 0 0 1];

A3 = genera_A(theta3,d3,a3,alpha3);
R3trans = A3(1:3,1:3).';
A3inv = [ R3trans, -R3trans*A3(1:3,4); 0 0 0 1];

p_m = T(1:4,4) - (L5+L6)*T(1:4,3) % Posicion del punto de desacople.



%%%% Comenzamos previo a p_m %%%%

% A1inv*T = A2*A3
s = A1inv(3,1:4)*p_m; %Al usar p_m estamos obligados a igualar a un elemento de la 4 columna de la matriz resultante a la dcha de la igualdad
collect(s);
s = subs(s, [pim,L1,L21, L22, L3, L4, L5, L6], [pi/2,104.43, 250.00, 50.0, 171.80, 78.2, 65.00, 66.00]);
s = simplify(s);
%pretty(s);
ecu = s==0; % elemento (3,4) del producto A2*A3
sol = solve(ecu,q1);
sol = simplify(sol);
sol_q1 = eval(sol)

% A2inv*A1inv*T = A3
A21inv = A2inv*A1inv;
s = A21inv(2,1:4)*p_m;
collect(s);
s = subs(s, [pim,q1,L1,L21, L22, L3, L4, L5, L6], [pi/2, sol_q1(1),104.43, 250.00, 50.0, 171.80, 78.2, 65.00, 66.00]);
s = simplify(s);
%pretty(s); % Ver qué angulos son los mismos y usar las relaciones de los videos de brrientos
ecu = s==0;
sol = solve(ecu,q2);
sol = simplify(sol);
sol_q2 = eval(sol)

% A3inv*A2inv*A1inv*T = I
A321inv = A3inv*A2inv*A1inv;
s = A321inv(1,1:4)*p_m;
collect(s);
s = subs(s, [pim, q1, q2, L1, L21, L22, L3, L4, L5, L6], [pi/2, sol_q1(1), sol_q2(1), 104.43, 250.00, 50.0, 171.80, 78.2, 65.00, 66.00]);
s = simplify(s);
%pretty(s); % Ver qué angulos son los mismos y usar las relaciones de los videos de brrientos
ecu = s==0; % Elemento (1,4) de la matriz identidad.
sol = solve(ecu,q3);
sol = simplify(sol);
sol_q3 = eval(sol)



%%%% Seguimos a partir de p_m %%%%

% Busco R_36 porque así oriento la pinza desde pm
R_06 = T(1:3,1:3);
R_03 = A1(1:3,1:3)*A2(1:3,1:3)*A3(1:3,1:3);
R_03 = collect(R_03);
R_03 = simplify(R_03);
R_03 = subs(R_03, [pim, q1, q2, q3,L1, L21, L22, L3, L4, L5, L6], [pi/2, sol_q1(1), sol_q2(1), sol_q3(1), 104.43, 250.00, 50.0, 171.80, 78.2, 65.00, 66.00]);
R_03 = simplify(R_03);
R_36 = (R_03.')*R_06; % R_03 * R_36 = T(1:3,1:3)
%R_36 = eval(R_36);

% Resuelvo R_36 = R_34*R_45*R_56 de la misma forma que antes
A4 = genera_A(theta4,d4,a4,alpha4);
R_34 = A4(1:3,1:3);
R_34trans = R_34.';

A5 = genera_A(theta5,d5,a5,alpha5);
R_45 = A5(1:3,1:3);
R_45trans = R_45.';

A6 = genera_A(theta6,d6,a6,alpha6);
R_56 = A6(1:3,1:3);
R_56trans = R_56.';


s = R_34trans(3, 1:3)*R_36(1:3,3); % He elegido estos valores porque son los más cortos para operar
collect(s);
s = subs(s, [pim,L1,L21, L22, L3, L4, L5, L6], [pi/2,104.43, 250.00, 50.0, 171.80, 78.2, 65.00, 66.00]);
s = simplify(s);
%pretty(s);
R_46 = R_45*R_56;
R_46 = subs(R_46, pim, pi/2);
ecu = s==R_46(3,3); % elemento (3,4) del producto A2*A3
sol = solve(ecu,q4);
sol = simplify(sol);
sol_q4 = eval(sol)


s = R_34trans(1, 1:3)*R_36(1:3,3); % He elegido estos valores porque son los más cortos para operar
collect(s);
s = subs(s, [pim,q4,L1,L21, L22, L3, L4, L5, L6], [pi/2, sol_q4(1), 104.43, 250.00, 50.0, 171.80, 78.2, 65.00, 66.00]);
s = simplify(s);
%pretty(s);
R_46 = R_45*R_56;
R_46 = subs(R_46, pim, pi/2);
ecu = s==R_46(1,3); % elemento (3,4) del producto A2*A3
sol = solve(ecu,q5);
sol = simplify(sol);
sol_q5 = eval(sol)


s = R_34trans(3, 1:3)*R_36(1:3,1); % He elegido estos valores porque son los más cortos para operar
collect(s);
s = subs(s, [pim,q4,L1,L21, L22, L3, L4, L5, L6], [pi/2, sol_q4(1), 104.43, 250.00, 50.0, 171.80, 78.2, 65.00, 66.00]);
s = simplify(s);
%pretty(s);
R_46 = R_45*R_56;
R_46 = subs(R_46, pim, pi/2);
ecu = s==R_46(3,1); % elemento (3,4) del producto A2*A3
sol = solve(ecu,q6);
sol = simplify(sol);
sol_q6 = eval(sol)


q = 180/pi*[sol_q1(1);sol_q2(1);sol_q3(1);sol_q4(1);sol_q5(1);sol_q6(1)]
