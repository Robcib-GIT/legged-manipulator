
%Función que devuelve un vector posicion en un sistema JUSTO EN LOMO del
%A1, es decir, en B. Está contemplada la rotación respecto al sistema M.

function [v] = ModeloDirectoPierna(RB, s0_B, s1_B, s2_B, s3_B, q1,q2,q3)

%Matrices de rotacion
R1 = [1 0 0; 0 cos(q1) -sin(q1); 0 sin(q1) cos(q1)];
R2 = [cos(q2) -sin(q2) 0; sin(q2) cos(q2) 0; 0 0 1];
R3 = [cos(q3) -sin(q3) 0; sin(q3) cos(q3) 0; 0 0 1];

%Longitudes links en Base M
s0_M = RB*s0_B;
s1_M = RB*R1*s1_B;
s2_M = RB*R1*R2*s2_B;
s3_M = RB*R1*R2*R3*s3_B;

%Modelo directo. Coordenadas base B
v = s0_M + s1_M + s2_M + s3_M;

end