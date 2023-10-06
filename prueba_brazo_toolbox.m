
clear L


%Longitudes links
L1 = 104.43;
L21 = 250.00;
L22 = 50.00;
L3 = 171.80;
L4 = 78.2;
L5 = 65.00;
L6 = 66.00; 


%Valores útiles
arc = atan(L22/L21);
d = sqrt(L21^2 + L22^2); %Distancia para DH


%Parámetros Denavit-Hartenberg

theta2 = pi/2 - arc ;

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

alpha1 = pi/2;
alpha2 = 0;
alpha3 = pi/2;
alpha4 = -pi/2;
alpha5 = pi/2;
alpha6 = 0;

%Limites grados libertad en radianes
q1min = -pi; q1max= pi; % waist
q2min = -108*pi/180 - arc; q2max= 114*pi/180 - arc; % shoulder
q3min = -123*pi/180 - arc; q3max= 92*pi/180 -arc; % elbow
q5min = -100*pi/180; q5max= 123*pi/180; % wrist angle
q4min = -pi; q4max= pi; % Forearm roll
q6min = -pi; q6max= pi; % wrist rotate



L(1) = Revolute('d', d1, ...   % link length (Dennavit-Hartenberg notation)
    'a', a1, ...               % link offset (Dennavit-Hartenberg notation)
    'alpha', alpha1, ...        % link twist (Dennavit-Hartenberg notation)
    'qlim', [q1min q1max] ); % minimum and maximum joint angle

L(2) = Revolute('d', d2, ...   % link length (Dennavit-Hartenberg notation)
    'a', a2, ...               % link offset (Dennavit-Hartenberg notation)
    'alpha', alpha2, ... % link twist (Dennavit-Hartenberg notation)
    'offset', (pi/2 - arc), ... %Offset pq no deja meter theta.
    'qlim', [q2min q2max] ); % minimum and maximum joint angle

L(3) = Revolute('d', d3, ...   % link length (Dennavit-Hartenberg notation)
    'a', a3, ...               % link offset (Dennavit-Hartenberg notation)
    'alpha', alpha3, ...        % link twist (Dennavit-Hartenberg notation)
    'offset', (arc), ...
    'qlim', [q3min q3max] ); % minimum and maximum joint angle

L(4) = Revolute('d', d4, ...   % link length (Dennavit-Hartenberg notation)
    'a', a4, ...               % link offset (Dennavit-Hartenberg notation)
    'alpha', alpha4, ...        % link twist (Dennavit-Hartenberg notation)
    'qlim', [q4min q4max] ); % minimum and maximum joint angle

L(5) = Revolute('d', d5, ...   % link length (Dennavit-Hartenberg notation)
    'a', a5, ...               % link offset (Dennavit-Hartenberg notation)
    'alpha', alpha5, ...        % link twist (Dennavit-Hartenberg notation)
    'qlim', [q5min q5max] ); % minimum and maximum joint angle

L(6) = Revolute('d', d6, ...   % link length (Dennavit-Hartenberg notation)
    'a', a6, ...               % link offset (Dennavit-Hartenberg notation)
    'alpha', alpha6, ...        % link twist (Dennavit-Hartenberg notation)
    'qlim', [q6min q6max] ); % minimum and maximum joint angle

qz = [0 0 0 0 0 0]; % zero angles, L shaped pose
qr = [0 pi/2 pi/2 0 0 0]; % ready pose, arm up
qs = [0 0 -pi/2 0 0 0];
qn=[0 pi/4 pi 0 pi/4 0];


brazo = SerialLink(L, 'name', 'Brazo', ...
    'configs', {'qz', qz, 'qr', qr, 'qs', qs, 'qn', qn});


brazo.model3d = 'UNIMATE/brazo';
brazo.teach();


clear L

