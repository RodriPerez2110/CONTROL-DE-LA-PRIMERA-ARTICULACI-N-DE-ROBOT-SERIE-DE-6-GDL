%Control y Sistemas | Facultad de Ingeniería
%Proyecto Integrador | Febrero de 2024
%Rodrigo Pérez(11068)

clear; clc; close all;

%% PLANTA

%Parámetros Dinámicos:
Jl_min = 0.56982; %[kg.m^2]
Jl_max = 12.54696; %[kg.m^2]
Jeq_min = 6.58e-4; %[kg.m^2]
Jeq_max = 11.26e-4; %[kg.m^2]
Jm = 6.356e-4; %[kg.m^2] (Momento de inercia del rotor junto con la caja reductora).
beq = 3.70e-4; %[(N.m)/(rad/s)]
r = 160;

%Parámetros Eléctricos:
Ra = 0.7; %[ohm]
La = 0.0013; %[H]
KE = 0.1412; %[(N.m)/A]
KT = KE;

%Puntos de Operación:
format long
CantPuntosOp = 4;
paso_Jl = (Jl_max-Jl_min)/(CantPuntosOp-1);
Jl = zeros(1,CantPuntosOp);
Jeq = zeros(1,CantPuntosOp);
Jl(1) = Jl_min;
Jl(CantPuntosOp) = Jl_max;
Jeq(1) = Jm+Jl_min/r^2;
Jeq(CantPuntosOp) = Jm+Jl_max/r^2;
for i=2:CantPuntosOp-1
  Jl(i) = Jl(i-1)+paso_Jl;
  Jeq(i) = Jm+Jl(i)/r^2;
end

% Familia de Plantas:
CantSalidas = 1;
CantEntradas = 2;
Plantas = ss(zeros(CantSalidas, CantEntradas, CantPuntosOp)); % Inicialización de plantas con ceros
for i=1:CantPuntosOp
  A = [0 1 0 ; 0 -beq/Jeq(i) KT/Jeq(i) ; 0 -KE/La -Ra/La];
  B = [0 0 ; 0 -1/Jeq(i) ; 1/La 0];
  C = [1 0 0];
  D = [0 0];
  Plantas(1,:,i) = ss(A,B,C,D);
end

%% ANÁLISIS de ESTABILIDAD Y AMORTIGUAMIENTO DE LAS PLANTAS a lazo abierto

%PARÁMETROS DE SEGUNDO ORDEN de las Plantas a Lazo Abierto, respecto a la Entrada Manipulada 'va':
wn = zeros(1,CantPuntosOp);
z = zeros(1,CantPuntosOp);
for i=1:CantPuntosOp
  wn(i) = sqrt((Ra*beq+KT*KE)/(La*Jeq(i)));
  z(i) = (Ra*Jeq(i)+La*beq)/(La*Jeq(i)*2*wn(i));
end

% POLOS de la Planta a Lazo Abierto, respecto a la Entrada Manipulada:
syms s
P=zeros(1,CantPuntosOp);
Pp=zeros(2,CantPuntosOp);
for i=1:CantPuntosOp
  P = (La*Jeq(i))*(s^2 + 2*z(i)*wn(i)*s + wn(i)^2);
  Pp(:,i) = double(solve(P==0,s));
end

%MAPA DE POLOS Y CEROS de la Planta a Lazo Abierto, respecto a la Entrada Manipulada:
Gva=tf(zeros(1,CantPuntosOp));
figure(1);
hold on;
for i=1:CantPuntosOp
  Gva(i) = tf((KT/(La*Jeq(i))),[1 2*z(i)*wn(i) wn(i)^2 0]);
  pzmap(Gva(i));
  hm = findobj(gca, 'Type', 'Line');          % Handle To 'Line' Objects
  hm(2).MarkerSize = 20;                      % ‘Zero’ Marker
  hm(3).MarkerSize = 20;                      % ‘Pole’ Marker
end
sgrid;
legend(cellstr(num2str(Jl', 'Jl=%.2d')));
hold off;

% Corroboración de la estabilidad de las plantas con la función isstable():
isstable(Plantas,'elem')

%% DIAGRAMA DE BODE de la Planta a Lazo Abierto, respecto a la Entrada Manipulada:
figure(2);
hold on;
for i=1:CantPuntosOp
  bode(Plantas(1,1,i));
end
grid;
legend(cellstr(num2str(Jl', 'Jl=%.2d')));
hold off;

% SENSIBILIDAD de la Planta, a Lazo Cerrado con realimentación unitaria, respecto a la Entrada Manipulada:
figure(3);
hold on;
for i=1:CantPuntosOp
  bode(feedback(1,Plantas(1,1,i)));
end
grid;
legend(cellstr(num2str(Jl', 'Jl=%.2d')));
hold off;

%% CONTROLABILIDAD Y OBSERVABILIDAD
for i=1:CantPuntosOp
  %División de la matriz B en la matriz de entrada manipulada B_va y de entrada de perturbación B_Tl:
  B_va = Plantas(1,:,i).B(:,1); %En realidad todos los vectores B_va(:,i) son iguales.
  B_Tl = Plantas(1,:,i).B(:,2);
  %Matrices de Controlabilidad y Observabilidad:
  Cont = [B_va , Plantas(1,:,i).A*B_va , Plantas(1,:,i).A^2*B_va]; %[B_va , A*B_va , A^2*B_va]
  Obs = [Plantas(1,:,i).C ; Plantas(1,:,i).C*Plantas(1,:,i).A ; C*Plantas(1,:,i).A^2]; %[C ; C*A ; C*A^2]
  %Rango de las matrices de Controlabilidad y Observabilidad:
  fprintf('Para Jl=%.2d: rank(Cont)=%d y rank(Obs)=%d', Jl(i), rank(Cont), rank(Obs));
  fprintf('\n');
end

%% GAIN SCHEDULING
%Ajuste de los parámetros de la familia de controladores I-PD:
[ControladoresPID(1,:,:),info] = pidtune(Plantas(1,1,:),'I-PD',30);

%%ANÁLISIS de ESTABILIDAD del sistema controlador-planta:
for i=1:CantPuntosOp
  fprintf('Info del controlador-planta Jl=%.2d:', Jl(i));
  fprintf('\n');
  disp(['   Stable: ', num2str(info(i).Stable)]);
  disp(['   Crossover Frequency: ', num2str(info(i).CrossoverFrequency)]);
  disp(['   Phase Margin: ', num2str(info(i).PhaseMargin)]);
  fprintf('\n');
end

%% FILTRO DE KALMAN:
% Matriz de covarianza del ruido del proceso:
Rv = 1e-3*eye(3); %Q

% Matriz de covarianza del ruido de medición:
Rw = 2.89e-8; %R
