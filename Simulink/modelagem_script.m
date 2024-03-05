%%Script para simulação dinâmica e cinemática do Cubli
clc
clear all
%%%%%%% Sistema de Referência Modelagem
%% Constantes
b = 17.03*10^(-6); %Atrito Viscoso Ponto de Contato
g = [0;0;-9.81]; %Gravidade
mc = 0.55; %Mass Cubli
Ic = [0.01008 -0.00309375 -0.00309375; %Matriz de Inércia
    -0.00309375 0.01008 -0.00309375;
    -0.00309375 -0.00309375 0.01008];
Ic_inv = inv(Ic);
rc = [0.0954545;0.0954545;0.0954545]; %Centro de Massa

%% Casos de Entrada - simular tudo para cada condição inicial com cada uma dessas entradas

Tau = [0;0;0];
%colocar sinal impulso, senoidal e step no simulink

%% Sistema de Referência Modelagem inicial
%%Cálculo do quaternion inicial
phi_s = acos(-sqrt(3)/3); %angulo de giro equilíbrio estável
phi_u = acos(sqrt(3)/3);%angulo de giro equilíbrio instável
e_s = [-sqrt(2)/2;sqrt(2)/2;0]; %eixo de rotação quatérnion de equib. estável
e_u = [sqrt(2)/2;-sqrt(2)/2;0];%eixo de rotação quatérnion de equib. instável

q00 = [cos(phi_u/2);e_u*sin(phi_u/2)]; %posição de equilíbrio instável
%q00 = [cos((phi_u - (10 * pi/180))/2);e_u*sin(((phi_u - 10*pi/180))/2)]; %posição inicial para precessão
%q00 =[cos(phi_s/2);e_s*sin(phi_s/2)]; %posição de equilíbrio estável

%%%%
%%Velocidades Iniciais
%w00 =20*pi/sqrt(3)*[1;1;1];%Velocidade inicial para precessão não estacionária.%
w00 = 2*pi/sqrt(3)*[1;1;1]; %Velocidade inicial para rotação própria
%w00 = [38.47;38.47;39.4];%Velocidade inicial para precessão estácionário
%w00 = [0;0;0]; %Equilibrio

%% Sistema de Coordenadas pião
%%Constantes

v = eig(Ic);
Ic_p = [v(2) 0 0 ; 0 v(3) 0 ; 0 0 v(1)]; %Matriz de Inércia na referencia de pião
Ic_pinv = inv(Ic_p);
rc_p = [0;0;norm(rc)]; 
%Quaternion inicial para referência de pião
r0 = [0;0;1];
theta = 10*pi/180; %Angulo de nutação inicial
r1 = [sin(theta);0;cos(theta)];
e = cross(r0,r1)/norm(cross(r0,r1)); %eixo de rotação quaternion
phi_p = acos(dot(r0,r1)/(norm(r0)*norm(r1))); %Angulo de giro quaternion
%q00_p = [cos(phi_p/2);e*sin(phi_p/2)]; %Quaternion inicial para precessão
q00_p = [1;0;0;0]; %Quaternion inicial para rotação própria

%%Velocidades angulares inicias
%w00_p = [0;0;20*pi]; %Velocidade inicial para precessão não estacionária
%w00_p = [0;0;2*pi]; %Velocidade inicial para rotação própria
w00_p = [0;0;0]; %Velocidade inicial Equilibrio

%Eixos do sistema modelado no sistema "pião"

%w00_p = [-4.4*sind(10);0;20*pi + 4.4*sind(10)]; %Velocidade inicial para precessão estacionário
Tau_p = [0;0;norm(Tau)]; %Torque para sistema de pião considerando torque igual nas 3 rodas de reação

%-----------------%

A=[-10.9808	40.9808	-30.	0.444037	0.	-0.162529
-40.9808	10.9808	30.	0.	0.444037	-0.162529
19.0192	-19.0192	0.	0.162529	0.162529	0.444037
110.967	-41.5293	-25.4161	-0.00231609	42.2705	-42.2726
-41.5293	110.967	-25.4161	-42.2726	-0.00231609	42.2705
-69.4381	-69.4381	50.8322	42.2705	-42.2726	-0.00231609];

B=[0.	0.	0.;
0.	0.	0.;
0.	0.	0.;
136.24	60.3319	60.3319;
60.3319	136.24	60.3319;
60.3319	60.3319	103.332];

C = eye(6);
D = zeros(6,3);


open modelagem_2021a_2.slx

