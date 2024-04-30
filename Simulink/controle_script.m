%%Script para simulação dinâmica e cinemática do Cubli
clc
clear all
%%%%%%% Sistema de Referência Modelagem
%% Constantes
b = 17.03*10^(-6); %Atrito Viscoso Ponto de Contato
g = [0;0;-9.81]; %Gravidade
mc = 0.55; %Mass Cubli
Ic_norm = [0.01008	-0.00309375	-0.00309375;
-0.00309375	0.01008	-0.00309375;
-0.00309375	-0.00309375	0.01008];
[eigvec,a] = eig(Ic_norm);
Ic = [0.0131738	0	0;
        0	0.0131738	0;
        0	0	0.0038925];
Ic_inv = inv(Ic);
A = eigvec(:,1);
B = eigvec(:,2);
C = eigvec(:,3);
T_RW2C = [C,B,A]';





% T_RW2C = normalize([-1,1,0;-1,0,1;1,1,1],"norm",2);     %Matriz de transformação entre ...
%Sistema de coordenadas de Controle e Sistema de Coordenadas em relação às RW

rc = [0;0;norm([0.0954545, 0.0954545, 0.0954545])]; %Centro de Massa


%% Sistema de Referência Modelagem inicial

%Quaternion inicial para referência de pião

r0 = [0;0;1];
theta = 45*pi/180; %Angulo de nutação inicial
r1 = [sin(theta);0;cos(theta)];
e = cross(r0,r1)/norm(cross(r0,r1)); %eixo de rotação quaternion

phi_p = acos(dot(r0,r1)/(norm(r0)*norm(r1))); %Angulo de giro quaternion
q00 = [cos(phi_p/2);e*sin(phi_p/2)]; %Quaternion inicial para precessão
% q00 = [1;0;0;0] %Quaternion inicial para rotação própria

%%Velocidades angulares inicias
w00 = [1;1;1]; %Velocidade inicial Equilibrio
%% Pontos de linearização
q_bar = [1;0;0;0];
w_bar = [0;0;0];

%-----------------%

run control_techniques.m

open ControleSimulink.slx
