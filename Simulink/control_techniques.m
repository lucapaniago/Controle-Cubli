%% Polos do Sistema em Malha Aberta
A = [0., 0., 0., 0.5, 0., 0.;
 0., 0., 0., 0., 0.5, 0.;
 0., 0., 0., 0., 0., 0.5;
 78.1896, 0., 0., -0.00129272, 0., 0.;
 0., 78.1896, 0., 0., -0.00129272, 0.;
 -264.624, -264.624, 0., 0., 0., -0.00437508];
B = [0.	0.	0.;
0.	0.	0.;
0.	0.	0.;
75.9085	0.	0.;
0.	75.9085	0.;
0.	0.	256.904];
C = [0	0	0	1	0	0;
    0	0	0	0	1	0;
    0	0	0	0	0	1];
D = zeros(3,3);
E = B;
sys = ss(A,B,C,D);

% h = pzplot(sys);
% hold on
P = pole(sys);

%% Alocação de Polos
p = [-10,-2,-3,-4,-6,-8];
K = place(A,B,p);




%% LQR
% Q = [1,0,0,0,0,0;
%     0,1,0,0,0,0;
%     0,0,1,0,0,0;
%     0,0,0,1,0,0;
%     0,0,0,0,1,0;
%     0,0,0,0,0,1];
% R = 500*eye(3,3);
% K = lqr(sys,Q,R);
%% Simulação
sys_lin = ss(A-B*K,E,C,D);
t = linspace(0,10,1000);
x0 = [0.05;0.1;0.14;0.1;0.1;0.1];
impulseX = zeros(1,1000);
impulseX(1,500) = 0.5;
impulseY = zeros(1,1000);
impulseY(1,250) = 0.5;
impulseZ = zeros(1,1000);
impulseZ(1,750) = 0.2;
w = zeros(3,length(t));
% w = [impulseX;impulseY;impulseZ]';
[y,t,x] = lsim(sys_lin,w,t,x0);
u = -K*(x');
pole(sys_lin)
h1 = pzplot(sys_lin);


q1 = x(:,1);
q2=x(:,2);
q3=x(:,3);
q0 = sqrt(1 - q1.^2 - q2.^2 - q3.^2);
quat = [q0,q1,q2,q3];
euler = quat2eul(quat)*180/pi; %degress