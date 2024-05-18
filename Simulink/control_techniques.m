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
C = [1	0	0	0	0	0;
    0	1	0	0	0	0;
    0	0	1	0	0	0];
D = zeros(3,3);
E = B;
sys_cont = ss(A,B,C,D);

% h = pzplot(sys_cont);
% hold on
P = pole(sys_cont);

%% Alocação de Polos
p = [-7,-7 + 2j,-7 - 2j,-6,-6+3j,-6-3j];
% K = place(A,B,p);




%% LQR
Q = [2,0,0,0,0,0;
    0,2,0,0,0,0;
    0,0,2,0,0,0;
    0,0,0,0.1,0,0;
    0,0,0,0,0.1,0;
    0,0,0,0,0,0.1];
R = 50*eye(3,3);
K = lqr(sys_cont,Q,R);
%% Simulação
sys_lin = ss(A-B*K,E,C,D);
t = linspace(0,10,1000);
x0 = [q00(2);q00(3);q00(4);w00(1);w00(2);w00(3)];
impulseX = zeros(1,1000);
impulseX(1,500) = 1.2;
impulseY = zeros(1,1000);
impulseY(1,250) = 1.2;
impulseZ = zeros(1,1000);
impulseZ(1,750) = 1.2;
w = zeros(3,length(t));
% w = [impulseX;impulseY;impulseZ]';
[y,t,x] = lsim(sys_lin,w,t,x0);
u = -K*(x');
pole(sys_lin)
% h1 = pzplot(sys_lin);


q1 = x(:,1);
q2=x(:,2);
q3=x(:,3);
q0 = sqrt(1 - q1.^2 - q2.^2 - q3.^2);
quat = [q0,q1,q2,q3];
euler = quat2eul(quat)*180/pi; %degress