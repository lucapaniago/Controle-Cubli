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
C = [1	0	0	0	0	0
    0	1	0	0	0	0;
    0	0	1	0	0	0;
    0	0	0	1	0	0;
    0	0	0	0	1	0;
    0	0	0	0	0	1];
D = zeros(6,3);
sys = ss(A,B,C,D);

% h = pzplot(sys);
% hold on
P = pole(sys);

%% Alocação de Polos
% p = [-20,-30,-40,-5,-6,-7];
% K = place(A,B,p);
% sys_lin = ss(A-B*K,B,C,D);
% t = linspace(0,4,1000);
% x0 = [0.2;0.1;0.14;1.2;1.2;1.2];
% y = lsim(sys_lin,zeros(3,1000),t,x0);
% u = -K*(y');
% figure(1)
% plot(t,y(:,1:3),"LineWidth",1.5)
% title("Resposta Dinâmica da Atitude em quaternions",'FontSize',12)
% xlabel("Tempo [s]","FontSize",12)
% ylabel("Quaternion","FontSize",12)
% legend("$q_1$","$q_2$","$q_3$","Interpreter",'latex','FontSize',12)
% figure(2)
% plot(t,y(:,4:6),"LineWidth",1.5)
% title("Resposta Dinâmica das Velocidades Angulares",'FontSize',12)
% xlabel("Tempo [s]","FontSize",12)
% ylabel("Velocidadas Angulares [rad/s]","FontSize",12)
% legend("$\omega_x$","$\omega_y$","$\omega_z$","Interpreter",'latex','FontSize',12)
% figure(3)
% u = -K*(y');
% plot(t,u(1:3,:),"LineWidth",1.5)
% title("Entradas de Torque pelas Rodas de Reação",'FontSize',12)
% xlabel("Tempo [s]","FontSize",12)
% ylabel("Torque [N.m~]","FontSize",12)
% legend("$\tau_x$","$\tau_y$","$\tau_z$","Interpreter",'latex','FontSize',12)

% figure(2)

% h1 = pzplot(sys_lin);

%% LQR
Q = [5,0,0,0,0,0;
    0,5,0,0,0,0;
    0,0,5,0,0,0;
    0,0,0,0.1,0,0;
    0,0,0,0,0.1,0;
    0,0,0,0,0,0.1];
R = eye(3,3);
K = lqr(sys,Q,R);
sys_lin = ss(A-B*K,B,C,D);
pole(sys_lin)
