%% Polos do Sistema em Malha Aberta
% A = [0., 0., 0., 0.5, 0., 0.;
%  0., 0., 0., 0., 0.5, 0.;
%  0., 0., 0., 0., 0., 0.5;
%  78.1896, 0., 0., -0.00129272, 0., 0.;
%  0., 78.1896, 0., 0., -0.00129272, 0.;
%  -264.624, -264.624, 0., 0., 0., -0.00437508];
A = [0., 0., 0., 0.5, 0., 0.;
 0., 0., 0., 0., 0.5, 0.;
 0., 0., 0., 0., 0., 0.5;
 209.258, 0., 0., -0.00129272, 0., 0.;
 0., 209.258, 0., 0., -0.00129272, 0.;
 0, 0, 0., 0., 0., -0.00437508];
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
p = [0.1,-0.1 + 0.1j,-0.1-0.1j,0.15,-0.3+0.3j,-0.3-0.3j];
K = place(A,B,p);

A_bar = (A-B*K);


%% LQR
% Q = [5,0,0,0,0,0;
%     0,5,0,0,0,0;
%     0,0,5,0,0,0;
%     0,0,0,0.001,0,0;
%     0,0,0,0,0.001,0;
%     0,0,0,0,0,0.001]*1e-1;
% R = 20*eye(3,3);
% % K = lqr(sys_cont,Q,R);
% %% Simulação
% sys_lin = ss(A-B*K,E,C,D);
% t = linspace(0,12,1200);
% x0 = [q00(2);q00(3);q00(4);w00(1);w00(2);w00(3)];
% impulseX = zeros(1,1200);
% impulseX(1,500) = 1.2;
% impulseY = zeros(1,1200);
% impulseY(1,250) = 1.2;
% impulseZ = zeros(1,1200);
% impulseZ(1,750) = 1.2;
% % w = zeros(3,length(t));
% w = [impulseX;impulseY;impulseZ]';
% [y,t,x] = lsim(sys_lin,w,t,x0);
% u = -K*(x');
% pole(sys_lin)
% h1 = pzplot(sys_lin);
% pp = getoptions(h1);
% pp.Title.String= "Pole Map";
% setoptions(h1,pp);
% q1 = x(:,1);
% q2=x(:,2);
% q3=x(:,3);
% q0 = sqrt(1 - q1.^2 - q2.^2 - q3.^2);
% quat = [q0,q1,q2,q3];
% euler = quat2eul(quat)*180/pi; %degress