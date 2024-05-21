%% Observador de Estados de Ordem Reduzida

V = [zeros(3,3), eye(3,3)];
T = [C;V];
inv_T = inv(T);
M = inv_T(:,1:3);
N = inv_T(:,4:6);
A11 = C*A*M;
A12 = C*A*N;
A21 = V*A*M;
A22 = V*A*N;
B1 = C*B;
B2 = V*B;

%Sistem fictício com matriz de estados A11^T e matriz de entradas A12^T

%% Método de Alocação de Polos para obtenção de matriz J
% p_obs = [-5,-5+1j,-5-1j]; %PP
p_obs = [-11,-11+j,-11-j];%LQR

J = place(A22.',A12.',p_obs);
% Q_obs =diag([5,9,7]);
% R_obs = eye(3,3);
% J = lqr(A22.',A12.',Q_obs,R_obs);

F = A22 - J*A12;
eig(F)
G = A21 -J*A11 + F*J;
H = B2 - J*B1;
S = M + N*J;

%autovalores e autovetores da dinâmica do erro em MF
[obs_autovec, obs_autoval] = eig(F);

%% Novo sistema na forma de espaço de estados

A_obs = [A, zeros(6,3);
        G*C, F];
B_obs = [B;H];

E_obs = [E;zeros(3,3)];

C_obs = [C,zeros(3,3)];

D_obs = zeros(3,3);

A_obs_c = A_obs - B_obs*K*[S*C,N];

sys_obs_u = ss(A_obs_c,E_obs,-K*[S*C,N],0);

h3 = pzplot(sys_obs_u);


%% Simulação
t = linspace(0,12.5,1250);
impulseX = zeros(1,1250);
impulseX(1,500) = 1.2;
impulseY = zeros(1,1250);
impulseY(1,250) = 1.2;
impulseZ = zeros(1,1250);
impulseZ(1,750) = 1.2;
w = zeros(3,length(t));
% w = [impulseX;impulseY;impulseZ]';
x0_obs = [q00(2);q00(3);q00(4);w00(1);w00(2);w00(3);0;0;0];
[u_obs,t,x_z] = lsim(sys_obs_u,w,t,x0_obs);
% y = vetor de entradas
pole(sys_obs_u)
h1 = pzplot(sys_obs_u);

x_hat = (S*C*x_z(:,1:6)' + N*x_z(:,7:9)')';

w_x = x_z(:,4);
w_y =x_z(:,5);
w_z =x_z(:,6);

w_x_hat = x_hat(:,4);
w_y_hat = x_hat(:,5);
w_z_hat = x_hat(:,6);

q1 = x_z(:,1);
q2=x_z(:,2);
q3=x_z(:,3);
q0 = sqrt(1 - q1.^2 - q2.^2 - q3.^2);
quat = [q0,q1,q2,q3];
euler = quat2eul(quat)*180/pi; %degress


