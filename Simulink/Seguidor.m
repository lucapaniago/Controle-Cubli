% Seguidor de Referência
t = linspace(0,10,1000);
%% Seguidor LQ
Q_ref =[1,0,0,0,0,0;
    0,1,0,0,0,0;
    0,0,1,0,0,0;
    0,0,0,0.01,0,0;
    0,0,0,0,0.01,0;
    0,0,0,0,0,0.01]*1e-3;
R_ref =100*diag([1,1,1]);


theta_ref = 6*pi/180; %Angulo de nutação inicial
r1_ref = [sin(theta_ref);0;cos(theta_ref)];
e = cross(r0,r1_ref)/norm(cross(r0,r1_ref)); %eixo de rotação quaternion

phi_p_ref = acos(dot(r0,r1_ref)/(norm(r0)*norm(r1_ref))); %Angulo de giro quaternion

% eul = [5,5,0]*pi/180;
% q_ref = eul2quat(eul);
% q_ref = [0,0.1,0.1,0.1];
q_ref = [cos(phi_p_ref/2);e*sin(phi_p_ref/2)]; %Quaternion inicial para precessão
w_ref = [0;0;0];
% x_ref = [q_ref(2)*ones(1,500),-q_ref(2)*ones(1,500);q_ref(3)*ones(1,500),-q_ref(3)*ones(1,500);q_ref(4)*ones(1,500),-q_ref(4)*ones(1,500);
%         zeros(1,1000);zeros(1,1000);w_ref(3)*ones(1,1000)];
x_ref =[q_ref(2)*ones(1,800),zeros(1,200);q_ref(3)*ones(1,800),zeros(1,200);q_ref(3)*ones(1,800), zeros(1,200);
        zeros(1,1000);zeros(1,1000);zeros(1,1000)];
[K_lq,p_lq,poles_lq] = lqr(sys_cont,Q_ref,R_ref);

A_c_lq = (A-B*K_lq);

eta_ss = ss(A_c_lq,Q_ref,eye(6),zeros(6,6));

eta0 = p_lq*x_ref(:,end);

eta = lsim(eta_ss,flip(x_ref,2) ,t,eta0);

sys_ref = ss(A_c_lq,B*inv(R_ref)*B',-K_lq,inv(R_ref)*B');

[u_lq,t,x_lq] = lsim(sys_ref,flip(eta,1),t);
% plot(t,x_lq(:,3))
hold on
% plot(t,x_ref(3,:))

% plot(t,u_lq)
% q0 = sqrt(1-x_lq(:,1).^2 - x_lq(:,2).^2 - x_lq(:,3).^2)';
% euler = quat2eul(q);

%% Seguidor Variável Exógena
%Seno
ws = 2;
zeta = 0.4;
A_r = [0,0,0,0.5,0,0;
        0,0,0,0,0.5,0;
        0,0,0,0,0,0.5;
        -8,0,0,-2*zeta,0,0;
        0,-8,0,0,-2*zeta,0;
        0,0,-8,0,0,-2*zeta];
%Degrau
% A_r = [0,0,0,0.5,0,0;
%         0,0,0,0,0.5,0;
%         0,0,0,0,0,0.5;
%         0,0,0,0,0,0;
%         0,0,0,0,0,0;
%         0,0,0,0,0,0];
    
M =[[1,0,0,0,0,0];
    [0,1,0,0,0,0];
    [0,0,1,0,0,0]];

A_cl_inv = inv(A-B*K);
N = inv(M*A_cl_inv*B)*M*A_cl_inv;
G_r = N*(A-A_r);
% w_seg = [(0.3*sin(100*t))';(0.3*sin(100*t))';(0.3*sin(100*t))']';
w_seg = 0.1*ones(1000,3);
A_ex_cl = [[A-B*G_r,B*(K-G_r)];[(A_r - A +B*G_r),(A_r - B*(K-G_r))]];
% E_ex_cl = [E;zeros(6,3)];
E_ex_cl = [-B*N*E + E ; B*N*E-E];
K_ex = [-G_r,K-G_r];

seg_exo_ss = ss(A_ex_cl,E_ex_cl,K_ex,0);
q_ref = eul2quat([5,7,2]*pi/180);
x0_seg = [0;0;0;0;0;0;q_ref(2);q_ref(3);q_ref(4);0;0;0];

[u_exo,t_exo,x_exo] = lsim(seg_exo_ss,w_seg,t,x0_seg);


u = K*(x_exo(:,7:12))' - (N*(A-A_r))*((x_exo(:,1:6))'+(x_exo(:,7:12))')-N*E*(w_seg)';
q1 = x_exo(:,1);
q2=x_exo(:,2);
q3=x_exo(:,3);
q0 = sqrt(1 - q1.^2 - q2.^2 - q3.^2);
quat = [q0,q1,q2,q3];
euler = quat2eul(quat)*180/pi;

%% PLOTS
figure
plot(t,x_exo(:,1:3),"LineWidth",1.5)
title("Resposta Dinâmica da Atitude em quaternions",'FontSize',12)
xlabel("Tempo [s]","FontSize",12)
ylabel("Quaternion","FontSize",12)
hold on
plot(t,q_ref(2)*sin(2*t),'b--',"LineWidth",1.75)
plot(t,q_ref(3)*sin(2*t),'r--',"LineWidth",1.75)
plot(t,q_ref(4)*sin(2*t),'y--',"LineWidth",1.75)
legend("$q_1$","$q_2$","$q_3$","$q_1{_{ref}}$","$q_2{_{ref}}$","$q_3{_{ref}}$","Interpreter",'latex','FontSize',12)

grid on
figure
plot(t,u(1:3,:),"LineWidth",1.5)
title("Entradas de Torque pelas Rodas de Reação",'FontSize',12)
xlabel("Tempo [s]","FontSize",12)
ylabel("Torque [N.m]","FontSize",12)
legend("$\tau_x$","$\tau_y$","$\tau_z$","Interpreter",'latex','FontSize',12)
grid on
figure
plot(t,euler(:,1:3),"LineWidth",1.5)
title("Resposta Dinâmica da Atitude em Ângulos de Euler (ZYX)",'FontSize',12)
xlabel("Tempo [s]","FontSize",12)
ylabel("Ângulo de Euler [º]","FontSize",12)
hold on
plot(t,5*exp(-zeta*t).*cos(ws*t),'b--',"LineWidth",1.75)
plot(t,7*exp(-zeta*t).*cos(ws*t),'r--',"LineWidth",1.75)
plot(t,2*exp(-zeta*t).*cos(ws*t),'y--',"LineWidth",1.75)
grid on

legend("$\phi$","$\theta$","$\psi$","$\phi_{ref}$","$\theta_{ref}$","$\psi_{ref}$","Interpreter",'latex','FontSize',12)
