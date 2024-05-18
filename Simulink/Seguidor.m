% Seguidor de Referência
t = linspace(0,30,1000);
%% Seguidor LQ
Q_ref =[1,0,0,0,0,0;
    0,01,0,0,0,0;
    0,0,1,0,0,0;
    0,0,0,100,0,0;
    0,0,0,0,100,0;
    0,0,0,0,0,100];
R_ref =1*diag([1,1,1]);


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
plot(t,x_lq(:,3))
hold on
plot(t,x_ref(3,:))
hold off
figure
plot(t,u_lq)
% q0 = sqrt(1-x_lq(:,1).^2 - x_lq(:,2).^2 - x_lq(:,3).^2)';
% euler = quat2eul(q);

%% Seguidor Variável Exógena
%Degrau
A_r = [0,0,0,0.5,0,0;
        0,0,0,0,0.5,0;
        0,0,0,0,0,0.5;
        -10,0,0,0,0,0;
        0,-10,0,0,0,0;
        0,0,-10,0,0,0];

M =[[1,0,0,0,0,0];
    [0,1,0,0,0,0];
    [0,0,1,0,0,0]];

A_cl_inv = inv(A-B*K);
N = inv(M*A_cl_inv*B)*M*A_cl_inv;
G_r = N*(A-A_r);
% w_seg = [(0.3*sin(100*t))';(0.3*sin(100*t))';(0.3*sin(100*t))']';
w_seg = 0.3*ones(1000,3);
A_ex_cl = [[A-B*G_r,B*(K-G_r)];[(A_r - A +B*G_r),(A_r - B*(K-G_r))]];
% E_ex_cl = [E;zeros(6,3)];
E_ex_cl = [-B*N*E + E ; B*N*E-E];
K_ex = [-G_r,K-G_r];

seg_exo_ss = ss(A_ex_cl,E_ex_cl,K_ex,0);

% x0_seg = [0;0;0;0;0;0;q_ref(2);q_ref(3);q_ref(4);0;0;0];
x0_seg = [0;0;0;0;0;0;0.05;0.1;0.01;0;0;0];

[u_exo,t_exo,x_exo] = lsim(seg_exo_ss,w_seg,t,x0_seg);

% plot(t,x_exo(:,1:3))
hold on
% plot(t,x_exo(:,4))
u = K*(x_exo(:,7:12))' - (N*(A-A_r))*((x_exo(:,1:6))'+(x_exo(:,7:12))')-N*E*(w_seg)';
plot(t,u)
q1 = x_exo(:,1);
q2=x_exo(:,2);
q3=x_exo(:,3);
q0 = sqrt(1 - q1.^2 - q2.^2 - q3.^2);
quat = [q0,q1,q2,q3];
euler = quat2eul(quat)*180/pi; %degress
% plot(t,euler)

% plot(t,q_ref(3)*ones(1000,1));





