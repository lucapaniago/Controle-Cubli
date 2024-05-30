clc
clear all

A = [0., 0., 0., 0.5, 0., 0.;
 0., 0., 0., 0., 0.5, 0.;
 0., 0., 0., 0., 0., 0.5;
 209.298, 0., 0., -0.00129272, 0., 0.;
 0., 209.298, 0., 0., -0.00129272, 0.;
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
tf_q = tf(sys_cont);
G_p = tf_q(2,2);
p_q2 = pole(G_p);


%% ZN Method

% Kc = sym("Kc");
% s = sym("s");
% 
% p_CL = vpa(Kc * poly2sym(tf_q2.Numerator{1,1},s)+poly2sym(tf_q2.Denominator{1,1},s));
% p_CL_coef=flip(coeffs(p_CL,s));
% rh_CL = routh(p_CL_coef,eps);
% S = solve(rh_CL(3,1)==0,Kc);
% 
% 
% Ku = 2.7572406252264238100898640194421;
% 
% tf_CL=feedback(tf_q2*Ku,1);
% pole_cl=pole(tf_CL);
% routh(vpa(tf_CL.Denominator{1,1}),eps)
% 
% Pu = 0;
% 
% Kc_ZN = 2*Ku;
% t_i = Pu/2;
% t_d = Pu/8; 
% N_d = 10000;
% G_c_ZN = Kc_ZN;
% 
% FT_CL=feedback(G_c_ZN*tf_q2,1);
% pole(FT_CL)

%% ITAE

initial_params = [2.5, 0.8, 1.4]; % Parametros iniciais para processo de minimização
options = optimoptions('fmincon','MaxFunctionEvaluations',3e+6,'MaxIterations',2e+6         );
result = fmincon(@ITAE,initial_params,[],[],[],[],[],[],[],options);
%Parametros PID
Kp = result(1); 
Ki = result(2);
Kd = result(3);

s = tf([1,0],[1]);
t_d = Kd/Kp;
N=10000; 
G_c = Ki/s + Kp*(1+(t_d*s)/((t_d*s)/N+1)); %Funcao de transferencia do PID
T = feedback(G_c*G_p,1); %Funcao de transferencia ref-saida em malha fechada
p = pole(T);%Polos em malha fechada sem filtro
G_f = tf([Ki],[Kd,Kp,Ki]); %Funcao de transferencia do filtro (pre-compensador)
T_f = series(G_f,T); %Funcao de transferencia do sistema em malha fechada com o filtro
p_f = pole(T_f);%Polos em malha fechada com o filtro

Frt = feedback(G_c,G_p,-1); %Funcao de transferencia entre entrada de controle e referencia
Frt_f = series(G_f,Frt);% Com filtro

ts = linspace(0,25,1000);

[q2,t1] = step(T,ts);
[u,t2] = step(Frt,ts);
[q2f,t1f] = step(T_f,ts);
[uf,t2f] = step(Frt_f,ts);
q2_ref = 0.1;
figure(1)
plot(t1,q2,"LineWidth",1.5)
hold on
plot(t1f,q2f,"LineWidth",1.5)
title("Resposta a um sinal degrau - Quaternion $q_2$", 'Interpreter','Latex','FontSize',13)
ylabel("$q_2/q_{2ref}$",'Interpreter','Latex','FontSize',15)
xlabel("Tempo [s]",'FontSize',13,'Interpreter','Latex')
legend("Sem pré-compensador","Com pré-compensador",'FontSize',12)

figure(2)

plot(t1(10:end,1),u(10:end,1),"LineWidth",1.5)
hold on
plot(t1f(10:end,1),uf(10:end,1),"LineWidth",1.5)
title("Entradas de controle $\tau_y$", 'Interpreter','Latex','FontSize',13)
ylabel("$\tau_y/q_{2ref}$",'Interpreter','Latex','FontSize',15)
xlabel("Tempo [s]",'FontSize',13,'Interpreter','Latex')
legend("Sem pré-compensador","Com pré-compensador",'FontSize',12)

figure(3)

plot(t1,q2_ref*q2,"LineWidth",1.5)
hold on
plot(t1f,q2_ref*q2f,"LineWidth",1.5)
title("Resposta a um sinal degrau $q_{2ref} = 0.2$ ", 'Interpreter','Latex','FontSize',13)
ylabel("$q_2$",'Interpreter','Latex','FontSize',15)
xlabel("Tempo [s]",'FontSize',13,'Interpreter','Latex')
legend("Sem pré-compensador","Com pré-compensador",'FontSize',12)

figure(4)
plot(t1(10:end,1),q2_ref*u(10:end,1),"LineWidth",1.5)
hold on
plot(t1f(10:end,1),q2_ref*uf(10:end,1),"LineWidth",1.5)
title("Entradas de controle $\tau_y$", 'Interpreter','Latex','FontSize',13)
ylabel("$\tau_y$",'Interpreter','Latex','FontSize',15)
xlabel("Tempo [s]",'FontSize',13,'Interpreter','Latex')
legend("Sem pré-compensador","Com pré-compensador",'FontSize',12)
