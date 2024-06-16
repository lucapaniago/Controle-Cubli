%% Intro
A =[0	0	0	0.5	0	0;
0	0	0	0	0.5	0;
0	0	0	0	0	0.5;
134.697	4.61732	0	-0.00126849	-0.0000419712	-0.0000859303;
4.61732	129.366	0	-0.0000419712	-0.00122003	0.000148836;
2.59264	-4.49059	0	-0.0000859303	0.000148836	-0.00617827];

B=[0	0	0;
0	0	0;
0	0	0;
74.4856	2.46455	5.04582;
2.46455	71.6398	-8.73961;
5.04582	-8.73961	362.787];

C = [1	0	0	0	0	0;
    0	1	0	0	0	0;
    0	0	1	0	0	0];
D = zeros(3,3);
E = B;
p = [1,-0.7-2.5j,0,-0.4+0.5j,-0.4-0.5j,-0.7+2.5j];
K = place(A,B,p);
A_bar = A - B*K;
sys_cont = ss(A_bar,B,C,D);
tf_q = tf(sys_cont);
G_p = tf_q(1,1);
p_gp = pole(sys_cont);

%% ITAE

initial_params = [1,0.7,1.5]; % Parametros iniciais para processo de minimização
options = optimoptions('fmincon','MaxFunctionEvaluations',7e+26,'MaxIterations',8e+26,'StepTolerance',1e-18);
result = fmincon(@ITAE,initial_params,[],[],[],[],[],[],[],options);
%Parametros PID
Kp = result(1); 
Ki = result(2);
Kd = result(3);

s = tf([1,0],[1]);
t_d = Kd/Kp;
N=100; 
G_c = Ki/s + Kp*(1+(t_d*s)/((t_d*s)/N+1)); %Funcao de transferencia do PID
T = feedback(G_c*G_p,1); %Funcao de transferencia ref-saida em malha fechada
p = pole(T);%Polos em malha fechada sem filtro
G_f = tf([Ki],[Kd,Kp,Ki]); %Funcao de transferencia do filtro (pre-compensador)
T_f = series(G_f,T); %Funcao de transferencia do sistema em malha fechada com o filtro
p_f = pole(T_f);%Polos em malha fechada com o filtro

Frt = feedback(G_c,G_p,-1); %Funcao de transferencia entre entrada de controle e referencia
Frt_f = series(G_f,Frt);% Com filtro

ts = linspace(0,35,1000);

[q1,t1] = step(T,ts);
[u,t2] = step(Frt,ts);
[q1f,t1f] = step(T_f,ts);
[uf,t2f] = step(Frt_f,ts);
q1_ref = 0.1;
figure(1)
plot(t1,q1,"LineWidth",1.5)
hold on
plot(t1f,q1f,"LineWidth",1.5)
title("Resposta a um sinal degrau - Quaternion $q_1$", 'Interpreter','Latex','FontSize',13)
ylabel("$q_1/q_{1ref}$",'Interpreter','Latex','FontSize',15)
xlabel("Tempo [s]",'FontSize',13,'Interpreter','Latex')
legend("Sem pré-compensador","Com pré-compensador",'FontSize',12)

figure(2)

plot(t1(10:end,1),u(10:end,1),"LineWidth",1.5)
hold on
plot(t1f(10:end,1),uf(10:end,1),"LineWidth",1.5)
title("Entradas de controle $\tau_x$", 'Interpreter','Latex','FontSize',13)
ylabel("$\tau_x/q_{1ref}$",'Interpreter','Latex','FontSize',15)
xlabel("Tempo [s]",'FontSize',13,'Interpreter','Latex')
legend("Sem pré-compensador","Com pré-compensador",'FontSize',12)
%Supondo referência de q_2 = 0.1
figure(3)

plot(t1,q1_ref*q1,"LineWidth",1.5)
hold on
plot(t1f,q1_ref*q1f,"LineWidth",1.5)
title("Resposta a um sinal degrau $q_{1ref} = 0.1$ ", 'Interpreter','Latex','FontSize',13)
ylabel("$q_1$",'Interpreter','Latex','FontSize',15)
xlabel("Tempo [s]",'FontSize',13,'Interpreter','Latex')
legend("Sem pré-compensador","Com pré-compensador",'FontSize',12)

figure(4)
plot(t1(10:end,1),q1_ref*u(10:end,1),"LineWidth",1.5)
hold on
plot(t1f(10:end,1),q1_ref*uf(10:end,1),"LineWidth",1.5)
title("Entradas de controle $\tau_x$", 'Interpreter','Latex','FontSize',13)
ylabel("$\tau_$x",'Interpreter','Latex','FontSize',15)
xlabel("Tempo [s]",'FontSize',13,'Interpreter','Latex')
legend("Sem pré-compensador","Com pré-compensador",'FontSize',12)

figure(5)
bode(T)
hold on
bode(T_f)

figure(6)
bode(Frt)
hold on
bode(Frt_f)
