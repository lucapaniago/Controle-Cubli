%% Intro
clc
clear all
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
p = [2,-1-5j,-2.5,-0.5+0.5j,-0.5-0.5j,-1+5j];
K = place(A,B,p);
A_bar = A - B*K;
sys_cont = ss(A_bar,B,C,D);
tf_q = tf(sys_cont);
G_p = tf_q(1,1);
p_gp = pole(sys_cont);

rlocus(G_p)
%% ZN Method
ts=linspace(0,150,1000);
Kc = sym("Kc");
s = sym("s");
p_Kc = vpa(Kc * poly2sym(G_p.Numerator{1,1},s)+poly2sym(G_p.Denominator{1,1},s));
p_Kc_coef=flip(coeffs(p_Kc,s));
rh_Kc = routh(p_Kc_coef,eps);
S = solve(rh_Kc(6,1)==0,Kc);

Ku = 0.90337226204908848270845329711761;
tf_CL_crt=feedback(Ku*G_p,1);
pole_cl_crt=pole(tf_CL_crt);
[q_crt,t_crt]=step(tf_CL_crt,linspace(0,50,2000));
routh(vpa(tf_CL_crt.Denominator{1,1}),eps);
Pu =2*pi/imag(pole_cl_crt(3));



Kc_ZN = 0.6*Ku;
t_i = Pu/2;
t_d = Pu/8; 
N_d = 100;
G_c_ZN = Kc_ZN*(1+tf([1],[t_i,0]) + tf([t_d,0],[t_d/N_d,1]));
FT_CL=feedback(G_c_ZN*G_p,1);
pole_cl = pole(FT_CL);
[q_zn,t] = step(FT_CL,ts);

Frt = feedback(G_c_ZN,G_p,-1); %Funcao de transferencia entre entrada de controle e referencia
[u_zn,t] = step(Frt,ts);
q1_ref = 0.1;
figure(1)
plot(t,q_zn,"LineWidth",1.5)
title("Resposta a um sinal degrau - Quaternion $q_1$", 'Interpreter','Latex','FontSize',13)
ylabel("$q_1/q_{1ref}$",'Interpreter','Latex','FontSize',15)
xlabel("Tempo [s]",'FontSize',13,'Interpreter','Latex')

figure(2)
plot(t,q1_ref*q_zn,"LineWidth",1.5)
title("Resposta a um sinal degrau $q_{1ref} = 0.1$ ", 'Interpreter','Latex','FontSize',13)
ylabel("$q_1$",'Interpreter','Latex','FontSize',15)
xlabel("Tempo [s]",'FontSize',13,'Interpreter','Latex')

figure(3)

plot(t(10:end,1),u_zn(10:end,1),"LineWidth",1.5)
title("Entradas de controle $\tau_x$", 'Interpreter','Latex','FontSize',13)
ylabel("$\tau_x/q_{1ref}$",'Interpreter','Latex','FontSize',15)
xlabel("Tempo [s]",'FontSize',13,'Interpreter','Latex')

figure(4)

plot(t(10:end,1),q1_ref*u_zn(10:end,1),"LineWidth",1.5)
title("Entradas de controle $\tau_x$", 'Interpreter','Latex','FontSize',13)
ylabel("$\tau_x$",'Interpreter','Latex','FontSize',15)
xlabel("Tempo [s]",'FontSize',13,'Interpreter','Latex')

figure(5)

bode(FT_CL)

figure(6)
subplot(1,2,1)
plot(t_crt,q_crt,"LineWidth",1.5)
title("Resposta Harmonica crítica para referência degrau - Quaternion $q_1$", 'Interpreter','Latex','FontSize',13)
ylabel("$q_1/q_{1ref}$",'Interpreter','Latex','FontSize',15)
xlabel("Tempo [s]",'FontSize',13,'Interpreter','Latex')
subplot(1,2,2)
pzplot(tf_CL_crt)

