%% Compensador de avanço

G_lead =tf([19.43,4.66],[1,1.168]);
K_l0 = 37.248;
L_0 = tf([0,0,37.2428000000000,44.1077252817023,199.004232783142,56.5658583247018,49.9538633901049],[1,1.19999999999997,6.06999999999997,-2.30399999999996,-3.20259999999930,-2.76339999999981,0])/K_l0;

L_1 = G_lead*L_0;

figure(1)
margin(L_0)
figure(2)
margin(L_1)

%% Compensador de atras-avanço

G_ll = tf([19.43,5.638,0.2333],[1,1.173,0.005839]);
L_2 = L_0*G_ll;

figure(3)
margin(L_2)

%% Malha Fechada - Avanço

G_cl_y = feedback(G_lead*L_0,1);

bode(G_cl_y)
grid on




