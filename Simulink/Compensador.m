%% Compensador de avanço

G_lead =tf([19.43,4.66],[1,1.168]);
K_l0 = 37.248;
L_0 = tf([0,0,37.2428000000000,44.1077252817023,199.004232783142,56.5658583247018,49.9538633901049],[1,1.19999999999997,6.06999999999997,-2.30399999999996,-3.20259999999930,-2.76339999999981,0])/K_l0;

L_1 = G_lead*L_0;

figure(1)
margin(L_0)
figure(2)
margin(L_1,opts)

%% Compensador de atraso-avanço

G_lag = tf([3.982,1],[7.841,1]);
L_2 = L_0*G_lag*G_lead;
opts = bodeoptions;
opts.YLabel.FontSize = 11;
opts.YLabel.FontSize = 11;
opts.TickLabel.FontSize = 10;
opts.FreqUnits = 'rad/s';
opts.XLim = [0.1,10];
opts.Grid = 'on';
opts.Title.FontSize = 11;

figure(3)
margin(L_2,opts)

%% Malha Fechada - Avanço

G_cl_y = feedback(G_lead*L_0,1);

bode(G_cl_y,opts)
grid on




