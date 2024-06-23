%% PID via Root Locus
K_l0 = 37.248;
L_0 = tf([0,0,37.2428000000000,44.1077252817023,199.004232783142,56.5658583247018,49.9538633901049],[1,1.19999999999997,6.06999999999997,-2.30399999999996,-3.20259999999930,-2.76339999999981,0])/K_l0;

G_c_PI = tf([91,14],[6.5,0]);

G_c_PID = tf([1.081e5,1.17e5,1.8e4],[6,6500,0]);

G_c_PD = tf([6160,1e+4],[0.6154,1000]);

T_pi = feedback(G_c_PI*L_0,1);
T_pd = feedback(G_c_PD*L_0,1);
T_pid = feedback(G_c_PID*L_0,1);

opts = bodeoptions;
opts.YLabel.FontSize = 11;
opts.YLabel.FontSize = 11;
opts.TickLabel.FontSize = 10;
opts.FreqUnits = 'rad/s';
opts.XLim = [0.1,20];
opts.Grid = 'on';
opts.Title.FontSize = 11;



% Bode Malha Aberta
figure
opts.Title.String = " Margens de Estabilidade PI";
margin(G_c_PI*L_0,opts)
figure
opts.Title.String = " Margens de Estabilidade PD";
margin(G_c_PD*L_0,opts)
figure
opts.Title.String = " Margens de Estabilidade PID";

margin(G_c_PID*L_0,opts)



%% Bode Malha Fechada
% figure
% bode(T_pi,opts)
% hold on
% bode(T_pd,opts)
% bode(T_pid,opts)
% legend("PI","PD","PID");
% grid on
% bd_pi = bandwidth(T_pi);
% bd_pd = bandwidth(T_pd);
% bd_pid = bandwidth(T_pid);





