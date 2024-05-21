%% Plot da resposta dinâmica controlada linear com observador de estado

figure(1)
plot(t,w_x,t,w_x_hat,"LineWidth",1.5)
title("Resposta Dinâmica do Observador - \omega_x",'FontSize',12)
xlabel("Tempo [s]","FontSize",12)
ylabel("Velocidade Angular [rad/s]","FontSize",12)
legend("$\omega_x$","$\hat{\omega}_x$","Interpreter",'latex','FontSize',12)
grid on
saveas(gcf,'wx.png')

figure(2)
plot(t,w_y,t,w_y_hat,"LineWidth",1.5)
title("Resposta Dinâmica do Observador - \omega_y",'FontSize',12)
xlabel("Tempo [s]","FontSize",12)
ylabel("Velocidade Angular [rad/s]","FontSize",12)
legend("$\omega_y$","$\hat{\omega}_y$","Interpreter",'latex','FontSize',12)
grid on
saveas(gcf,'wy.png')
figure(3)
plot(t,w_z,t,w_z_hat,"LineWidth",1.5)
title("Resposta Dinâmica do Observador - \omega_z",'FontSize',12)
xlabel("Tempo [s]","FontSize",12)
ylabel("Velocidade Angular [rad/s]","FontSize",12)
legend("$\omega_z$","$\hat{\omega}_z$","Interpreter",'latex','FontSize',12)
grid on
saveas(gcf,'wz.png')

figure(4)
plot(t,u_obs(:,1:3),"LineWidth",1.5)
title("Entradas de Torque pelas Rodas de Reação",'FontSize',12)
xlabel("Tempo [s]","FontSize",12)
ylabel("Torque [N.m]","FontSize",12)
legend("$\tau_x$","$\tau_y$","$\tau_z$","Interpreter",'latex','FontSize',12)
grid on
saveas(gcf,'Torque.png')


figure(5)
plot(t,x_z(:,1:3),"LineWidth",1.5)
title("Resposta Dinâmica da Atitude em quaternions",'FontSize',12)
xlabel("Tempo [s]","FontSize",12)
ylabel("Quaternion","FontSize",12)
legend("$q_1$","$q_2$","$q_3$","Interpreter",'latex','FontSize',12)
grid on
saveas(gcf,'Quaternion.png')

figure(6)
plot(t,x_z(:,4:6),"LineWidth",1.5)
title("Resposta Dinâmica das Velocidades Angulares",'FontSize',12)
xlabel("Tempo [s]","FontSize",12)
ylabel("Velocidadas Angulares [rad/s]","FontSize",12)
legend("$\omega_x$","$\omega_y$","$\omega_z$","Interpreter",'latex','FontSize',12)
grid on
saveas(gcf,'VelAng.png')


figure(7)
plot(t,euler(:,1:3),"LineWidth",1.5)
title("Resposta Dinâmica da Atitude em Ângulos de Euler (ZYX)",'FontSize',12)
xlabel("Tempo [s]","FontSize",12)
ylabel("Ângulo de Euler [º]","FontSize",12)
legend("$\phi$","$\theta$","$\psi$","Interpreter",'latex','FontSize',12)
grid on
saveas(gcf,'EulAng.png')

figure(8)
plot(t,w_x-w_x_hat,t,w_y-w_y_hat,t,w_z-w_z_hat,"LineWidth",1.5)
title("Erros de estimação das velocidades angulares",'FontSize',12)
xlabel("Tempo [s]","FontSize",12)
ylabel("Erro das Velocidadas Angulares [rad/s]","FontSize",12)
grid on
legend("$\varepsilon_{\omega x}$","$\varepsilon_{\omega y}$","$\varepsilon_{\omega z}$","Interpreter",'latex','FontSize',14)
saveas(gcf,'erro.png')
%% Plot da resposta dinâmica controlada não linear