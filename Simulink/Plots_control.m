%% Plot da resposta dinâmica controlada linear

figure(1)
plot(t,y(:,1:3),"LineWidth",1.5)
title("Resposta Dinâmica da Atitude em quaternions",'FontSize',12)
xlabel("Tempo [s]","FontSize",12)
ylabel("Quaternion","FontSize",12)
legend("$q_1$","$q_2$","$q_3$","Interpreter",'latex','FontSize',12)
figure(2)
plot(t,x(:,4:6),"LineWidth",1.5)
title("Resposta Dinâmica das Velocidades Angulares",'FontSize',12)
xlabel("Tempo [s]","FontSize",12)
ylabel("Velocidadas Angulares [rad/s]","FontSize",12)
legend("$\omega_x$","$\omega_y$","$\omega_z$","Interpreter",'latex','FontSize',12)
figure(3)

plot(t,u(1:3,:),"LineWidth",1.5)
title("Entradas de Torque pelas Rodas de Reação",'FontSize',12)
xlabel("Tempo [s]","FontSize",12)
ylabel("Torque [N.m]","FontSize",12)
legend("$\tau_x$","$\tau_y$","$\tau_z$","Interpreter",'latex','FontSize',12)

figure(4)
plot(t,euler(:,1:3),"LineWidth",1.5)
title("Resposta Dinâmica da Atitude em Ângulos de Euler (XYZ)",'FontSize',12)
xlabel("Tempo [s]","FontSize",12)
ylabel("Ângulo de Euler","FontSize",12)
legend("$\phi$","$\theta$","$\psi$","Interpreter",'latex','FontSize',12)

%% Plot da resposta dinâmica controlada não linear