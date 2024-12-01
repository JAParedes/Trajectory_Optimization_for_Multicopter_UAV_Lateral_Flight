% Program plots the optimal state and input trajectories obtained by the
% NLP solvers in "Trajectory_Optimization_for_Multicopter.m" and
% "Trajectory_Optimization_for_Multicopter_w_Rot_Dyn.m"

% Scenario 1: Multicopter vertical motion and flip
% Scenario 2: Multicopter horizontal motion and flip

close all
clear all
clc

rot_dyn = 1;    % 0: Dynamics do not consider rotational dynamics, 1: Dynamics consider rotational dynamics
scenario = 2;   % 1: Scenario 1, 2: Scenario 2

if rot_dyn == 0
    if scenario == 1
        load('Multicopter_vertical_motion_and_flip.mat')
    elseif scenario == 2
        load('Multicopter_horizontal_motion_and_flip.mat')
    end
else
    if scenario == 1
        load('Multicopter_w_Rot_Dyn_vertical_motion_and_flip.mat')
    elseif scenario == 2
        load('Multicopter_w_Rot_Dyn_horizontal_motion_and_flip.mat')
    end
end

figure(1)
set(gcf, 'color', [1 1 1])

plot(tt, px,'linewidth',2);
hold on
plot(tt, vx,'linewidth',2);
plot(tt, pz,'linewidth',2);
plot(tt, vz,'linewidth',2);
plot(tt, theta,'linewidth',2);
if rot_dyn == 1
    plot(tt, thetadot,'linewidth',2);
end
hold off
ax = gca;
ax.FontSize = 18; 
set(gca,'TickLabelInterpreter','latex')

grid on
box on

xlabel('$t$ (s)','Interpreter','latex','Fontsize',17);
ylabel('Optimal State Trajectories','fontsize',17,'interpreter','latex');

if rot_dyn == 0
    legend({'$x^*$','$\dot{x}^*$','$z^*$','$\dot{z}^*$','$\theta^*$'},...
        'Location','NorthWest','Interpreter','latex','Fontsize',15);
    x_min = min([px pz vx vz theta]);
    x_max = max([px pz vx vz theta]);
elseif rot_dyn == 1
    legend({'$x^*$','$\dot{x}^*$','$z^*$','$\dot{z}^*$','$\theta^*$','$\dot{\theta}^*$'},...
        'Location','NorthWest','Interpreter','latex','Fontsize',15);
    x_min = min([px pz vx vz theta thetadot]);
    x_max = max([px pz vx vz theta thetadot]);
end

axis([0,max(tt),x_min-0.1*(x_max-x_min),x_max+0.1*(x_max-x_min)]);


figure(2)

set(gcf, 'color', [1 1 1])

plot(tt, uT,'linewidth',2); 
hold on
plot(tt, uR,'linewidth',2); 
hold off

ax = gca;
ax.FontSize = 18; 
set(gca,'TickLabelInterpreter','latex')

grid on
box on

xlabel('$t$ (s)','Interpreter','latex','Fontsize',15);
ylabel('Optimal Input Trajectories','fontsize',17,'interpreter','latex');
legend({'$u_{\rm T}^*$','$u_{\rm R}^*$'},...
    'Location','NorthWest','Interpreter','latex','Fontsize',15);
u_min = min([uT uR]);
u_max = max([uT uR]);
axis([0,max(tt),u_min-0.1*(u_max-u_min),u_max+0.1*(u_max-u_min)]);