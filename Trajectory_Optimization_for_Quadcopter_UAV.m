% CASADI Program for optimal-time control of quadcopter UAV lateral flight.
% Scenario 1: Quadcopter UAV vertical motion and flip
% Scenario 2: Quadcopter UAV horizontal motion and flip
% Scenarios are chosen depending on final conditions, set in line 51.

close all
clear all
clc

import casadi.*

%% Free time choice
fix_tf = 0; % 0: final time free, 1: fixed final time

%% Setup

% Set discretization variables
T = MX.sym('T'); % Time horizon
N = 200; % number of control intervals, change if solution is unfeasible
dt = T/N; % time step

% Declare model variables
nx = 5;
x1 = SX.sym('x1');
x2 = SX.sym('x2');
x3 = SX.sym('x3');
x4 = SX.sym('x4');
x5 = SX.sym('x5');
x = [x1; x2; x3; x4; x5];

nu = 2;
u1 = SX.sym('u1');
u2 = SX.sym('u2');
u = [u1; u2];

g = 9.81;

% Model equations
xdot = quad_model(x, u, g);

% Continuous time dynamics
f = Function('f', {x, u}, {xdot});

% Initial conditions
x0 = 0;
vx0 = 0;
z0 = 0;
vz0 = 0;
theta0 = 0;

% Terminal conditions

% Scenario 1: Quadcopter UAV vertical motion and flip
xf = 0;
vxf = 0;
zf = 3;
vzf = 0;
thetaf = 2*pi;

% Scenario 2: Quadcopter UAV horizontal motion and flip
% xf = 12;
% vxf = 0;
% zf = 0;
% vzf = 0;
% thetaf = 2*pi;

Xfinal = [xf; vxf; zf; vzf; thetaf];

% Constraints
uTmax = 20;
uTmin = 1;
uTmid = (uTmax+uTmin)/2;
uRmax = 10;
tf = 5;

% Start with an empty NLP
w={};
w0 = [];
lbw = [];
ubw = [];
J = 0;
g={};
lbg = [];
ubg = [];

% Final time condition
w = {w{:}, T};

if fix_tf == 0
    lbw = [lbw; 0];
    ubw = [ubw; inf];
else
    lbw = [lbw; tf];
    ubw = [ubw; tf];
end
w0 = [w0;  1.2];


% Set initial conditions
Xk = MX.sym('X0', nx);
w = {w{:}, Xk};
lbw = [lbw; x0; vx0; z0; vz0; theta0];
ubw = [ubw; x0; vx0; z0; vz0; theta0];
w0  = [w0;  x0; vx0; z0; vz0; theta0];

% Formulate the NLP
for k=0:N-1
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)], nu);
    w = {w{:}, Uk};
    lbw = [lbw; uTmin; -uRmax];
    ubw = [ubw; uTmax; uRmax];
    w0 = [w0;  uTmid; 0];
    
    % New NLP variable for state at end of interval
    Xkp1 = MX.sym(['X_' num2str(k+1)], nx);
    w = [w, {Xkp1}];
    lbw = [lbw; -inf; -inf;  0; -inf; -inf];
    ubw = [ubw;  inf;  inf;inf;  inf;  inf];
    w0 = [w0; 0; 0; 0; 0; 0];

    % Add equality constraint
    g = [g, {dt*f(Xk, Uk)+Xk-Xkp1}];
    lbg = [lbg; 0; 0; 0; 0; 0];
    ubg = [ubg; 0; 0; 0; 0; 0];
    
    % update X_{k+1}
    Xk = Xkp1;
end

% Set terminal conditions
lbw(end-4) = xf;
lbw(end-3) = vxf;
lbw(end-2) = zf;
lbw(end-1) = vzf;
lbw(end) = thetaf;

ubw(end-3) = xf;
ubw(end-3) = vxf;
ubw(end-2) = zf;
ubw(end-1) = vzf;
ubw(end) = thetaf;

w0(end-4) = xf;
w0(end-3) = vxf;
w0(end-2) = zf;
w0(end-1) = vzf;
w0(end) = thetaf;

%Final Cost function
Qf = 100*eye(5);
Jf = (w{end}-Xfinal).'*Qf*(w{end}-Xfinal);

% Cost function
J = J+T+Jf;

% Supply gradient (not used in this example)
dJdw = gradient(J,vertcat(w{:}));
dGdw = jacobian(vertcat(g{:}),vertcat(w{:}));

% Create an NLP solver
prob = struct('f',  J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
solver = nlpsol('solver', 'ipopt', prob);

%% Run solver

sol = solver('x0', w0, ...
            'lbx', lbw, 'ubx', ubw,...
            'lbg', lbg, 'ubg', ubg);
w_opt = full(sol.x);

%% Arranging the solution

T_opt  = w_opt(1);
x1_opt = w_opt(2:7:end);
x2_opt = w_opt(3:7:end);
x3_opt = w_opt(4:7:end);
x4_opt  = w_opt(5:7:end);
x5_opt  = w_opt(6:7:end);
u1_opt  = w_opt(7:7:end);
u2_opt  = w_opt(8:7:end);
tgrid  = linspace(0, T_opt, N+1);

ocpSol.t = tgrid';
ocpSol.X = [x1_opt x2_opt x3_opt x4_opt x5_opt];
ocpSol.U = [[u1_opt; u1_opt(end)] [u2_opt; u2_opt(end)]];

%% Plotting results

figure(1)
set(gcf, 'color', [1 1 1])

plot(tgrid, ocpSol.X(:,1:5),'linewidth',2);
ax = gca;
ax.FontSize = 17; 
set(gca,'TickLabelInterpreter','latex')

grid on
box on

xlabel('$t$','fontsize',17,'interpreter','latex');
ylabel('States','fontsize',17,'interpreter','latex');
legend({'$x$','$\dot{x}$','$z$','$\dot{z}$','$\theta$'},'fontsize',15,'interpreter','latex');
xlim([0,max(tgrid)])

figure(2)
set(gcf, 'color', [1 1 1])

plot(tgrid, ocpSol.U(:,2),'linewidth',2); 
hold on
plot(tgrid, ocpSol.U(:,1),'linewidth',2); 
hold off

ax = gca;
ax.FontSize = 17; 
set(gca,'TickLabelInterpreter','latex')

grid on
box on

xlabel('$t$','fontsize',15,'interpreter','latex');
ylabel('Inputs','fontsize',17,'interpreter','latex');
legend({'$\omega$','$F_{\rm T}/m$'},'fontsize',15,'interpreter','latex');
xlim([0,max(tgrid)])

%% Saving for animation (uncomment)

% XX = ocpSol.X(:,1:5).';
% UU = ocpSol.U(:,1:2).';
% tt = tgrid;
% px = XX(1,:);
% vx = XX(2,:);
% pz = XX(3,:);
% vz = XX(4,:);
% ang = XX(5,:);
% omega = UU(2,:);
% F_T = UU(1,:);
% 
% save('Quadcopter_UAV_vertical_motion_and_flip.mat','tt','px','vx','pz','vz','ang','omega','F_T')
% save('Quadcopter_UAV_horizontal_motion_and_flip.mat','tt','px','vx','pz','vz','ang','omega','F_T')


%% Quadcopter Lateral Flight Model
function [dxdt] = quad_model(x, u, g)

xx = x(1);
vx = x(2);
zz = x(3);
vz = x(4);
theta = x(5);

uT = u(1);
uR = u(2);

dxdt1 = vx;
dxdt2 = uT*sin(theta);
dxdt3 = vz;
dxdt4 = uT*cos(theta) - g;
dxdt5 = uR;

dxdt = [dxdt1; dxdt2; dxdt3; dxdt4; dxdt5];

end