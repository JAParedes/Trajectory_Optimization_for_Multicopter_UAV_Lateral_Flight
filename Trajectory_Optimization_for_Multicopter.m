% Program performs time-optimal trajectory planning for multicopter lateral flight.

% The multicopter dynamics do not consider rotational dynamics.

% The program sets up a nonlinear program (NLP) and solves it using CasADi

% Scenario 1: Multicopter vertical motion and flip
% Scenario 2: Multicopter horizontal motion and flip

% Scenarios are chosen depending on final conditions, set with the
% "scenario" variable in line 20

close all
clear all
clc

import casadi.*

%% Scenario choice
scenario = 1; % 1: Scenario 1, 2: Scenario 2

%% Save log of results option
save_log = 0; % 0: Save results in .mat file, 1: Do not save results in .mat file.

%% Setup

% Set discretization variables
T = MX.sym('T'); % Final time variable
N = 200; % Number of discrete-time iteration steps considered, increase if solution is infeasible
dt = T/N; % time step

% Declare model variables
nx = 5;
x1 = SX.sym('x1');  % Horizontal position x
x2 = SX.sym('x2');  % Horizontal velocity \dot{x}
x3 = SX.sym('x3');  % Vertical position z
x4 = SX.sym('x4');  % Vertical velocity \dot{z}
x5 = SX.sym('x5');  % Pitch angle \theta
x = [x1; x2; x3; x4; x5];

nu = 2;
u1 = SX.sym('u1');  % Normalized thrust control input u_T
u2 = SX.sym('u2');  % Normalized rotational control input u_R
u = [u1; u2];

g = 9.81;           % Gravitational acceleration constant

% Model equations
xdot = multicopter_model(x, u, g);

% Continuous time dynamics
f = Function('f', {x, u}, {xdot});

% Initial conditions
x0 = 0;
vx0 = 0;
z0 = 0;
vz0 = 0;
theta0 = 0;

Xinit = [x0; vx0; z0; vz0; theta0];

% Final conditions

if scenario == 1

% Scenario 1: Multicopter vertical motion and flip
    xf = 0;
    vxf = 0;
    zf = 2.7;
    vzf = 0;
    thetaf = 2*pi;

elseif scenario == 2

    % Scenario 2: Multicopter horizontal motion and flip
    xf = 12;
    vxf = 0;
    zf = 0;
    vzf = 0;
    thetaf = 2*pi;

end

Xfinal = [xf; vxf; zf; vzf; thetaf];

% Constraints
uTmax = 20;
uTmin = 1;
uRmax = 10;
uTmid = (uTmax+uTmin)/2; %Used for optimization problem initialization

% Start with an empty NLP
w={};       % Optimization vector w
w0 = [];    % Initial guess of optimization vector solution w_0 
lbw = [];   % Lower bounds of optimization vector
ubw = [];   % Upper bounds of optimization vector
J = 0;      % Cost function J
% Note: The default nonlinear programming problem formulation in CasADi
% allows the user to set lower and upper constraints to a nonlinear
% function of its optimization vector. Since we want to use equality
% constraints, we set the lower and upper bounds to 0.
g={};       % Equality constraints
lbg = [];   % Lower equality constraint bound, set to 0.
ubg = [];   % Upper equality constraint bound, set to 0.

% Final time condition
w = {w{:}, T};      % First variable of the optimization vector is the final time.

lbw = [lbw; 0];     % Lower bound of final time is 0
ubw = [ubw; inf];   % Upper bound of final time not specified (infinity).

w0 = [w0;  5];      % Initial guess of the final time.

% Setting initial conditions
Xk = MX.sym('X0', nx);  % Second set of variables are componsed of the initial state conditions.
w = {w{:}, Xk};
lbw = [lbw; Xinit];     % Lower and upper constraints are the same to fix the initial conditions.
ubw = [ubw; Xinit];
w0  = [w0;  Xinit];     % Initial condition guess is set to be the known values.

% Setting up the equality and inequality constraints for the states and
% inputs in the NLP for each discrete-time iteration step k
for k=0:N-1
    % Setting up U_k variable and its lower and upper bounds
    Uk = MX.sym(['U_' num2str(k)], nu);
    w = {w{:}, Uk};
    lbw = [lbw; uTmin; -uRmax];
    ubw = [ubw; uTmax; uRmax];
    w0 = [w0;  uTmid; 0]; % Initial guess of uT at step k is given by uTmid
    
    % Setting up X_{k+1} variable and its lower and upper bounds
    Xkp1 = MX.sym(['X_' num2str(k+1)], nx);
    w = [w, {Xkp1}];
    % No upper and lower bounds are considered for this problem.
    lbw = [lbw; -inf; -inf; -inf; -inf; -inf];
    ubw = [ubw;  inf;  inf;  inf;  inf;  inf];
    w0 = [w0; 0; 0; 0; 0; 0];

    % Setting up equality constraints, given by the discretized system
    % dynamics.
    g = [g, {dt*f(Xk, Uk)+Xk-Xkp1}]; % X_{k+1} = X_k + f(X_k, U_k)
    lbg = [lbg; 0; 0; 0; 0; 0];
    ubg = [ubg; 0; 0; 0; 0; 0];
    
    % Update Xk variable to use in the next iteration step
    Xk = Xkp1;
end

% Setting final conditions

lbw(end-4:end) = Xfinal;  % Lower and upper constraints are the same to fix the final conditions.
ubw(end-4:end) = Xfinal;
w0(end-4:end) = Xfinal;   % Final condition guess is set to be the known values.

%Final state cost (while redundant, prevents the NLP solver from relaxing the final conditions)
Qx = 100*eye(5);          % Terminal cost weights on final state error
Jf = (w{end}-Xfinal).'*Qx*(w{end}-Xfinal);

% Cost function
J = J+T+Jf;

% Gradients of cost function J and constraint nonlinear function g (not used in this example, although may help the solver)
% dJdw = gradient(J,vertcat(w{:}));
% dGdw = jacobian(vertcat(g{:}),vertcat(w{:}));

% Create an NLP solver
prob = struct('f',  J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
solver = nlpsol('solver', 'ipopt', prob);

%% Run solver

sol = solver('x0', w0, ...
            'lbx', lbw, 'ubx', ubw,...
            'lbg', lbg, 'ubg', ubg);
w_opt = full(sol.x);

%% Getting the solution trajectories

T_opt  = w_opt(1);                  % Final time for optimal trajectory
x_opt = w_opt(2:7:end);             % Horizontal position optimal trajectory
vx_opt = w_opt(3:7:end);            % Horizontal velocity optimal trajectory
z_opt = w_opt(4:7:end);             % Vertical position optimal trajectory
vz_opt  = w_opt(5:7:end);           % Vertical velocity optimal trajectory
theta_opt  = w_opt(6:7:end);        % Pitch angle optimal trajectory
uT_opt  = w_opt(7:7:end);           % Normalized thrust control input optimal trajectory
uR_opt  = w_opt(8:7:end);           % Normalized rotstional control input optimal trajectory
tgrid  = linspace(0, T_opt, N+1);   % Time vector for optimal trajectory

ocpSol.t = tgrid';
ocpSol.X = [x_opt vx_opt z_opt vz_opt theta_opt];
ocpSol.U = [[uT_opt; uT_opt(end)] [uR_opt; uR_opt(end)]];

%% Plotting results

figure(1)
set(gcf, 'color', [1 1 1])

plot(tgrid, ocpSol.X(:,1:5),'linewidth',2);
ax = gca;
ax.FontSize = 17; 
set(gca,'TickLabelInterpreter','latex')

grid on
box on

xlabel('$t$ (s)','Interpreter','latex','Fontsize',17);
ylabel('Optimal State Trajectories','fontsize',17,'interpreter','latex');
legend({'$x^*$','$\dot{x}^*$','$z^*$','$\dot{z}^*$','$\theta^*$'},...
    'Location','NorthWest','Interpreter','latex','Fontsize',15);
x_min = min(min(ocpSol.X(:,1:5)));
x_max = max(max(ocpSol.X(:,1:5)));
axis([0,max(tgrid),x_min-0.1*(x_max-x_min),x_max+0.1*(x_max-x_min)])

figure(2)
set(gcf, 'color', [1 1 1])

plot(tgrid, ocpSol.U(:,1:2),'linewidth',2); 

ax = gca;
ax.FontSize = 17; 
set(gca,'TickLabelInterpreter','latex')

grid on
box on

xlabel('$t$ (s)','Interpreter','latex','Fontsize',15);
ylabel('Optimal Input Trajectories','fontsize',17,'interpreter','latex');
legend({'$u_{\rm T}^*$','$u_{\rm R}^*$'},...
    'Location','NorthWest','Interpreter','latex','Fontsize',15);
u_min = min(min(ocpSol.U(:,1:2)));
u_max = max(max(ocpSol.U(:,1:2)));
axis([0,max(tgrid),u_min-0.1*(u_max-u_min),u_max+0.1*(u_max-u_min)]);

%% Saving for animation

if save_log == 1

    XX = ocpSol.X(:,1:5).';
    UU = ocpSol.U(:,1:2).';
    tt = tgrid;
    px = XX(1,:);
    vx = XX(2,:);
    pz = XX(3,:);
    vz = XX(4,:);
    theta = XX(5,:);
    uT = UU(1,:);
    uR = UU(2,:);
    
    if scenario == 1
        save('Multicopter_vertical_motion_and_flip.mat','tt','px','vx','pz','vz','theta','uT','uR')
    elseif scenario == 2
        save('Multicopter_horizontal_motion_and_flip.mat','tt','px','vx','pz','vz','theta','uT','uR')
    end

end


%% Multicopter Lateral Flight Model
function [dxdt] = multicopter_model(x, u, g)

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