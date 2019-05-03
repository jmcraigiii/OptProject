function [u_output,results] = getMPC(xSim, modelParams,controlParams,results)
%Uses Casadi to solve the OCP that minimizes running cost given nonlinear
%contraints at every level.
addpath( genpath( 'C:\Users\John Admin\Documents\MATLAB\Add-Ons\casadi-windows-matlabR2016a-v3.4.5') )

import casadi.*


T = controlParams.Hz; % Time horizon
%This is going to be how far into the future the program looks
Ts = controlParams.Ts;  %Sampling time is T/N

N = floor(T/Ts); % number of control intervals
%This essentially discretizes the control.

% Declare model variables
x = SX.sym('x',4); %The four state variables
u = SX.sym('u',1); %The control input

% Model equations
xdot = CartPoleDynamics(x,u,modelParams');
%Put this in with the dynamics of the system coming from the the FDab

x_des = controlParams.xDesired;

%Weighting Constants
    % The objective function has a weighted Quadratic Cost
Q = controlParams.Q;  R = controlParams.R;

% Objective term
    %This is what casadi is going to try to minimize
    % The running cost of the system
J = (x_des'-x')*Q*(x_des-x) + u'*R*u;

% Continuous time dynamics
f = Function('f', {x, u}, {xdot, J});
%Makes a function that combines the state and cost dynamics


% Start with an empty NLP
problem.vars = {};
problem.vars_init = [];
problem.vars_lb = [];
problem.vars_ub = [];

problem.cost = 0;
problem.constraints = {};
problem.constraints_lb = [];
problem.constraints_ub = [];

x_len = length(x);
u_len = length(u);


%"Lift" initial conditions
% "Lift" refers to lifting the NLP into higher dimensions to solve
% simultaneously instead of using single shooting
x0 = xSim; %The MPC starts with the current value of the state in the simulator.

x_inds = [];
u_inds = [];
u_lb = -20; %The force bounds Pat used in the homework
u_ub = 20;
x_lb = -inf*[1 1 1 1]';
x_ub = inf*[1 1 1 1]';

[problem, Uk, u_inds(end+1,:)] = add_var(problem, 'U_0', 1, 0, u_lb, u_ub);
[problem, Xk, x_inds(end+1,:)] = add_var(problem, 'X_0', 4, x0, x0, x0);
[fk, qk] = f(Xk,Uk);

z4 = zeros(4,1);
inf4 = ones(4,1)*inf;

% Formulate the NLP
    % Add in all of the collocation contraints
for k=0:N-1
    Xkprev = Xk;
    fkprev = fk;
    qkprev = qk;
    
    % New NLP variables for the state and control at end of interval
    [problem, Uk, u_inds(end+1,:)] = add_var(problem, ['U_' num2str(k+1)], 1, 0, u_lb , u_ub);
    [problem, Xk, x_inds(end+1,:)] = add_var(problem, ['X_' num2str(k+1)], 4, z4, x_lb , x_ub);
    
    
    [fk, qk] = f(Xk,Uk); %Calculates the change in both the function cost and dynamics
    
    
    %% 
    
    % Trapezoidal integration for cost and state (h = Ts)
    problem.cost  = problem.cost + Ts*(qk + qkprev)/2;
    [problem] = add_constraint(problem, Xk - (Xkprev + Ts*(fkprev + fk)/2 ) , 0*x_lb, 0*x_ub );
    %[problem] = add_constraint(problem, Xk - (Xkprev + Ts*(fkprev)) , 0*x_lb, 0*x_ub );
    %This constraint enforces that each collocation point needs to be
    %coincident
    
end
problem.cost  = problem.cost + (x_des'-Xk')*(100*Q)*(x_des-Xk); % Adds in a final cost
%[problem] = add_constraint(problem, Xk - x_des, z4,z4);

%Above the objective function and all of the contraints were defined. so
%now you can just use one of the casadi solvers to find the solution.
% Create an NLP solver
prob = struct('f', problem.cost, 'x', vertcat(problem.vars{:}), 'g', vertcat(problem.constraints{:}));
opts.print_time = 0; %opts.ipopt.print_level = 1;
solver = nlpsol('solver', 'ipopt', prob, opts);

if controlParams.k >1
    problem.vars_init = results.var_optPrev;
end

% Solve the NLP
sol = solver('x0', problem.vars_init, 'lbx', problem.vars_lb, 'ubx', problem.vars_ub,...
            'lbg', problem.constraints_lb, 'ubg', problem.constraints_ub);


var_opt = full(sol.x);  
        
x_opt = reshape( var_opt(x_inds(:)), N+1, x_len);
u_opt = reshape( var_opt(u_inds(:)), N+1, u_len);


results.x_opt_store{controlParams.k} = x_opt;
results.u_opt_store{controlParams.k} = u_opt;
results.var_optPrev = var_opt;
u_output = u_opt(1);


end % Ends the main function

%% Helper Functions
%These functions take the process of adding more Casadi function and
%constraint variables to the code and makes it more streamlined.

function [problem, var, var_inds] = add_var(problem, name, len, init, lb, ub)
    import casadi.*
    var = MX.sym(name, len);    
    problem.vars = {problem.vars{:}, var};
    problem.vars_lb = [problem.vars_lb; lb];
    problem.vars_ub = [problem.vars_ub; ub ];
    problem.vars_init = [problem.vars_init; init ];
    
    ind_end = length( problem.vars_lb );
    ind_begin = ind_end-len+1;
    var_inds = ind_begin: ind_end;
    %The _inds suffix comes from the indices that are being stored
    %Casadi places all of the optimization variables together, and doesn't
    %really sort out x (state) versus u (control) from the sol.x (solution
    %to all of the problemvariables).
end

function [problem] = add_constraint(problem, constr, lb, ub)
    problem.constraints = {problem.constraints{:}, constr};
    problem.constraint_lb = [problem.constraints_lb; lb];
    problem.constraint_ub = [problem.constraints_ub; ub ];
end