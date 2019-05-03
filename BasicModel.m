%% Make Basic Spatial V2 Model
clear
clc

addpath( genpath( 'C:\Users\John Admin\Documents\MATLAB\optimization\Project\spatial_v2' ) ); %CHANGE THIS
%Adds the spatial_v2 package to the current file path
%Allows us to make use of the functions inside.
%% Define Model Parameters

massCart = 1; % All Units are SI
massPole = 1;
lengthPole = .5;
gravity = 9.81; %Earth Gravity
modelParams = [massCart, massPole, lengthPole, gravity];

%% Calculate the Dynamics/Kinematics of the System
cartPole = makeModel(modelParams); %Uses Spatial V2 Package to construct the model of the Cart Pole System

% States of the model
%   Location of the Cart, Angle of the Pole (WRT horizontal)
%   And their derivatives

%These states constitute the joint angles and joint rates.
%q = [p; theta];
%qd = [pdot; thetadot]

%State => x = [q;qd]

%qdd is calculated via the current joint rates and applied torque using the
%[F]orward [D]ynamics [A]rticulated [B]ody algorithm  fdAB().  

syms mCart_sym mPole_sym lPole_sym g_sym tau_sym
syms p_sym pdot_sym theta_sym thetadot_sym force_sym;
q_sym = [p_sym; theta_sym];
qd_sym = [pdot_sym; thetadot_sym];
x_sym = [q_sym;qd_sym];
tau_sym = [force_sym; 0];  %Tau is the external force --> It is only applied on the cart
u_sym = force_sym;

%Creates a  cartPole model with entirely symbolic variables
modelParams_sym = [mCart_sym, mPole_sym, lPole_sym, g_sym];
cartPole_sym = makeModel(modelParams_sym);

qdd_sym = FDab(cartPole_sym, q_sym, qd_sym, tau_sym);
%solves the ABA for the symbolic variables.

f_sym = [qd_sym; qdd_sym];  
%f_sym represents the changing dynamics dynamics of the system. 

f_func = matlabFunction(f_sym, 'file','cartPoleDynamics','vars',{x_sym, u_sym, [modelParams_sym]}); 
% matlabFunction() converts a symbolic equation into a function that can be
% called.
%Here, inputting the current state (x), the control to the system (u), and
%the external force (tau) produces the change in state (xd).

%% Select the Control For the System
% The Cart Pole is a non-linear SISO system.
% The control (u) can be determined by a variety of different methods.
% 'nothing' turns the controller off (slight friction at the joints.
% 'PD' is basic PD Control

controlParams.Method = 'MPC';
controlParams.xDesired = [0;pi;0;0];
controlParams.x0 = [0;pi-.2;0;0];     %Initial values of the System states and control


%Constants for PID
controlParams.Constants.Kp = 10;
controlParams.Constants.Kd = 2;
controlParams.Constants.Ki = 0;
%Get value fo the control (force on the cart) from the function


%Constants for MPC
controlParams.Ts = .1;  %The sampling period of the controller
controlParams.Hz = 10;   %The MPC time horizon
controlParams.Q = diag([20,400,30,200]); %The weights on state error and control
controlParams.R = 20;

%% Simulate the System

simulationParams.timeStep = .01;
simulationParams.timeFinal =10;

simulationParams.slow_mo = 1; % "Slowing Factor"
simulationParams.frameRate = 30; %Frames Per Second

simulationParams.animate =0; % 1 = animate.  0 = no animation
simulationParams.animateEnd =1;

%initialize these parts of the structure
[results] = simCartPole(modelParams,controlParams,simulationParams);
%% Plot Received Data

figure(2) % Plot of the States as a function of Time

    plot(results.t_store, results.x_store,'LineWidth',1.5)
    l = legend('$x$','$\theta$','$\dot{x}$','$\dot{\theta}$');
    l.Interpreter = 'Latex';

    xlabel('Time (s)','Interpreter','Latex');
    ylabel('State','Interpreter','Latex')
    set(gca,'FontSize',16)

figure(3) % Plot of the Control Input as a function of time

    plot(results.t_store, results.u_store,'LineWidth',1.5);
    xlabel('Time (s)','Interpreter','Latex');
    ylabel('Force (N)','Interpreter','Latex');

    set(gca,'FontSize',16);
    
return
%%
plotOptSol(1,modelParams,controlParams,simulationParams,results); %Plots one of the collocation created solutions.
return