function [results] = simCartPole(modelParams,controlParams,simulationParams)
%Uses HGTransforms and control to calculate and display cartPole motion
%Essentially, a different coordinate system is applied to each of the
%bodies and changing the transformations between those CS simulates
%movement.

%Key Functions
%   makehgtform( inputs ) --> Makes a 4x4 CS transformation matrix
%       which contains rotation and transformation information
%
%   hgtransform() makes a hgTransform of the the current axis.  Change its
%   'Matrix' property to apply a transform.


% Grabs the data for the simulation from the workspace if it isnt included
% in the function.
%if nargin== 0 && ismember('modelParams',evalin('base','who'))
%    modelParams = evalin('base','modelParams');
%    simulationParams = evalin('base','simulationParams');
%    controlParams = evalin('base','controlParams');
%end

%% Set Up the Animation Graphics
if simulationParams.animate==1 || simulationParams.animateEnd ==1
hCart = .2;
wCart = .5;  %Length and width of the cart to be displayed.

lPole = modelParams(3);
%lPole = .75;
wPole = .25;


window = figure(1); % selects the figure(1) window
clf (window)       % Clears the figure

set(window, 'name','Animation Window','NumberTitle','off',...
    'pointer','crosshair','toolbar','none','menubar','none');

track = plot(gca, [-30 30],[0 0],'k','LineWidth',2.5);  % Plots a horizontal line
track.Color = [51, 127, 51]/255; %Changes the Color of the track
%Displays a path for the CartPole to move along.

frame0 = hgtransform(gca); %Sets the first coordinate frame directly at 0,0
frame1 = hgtransform(frame0);
frame2 = hgtransform(frame1);  %Places all of the frames on top of one another
       %also establishes a parent/child relationship
       
cart = fill(frame1,[-1 1 1 -1 -1]*wCart/2, [-1 -1 1 1 -1]*hCart/2,'r');
cart.LineStyle = 'none';
cart.FaceColor = [73, 163, 240]/255;

pole = plot(frame2,[0 0], [0 -lPole] ,'b','LineWidth',5);
pole.Color = [202, 31, 123]/255;
% draws the cart and pole objects on the graph
h_title = title('t=0');


axis equal
xlim([-5,5])
ylim([-2*lPole 2*lPole]);
if abs(ylim) < 5
    ylim([-2,2]);
end

end
%% Start Simulation of the Control Loop

x = controlParams.x0;
dt = simulationParams.timeStep; %This is the timestep of the simulator
tend = simulationParams.timeFinal;


tic 
dt_anim = 1 / (simulationParams.slow_mo * simulationParams.frameRate);
next_anim = 0;

results.x_store = [];
results.t_store = [];   
results.u_store = [];
results.x_opt_store=  cell(1);
results.u_opt_store= cell(1);
%Creates empty matrices to store the state and control throughout the
%execution of the program

controlParams.k = 0;  %keeps track of which iteration its on.
flag = 1;
for t = 0:dt:tend
    if controlParams.Method == 'MPC'
        if t>=controlParams.Ts*controlParams.k%The sampling time of the controller is different than the sim timestep
            %Every time the simulation moves one more sampling period into
            %the future, the controller will recalculate and hold the new
            %value of u for an extended period of time.
            
            controlParams.k = controlParams.k+1; %This way, the controller is somewhat discretized and only updates erry now and then/
            
            %if (flag == 1)
            %    [u,results] = controlLaw(x,modelParams, controlParams,simulationParams,results); 
            %    %temporarilily
            %    flag = 0;
            %else
            %Keep using the previous control value (DISCRETE);
            %u = results.u_opt_store{1}(controlParams.k);
            %end
            
            % **If you just want to use the first collocation solution
            %uncomment the above lines and comment out the one right below **
            
        [u,results] = controlLaw(x,modelParams, controlParams,simulationParams,results);
        fprintf('t = %2.2f\r',t);
        end
        
        
    else
        u = controlLaw(x,modelParams, controlParams,simulationParams,results); %Executes the control Normally
    end
    
    results.x_store(end+1,:) = x;
    results.u_store(end+1) = u;   % Stores all of system information for this time step
    results.t_store(end+1) = t;
    
    if t~= 0
        xDotprev = xDot;
    else
        xDotprev = cartPoleDynamics(x,u,modelParams); %Essentially uses euler integration for just the first step.
    end
    
    xDot = cartPoleDynamics(x,u,modelParams); % Gets the change in the state from the FW dynamics
    %x = x+xDot*dt;% Euler Integration
    x = x+.5*(xDot+xDotprev)*dt; % trapezoidal integration of the state
    
    if t >= next_anim && simulationParams.animate == 1 % Only animate select frames to not lag out the computer.
            h_title.String = ['t = ' sprintf('%.2f',t)];

            frame1.Matrix = makehgtform('translate',[x(1) 0 0]);
            frame2.Matrix = makehgtform('zrotate',x(2));

            pause( t*simulationParams.slow_mo - toc )
            next_anim = t + dt_anim;
    end
    
    %if simulationParams.animate == 1  && x(1) > max(xlim())-.5 || x(1) <min(xlim())+.5
    %    %Readjusts the xaxis to keep cart on screen
    %    xlim([x(1)-5, x(1) + 5]);
    %end
    
end


if simulationParams.animate == 0 && simulationParams.animateEnd == 1 %Animates at the end
    tic
    i = 0;
    for t = 0:dt:tend
        i = i+1;
        if t >= next_anim
            % Only animate select frames to not lag out the computer.
            h_title.String = ['t = ' sprintf('%.2f',t)];

            frame1.Matrix = makehgtform('translate',[results.x_store(i,1) 0 0]);
            frame2.Matrix = makehgtform('zrotate',results.x_store(i,2));

            pause( t*simulationParams.slow_mo - toc )
            next_anim = t + dt_anim;
        end
    end
end

end