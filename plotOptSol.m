function plotOptSol(k,modelParams,controlParams,simulationParams, results)
%Takes the desired iteration of the collocation and plots the solution it
%gave

startTime = (k-1)*controlParams.Ts;
animate = 1;

figure(4) % Plot of the States as a function of Time
    t = linspace(startTime, startTime+controlParams.Hz, (controlParams.Hz/controlParams.Ts)+1);

    plot(t, results.x_opt_store{k},'LineWidth',1.5)
    l = legend('$x$','$\theta$','$\dot{x}$','$\dot{\theta}$');
    l.Interpreter = 'Latex';

    xlabel('Time (s)','Interpreter','Latex');
    ylabel('State','Interpreter','Latex')
    set(gca,'FontSize',16)

figure(5) % Plot of the Control Input as a function of time

    plot(t, results.u_opt_store{k},'LineWidth',1.5);
    xlabel('Time (s)','Interpreter','Latex');
    ylabel('Force (N)','Interpreter','Latex');

    set(gca,'FontSize',16);
    
if animate == 1
next_anim = 0;
dt_anim = 1 / (simulationParams.slow_mo * simulationParams.frameRate);

  for i = 1
   figure(1)
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
  end  %Just sets up the figure plotting window
  
      tic
    i = 0;
    for t = linspace(startTime, startTime+controlParams.Hz, (controlParams.Hz/controlParams.Ts)+1)
        i = i+1;
        if t >= next_anim
            % Only animate select frames to not lag out the computer.
            h_title.String = ['t = ' sprintf('%.2f',t)];

            frame1.Matrix = makehgtform('translate',[results.x_opt_store{k}(i,1) 0 0]);
            frame2.Matrix = makehgtform('zrotate',results.x_opt_store{k}(i,2));
            pause( t*simulationParams.slow_mo - toc )
            next_anim = t + dt_anim;
        end
    end
    
end
    

end