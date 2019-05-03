function plotLast2(k,modelParams,controlParams,simulationParams, results)
%Takes the desired iteration of the collocation and plots its optimal
%control as well as the control of the previous iteration.

startTime = (k-1)*controlParams.Ts;

figure(4) % Plot of the States as a function of Time

var = 'control';

switch var
    case 'control'
    t = linspace(startTime, startTime+controlParams.Hz, (controlParams.Hz/controlParams.Ts)+1);
    plot(t, results.u_opt_store{k},'k','LineWidth',1.5)
    hold on     
    tlast = t-controlParams.Ts;
     plot(tlast, results.u_opt_store{k-1},'b--','LineWidth',1.5)

     l = legend('$u_k$', '$u_{k-1}$');
     l.Interpreter = 'Latex';
     l.Location = 'best';

    xlabel('Time (s)','Interpreter','Latex');
    ylabel('Force (N)','Interpreter','Latex')
    set(gca,'FontSize',16)
    
    
    case 'state'

    clf
    t = linspace(startTime, startTime+controlParams.Hz, (controlParams.Hz/controlParams.Ts)+1);
    tlast = t-controlParams.Ts;

    plot(t, results.x_opt_store{k},'LineWidth',1.5)
    hold on
    plot(tlast, results.x_opt_store{k-1},'--','LineWidth',1.5)
    l = legend('$x_k$','$\theta_k$','$\dot{x}_k$','$\dot{\theta}_{k+1}$', ...
        '$x_{k+1}$','$\theta_{k+1}$','$\dot{x}_{k+1}$','$\dot{\theta}_{k+1}$');
    l.Interpreter = 'Latex';
    l.Location = 'bestoutside';

    xlabel('Time (s)','Interpreter','Latex');
    ylabel('State','Interpreter','Latex')
    set(gca,'FontSize',16)
end