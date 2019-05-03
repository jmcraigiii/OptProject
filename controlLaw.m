function [u,results] = controlLaw(x, modelParams,controlParams,simulationParams,results)
% Function which outputs the control to the system (Force on the Cart)
% given the current state (x) and other control Parameters.
% x = [ position; angle; velocity; omega]


switch (controlParams.Method)
    case 'nothing'
        u = -100*x(3); %Friction applied at the joint of rotation
    case 'PD'
%% PD CONTROL
    %PD control
        % Proportionally corrects error and has a 'damping' term on the
        % rate of change of error.
    xDesired = controlParams.xDesired;
    Kp = controlParams.Constants.Kp;
    Kd = controlParams.Constants.Kd;

    u = Kp*(wrapToPi(xDesired(3)-x(3))) - Kd*(x(4)); 
    %PD control of the states.
    %Here, the position of the block is changed.  the velocity is set to
    %zero.


    
    case 'MPC'
%% MPC
    % Model Predictive Control
    [u, results] = getMPC(x, modelParams, controlParams,results);
    

    otherwise
        fprintf(' \"%s\" is Not a Valid Control Method\r', controlParams.Method);
        u = 0;
        return
end
end