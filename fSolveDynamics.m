function [system, t, x] = fSolveDynamics(system, control, t_end)
    % resave
    len = length(system.tsa);
    state = zeros(1,4*len)';
    for ii=1:len
        state((1:4)+4*(ii-1)) = system.tsa(ii).state(1,:);
    end
    % calculate control
%     u = ...
    % MAIN SOLVER
    [t, x] = ode45(@(t,state) system_dynamics(system, t, state, control),...
        linspace(0,t_end,1e3), state);
    % re-save variables for convenience
    for ii=1:len
        system.tsa(ii).state    = x(:,(1:4)+4*(ii-1));
    end    
    
%     [dstate, T] = system_dynamics(system, t, x, control);
%     for ii=1:len
%         system.tsa(ii).dstate   = dstate((1:4)+4*(ii-1),:);        
%         system.tsa(ii).T        = T(ii,:);  
%     end
end

