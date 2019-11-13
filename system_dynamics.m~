function [dstate, system] = system_dynamics(system, t, state, control)
    k = system.k;
    for ii = 1:length(system.tsa)
        tsa(ii) = system.tsa(ii);
        ind = (1:4)+4*(ii-1);
        tsa(ii).state = state(ind);
        x(ii) = tsa(ii).state(3) - tsa(ii).x0;        
    end
    control = control_fl(system,state,t);
    for ii = 1:length(system.tsa)        
        ind = (1:4)+4*(ii-1);
        forces(ii).F = k*(x(1)+x(2));
        forces(ii).tau = control(ii);
%         [tsa(ii).dstate, tsa(ii).T] = tsa_dynamics(tsa(ii), forces(ii));
%         [dstate(ind,:), T(ii,:)] = tsa_dynamics(tsa(ii), forces(ii));
        [dstate(ind),tens] = tsa_dynamics(tsa(ii), forces(ii));
    end
    system.T = k*(x(1)+x(2));
    dstate = dstate';
%     dstate = [tsa(1).dstate, tsa(2).dstate]';
%     T = [tsa(1).T, tsa(2).T]';
end

