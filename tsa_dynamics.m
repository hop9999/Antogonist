function [dstate, T] = tsa_dynamics(tsa, forces)
    F = forces.F;
    tau = forces.tau;

    theta   = tsa.state(1);
    dtheta  = tsa.state(2);
    x       = tsa.state(3);
    dx      = tsa.state(4);

    x_c     = tsa.FindX(theta);
    J       = tsa.FindJacobianMixed(theta,x);
    delta_x = x_c - x;

    T = tsa_tension(tsa.state, tsa);
    dk_dtheta = stiffnes_derivative(theta, tsa);
    %dk_dtheta = 0;

    m = tsa.m;
    b_x = tsa.b_x;
    b_theta = tsa.b_theta;
    I = tsa.I;

    ddtheta = (tau - J.*T - b_theta*dtheta - 1/2*dk_dtheta.*delta_x.^2)./I;
    ddx = (T - F - b_x.*dx)./m;
    dstate = [dtheta, ddtheta, dx, ddx]';
end

