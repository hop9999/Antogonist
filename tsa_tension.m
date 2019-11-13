function T = tsa_tension(state, tsa)
    k_s = tsa.k_s;
    b_s = tsa.b_s;

    theta   = state(1);
    dtheta  = state(2);
    x       = state(3);
    dx      = state(4);

    % x_c = tsa_contraction(theta, tsa);
    % J = tsa_jacobian(theta, x, tsa);
    x_c         = tsa.FindX(theta);
    J           = tsa.FindJacobianMixed(theta,x);
    delta_x     = x_c - x;
    ddelta_x    = J.*dtheta - dx;

    proj = tsa.FindProjection(theta);
    k = k_s*proj.^2;
    b = b_s*proj.^2;
    
    for ii = 1:length(delta_x)
        if delta_x(ii)>0
            T(ii) = k*delta_x(ii) + b*ddelta_x(ii);
        else
            T(ii) = 0;
        end
    end
end