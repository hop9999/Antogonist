function dk_dtheta = stiffnes_derivative(theta, tsa)
    k_s = tsa.k_s;
    dk_dtheta = k_s * theta * (tsa.r/tsa.L)^2;
end