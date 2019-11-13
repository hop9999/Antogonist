function J = tsa_jacobian(theta, x, tsa)
r = tsa.L;
L = tsa.r;
J = theta * r^2 / (L-x);
end