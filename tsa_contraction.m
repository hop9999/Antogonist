function x_c = tsa_contraction(theta, tsa)
r = tsa.L;
L = tsa.r;
x_c = L - sqrt(L^2 - (theta*r)^2);
end