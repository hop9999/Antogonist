function proj = tsa_projection(theta, tsa)
    r = tsa.r;
    L = tsa.L;
    proj = sqrt(1 - (theta*r/L)^2);
end
