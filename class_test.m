clear; clc;
TSA(1) = classTSA;%(1e-3, 160e-3);
for i=1:2
TSA(1).L = 160e-3;
a.r = 1e-3;
a.I = 1e-7;
a.ks = 6e4;
a.bs = 300;
% a.SetParams(r, L);
% a.SetParams(r,L,I,ks,bs);
figure(1)
subplot(3,1,1)
theta = linspace(0,100);
plot(theta, a.FindX(theta))
title('Fwd Kin')
subplot(3,1,2)
x = linspace(0,5e-2);
plot(x, a.FindTheta(x))
title('Inv Kin')
subplot(3,1,3)
yyaxis left
plot(theta, a.FindJacobianMixed(theta,x))
yyaxis right
plot(theta, a.FindJacobianInvMixed(theta,x))
title('Jacobian and Its Inverse')