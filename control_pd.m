function u = control_pd(system,state,t)
q = zeros(1,2);
dq = zeros(1,2);
x = zeros(1,2);
dx = zeros(1,2);
for ii = 1:length(system.tsa)
    q(ii) = state(4*(ii-1) + 1);
    dq(ii) = state(4*(ii-1) + 2);
    x(ii) = state(4*(ii-1) + 3);
    dx(ii) = state(4*(ii-1) + 4);
end
a10 = 0.05;
a1 = 0.01;
w1 = 2*pi;

a20 = 0.07;
a2 = 0.02;
w2 = 3*pi;

x1_des = a10 + a1*sin(w1*t);
dx1_des = a1*w1*cos(w1*t);
ddx1_des = -a1*w1^2*sin(w1*t);
dddx1_des = -a1*w1^3*cos(w1*t);
ddddx1_des = a1*w1^4*sin(w1*t);

x2_des = a20 + a2*sin(w2*t);
dx2_des = a2*w2*cos(w2*t);
ddx2_des = -a2*w2^2*sin(w2*t);
dddx2_des = -a2*w2^3*cos(w2*t);
ddddx2_des = a2*w2^4*sin(w2*t);

w = 800;
Kp = w^2*10e-7;
Kd = 2*w*10e-7;

u1_fb = Kp*(x1_des - x(1)) + Kd*(dx1_des - dx(1));
u2_fb = Kp*(x2_des - x(2)) + Kd*(dx2_des - dx(2));
u1 = u1_fb;
u2 = u2_fb;

u = [u1,u2];
end