clear all; close all;
clc;

% create struct with tsa params
for ii=1:2
    tsa(ii) = classTSA;
    tsa(ii).L = 0.25;
    tsa(ii).r = 0.7e-3;
    tsa(ii).I = 9e-7;
    tsa(ii).b_theta = 1.0e-5;
    tsa(ii).b_x = 1.0e1;
    tsa(ii).k_s = 6e4;
    tsa(ii).b_s = 300;
    tsa(ii).m = 0.3;
    tsa(ii).x0 = 0.035;
    tsa(ii).state = [0,0,tsa(1).x0+0.2,0]; % [theta, dtheta, x, dx]
end
% make second tsa equialient to first one
% tsa(2) = tsa(1);
tsa(1).state = [200,0,0.05,0]; 
tsa(2).state = [200,0,0.05,0]; 
% set a antogonistic system with two tsa's and spring
k = 200;
system.tsa(1) = tsa(1);
system.tsa(2) = tsa(2);
system.k = k;

% state_0 = [tsa(1).state, tsa(2).state];
control = [0, 0];
% t_span = [0, 3];
t_end = 5;

% [t,x] = ode45( @(t,state) system_dynamics(t,state,system,control),t_span,state_0);
[system,t,x] = fSolveDynamics(system, control, t_end);
% [t,tsa(1).x(:,1:4), tsa(2).x(:,1:4)] = ode45( @(t,state) system_dynamics(t, state, system, control),t_span, state_0);
% restore forces and tensions
%%
a10 = 0.05;
a1 = 0.01;
w1 = 2*pi;

a20 = 0.07;
a2 = 0.02;
w2 = 3*pi;

x1_des = a10 + a1*sin(w1*t);
x2_des = a20 + a2*sin(w2*t);

fig = figure;
fig.Name = "x";
hold on;
plot(t, system.tsa(1).state(:,3));
plot(t, system.tsa(2).state(:,3));
plot(t, x1_des);
plot(t, x2_des);
xlabel("t, s");
ylabel("pos, m");
legend('x1','x2','x1\_des','x2\_des')
fig = figure;
% fig.Name = "theta";
% hold on;
% plot(t, system.tsa(1).state(:,1));
% plot(t, system.tsa(2).state(:,1));
% fig = figure;
fig.Name = "error";
hold on;
plot(t, system.tsa(1).state(:,3) - x1_des);
plot(t, system.tsa(2).state(:,3) - x2_des);
xlabel("t, s");
ylabel("error, m");
legend('error1','error2')