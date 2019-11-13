function u = control_fl(system,state,t)
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
    
    q1_des = (2*system.tsa(1).L*x1_des - x1_des^2)^(1/2)/system.tsa(1).r;
    dq1_des = -(2*x1_des*dx1_des - 2*system.tsa(1).L*dx1_des)/(2*system.tsa(1).r*(2*system.tsa(1).L*x1_des - x1_des^2)^(1/2));
    ddq1_des = -(2*x1_des*dx1_des - 2*system.tsa(1).L*dx1_des)^2/(4*system.tsa(1).r*(2*system.tsa(1).L*x1_des - x1_des^2)^(3/2)) - (2*dx1_des^2 + 2*x1_des*ddx1_des - 2*system.tsa(1).L*ddx1_des)/(2*system.tsa(1).r*(2*system.tsa(1).L*x1_des - x1_des^2)^(1/2));
    
    q2_des = (2*system.tsa(2).L*x2_des - x2_des^2)^(1/2)/system.tsa(2).r;
    dq2_des = -(2*x2_des*dx2_des - 2*system.tsa(2).L*dx2_des)/(2*system.tsa(2).r*(2*system.tsa(2).L*x2_des - x2_des^2)^(1/2));
    ddq2_des = -(2*x2_des*dx2_des - 2*system.tsa(2).L*dx2_des)^2/(4*system.tsa(2).r*(2*system.tsa(2).L*x2_des - x2_des^2)^(3/2)) - (2*dx2_des^2 + 2*x2_des*ddx2_des - 2*system.tsa(2).L*ddx2_des)/(2*system.tsa(2).r*(2*system.tsa(2).L*x2_des - x2_des^2)^(1/2));

    T1_des = system.k*(x(1) + x(2) - system.tsa(1).x0  - system.tsa(1).x0)/2*2 + system.tsa(1).m*ddx1_des + system.tsa(1).b_x*dx(1);
    T2_des = system.k*(x(1) + x(2) - system.tsa(1).x0  - system.tsa(2).x0)/2*2 + system.tsa(2).m*ddx2_des + system.tsa(2).b_x*dx(2);
    
    q1_des  = FindTheta(system.tsa(1), x1_des);
    q2_des  = FindTheta(system.tsa(2), x2_des);
    
    w = 800;
    Kp = w^2*10e-7;
    Kd = 2*w*10e-7;
    
    J1 = FindJacobianMixed(system.tsa(1), q1_des, x1_des);
    J2 = FindJacobianMixed(system.tsa(2), q2_des, x2_des);
    
    dk_dtheta_1 = stiffnes_derivative(q(1), system.tsa(1));
    x_c_1     = system.tsa(1).FindX(q(1));
    delta_x_1 = x_c_1 - x(1);
    
    dk_dtheta_2 = stiffnes_derivative(q(2), system.tsa(2));
    x_c_2     = system.tsa(2).FindX(q(2));
    delta_x_2 = x_c_2 - x(2);
    
    u1_ff = J1*T1_des + system.tsa(1).b_theta*dq1_des + 1/2*dk_dtheta_1.*delta_x_1.^2 + system.tsa(1).I*ddq1_des;
    u2_ff = J2*T2_des + system.tsa(2).b_theta*dq2_des + 1/2*dk_dtheta_2.*delta_x_2.^2 + system.tsa(2).I*ddq2_des;

    u1_fb = Kp*(x1_des - x(1)) + Kd*(dx1_des - dx(1));
    u2_fb = Kp*(x2_des - x(2)) + Kd*(dx2_des - dx(2));
    u1 = u1_ff+u1_fb;
    u2 = u2_ff+u2_fb;

    u = [u1,u2];
end