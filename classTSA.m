classdef classTSA
    properties (SetAccess = public)
        r           {mustBeNumeric};
        L           {mustBeNumeric};        
        I           {mustBeNumeric};    % motor+coupler inertia
        k_s         {mustBeNumeric};    % tensile stiffness    
        b_s         {mustBeNumeric};    % tensile damping
        b_theta     {mustBeNumeric};    % rotational damping
        b_x         {mustBeNumeric};    % linear damping
        m           {mustBeNumeric};    % mass
        x0          {mustBeNumeric};    % initial pre-twist (meters)
        state                           % full state
        dstate                          % diff state for solver
        T           {mustBeNumeric};    % tension
        tau         {mustBeNumeric};    % tension
    end
    methods
        function obj    = SetParams(obj, r, L, I, ks, bs) % r L I            
            if nargin > 0
                obj.r = r;
                obj.L = L;
                if nargin > 3
                    obj.I = I;
                    obj.ks = ks;            
                    obj.bs = bs;
                end
            end
        end
        function x      = FindX(obj, theta)
            x = obj.L - sqrt(obj.L.^2 - (theta.*obj.r).^2);
        end        
        function theta  = FindTheta(obj, x)
            theta = sqrt(2*x.*obj.L - x.^2)./obj.r;
        end
        function J      = FindJacobianMixed(obj, theta, x)
            J = theta*obj.r^2./(obj.L - x);
        end
        function J_inv  = FindJacobianInvMixed(obj, theta, x)
            J_inv = (obj.L - x)./(theta*obj.r^2);
        end
        function proj   = FindProjection(obj, theta)
            proj = sqrt(1 - (theta.*obj.r./obj.L).^2);
        end            
    end
end


