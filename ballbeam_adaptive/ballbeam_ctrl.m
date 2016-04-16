function out=ballbeam_ctrl(in,P)
    z_d   = in(1);
    z     = in(2);
    theta = in(3);
    t     = in(4);
    
    % use a digital differentiator to find zdot and thetadot
    persistent zdot
    persistent z_d1
    persistent thetadot
    persistent theta_d1
    % reset persistent variables at start of simulation
    if t<P.Ts,
        zdot        = 0;
        z_d1        = 0;
        thetadot    = 0;
        theta_d1    = 0;
    end
    zdot = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*zdot...
        + 2/(2*P.tau+P.Ts)*(z-z_d1);
    thetadot = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*thetadot...
        + 2/(2*P.tau+P.Ts)*(theta-theta_d1);
    z_d1 = z;
    theta_d1 = theta;


    [ubl, x] = baseline_ctrl(z_d,theta,z,thetadot,zdot,t,P);

    % build regressor vector
    Phi = [...  
        ubl;...  % baseline controller
        1;...
        theta;...
        z;...
        thetadot;...
        zdot;...
        z*zdot*thetadot/(P.m2*P.L^2/3+P.m1*z^2);...
        z*cos(theta)/(P.m2*P.L^2/3+P.m1*z^2);...
        z*thetadot^2;...
        sin(theta);...
        ];
    % initialize persistent variables
    persistent xref
    persistent thetahat
    if t<P.Ts,
        xref = zeros(5,1);
        thetahat = zeros(length(Phi),1);
    end
    % update reference model and adaptive parameters
    N = 10;
    for i=1:N,
        xref = xref + P.Ts/N*(P.Aref*xref + P.Bref*z_d + P.Lref_obsv*(x-xref));
        zref = P.Cref*xref;
        e = deadzone(x-xref, P.deadzone_limit);
        thetahat = sat( thetahat + P.Ts/N*Proj(thetahat,P.Gam*Phi*e'*P.Pref_obsv*P.B,P), P.proj_limit);
        uad = -thetahat'*Phi;    
    end
    uad = 0;
      
    u = ubl + uad;

    out = [u; zref; thetahat];
end

%-----------------------------------------------------
% baseline LQR controller
function [F,x] = baseline_ctrl(z_d,theta,z,thetadot,zdot,t,P)
    
    % integrator
    error = z - z_d;
    persistent integrator
    persistent error_d1
    % reset persistent variables at start of simulation
    if t<P.Ts==1,
        integrator  = -(P.m2*P.g/2)/P.K(5);
        error_d1    = 0;
    end
    integrator = integrator + (P.Ts/2)*(error+error_d1);
    error_d1 = error;
    
    % construct the state
    x = [theta; z; thetadot; zdot; integrator];
    F_tilde = -P.K*x;
    F_unsat = F_tilde;
    F = sat( F_unsat, P.Fmax);
    
    % integrator anti-windup
    ki = P.K(5);
    if ki~=0,
       integrator = integrator + P.Ts/ki*(F-F_unsat);
    end

end

%-----------------------------------------------------------------
% deadzone operator
function out = deadzone(in,limit)
    out = in;
    for i=1:length(in),
        if abs(in(i))<=limit(i),
            out(i) = 0;
        end
    end
end

%-----------------------------------------------------------------
% projection operator
function yout = Proj(theta,y,P)
    yout = y;
    delta = 0.1;
    for i=1:length(theta),
        if (theta(i)>(P.proj_limit(i)-delta)) & (y(i)>0)
            yout(i) = ((P.proj_limit(i)-theta(i))/delta)*y(i);
        end
        if (theta(i)<(-P.proj_limit(i)+delta)) & (y(i)<0),
            yout(i) = ((theta(i)-P.proj_limit(i))/delta)*y(i);
        end
    end
end


%-----------------------------------------------------------------
% saturation function
function out = sat(in,limit)
    out = in;
    for i=1:length(limit),
        if     in(i) > limit(i),      out(i) = limit(i);
        elseif in(i) < -limit(i),     out(i) = -limit(i);
        end
    end
end