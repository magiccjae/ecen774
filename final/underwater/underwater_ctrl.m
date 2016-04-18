function out=underwater_ctrl(in,P)
pr    = in(1);
p     = in(2);
v     = in(3);
t     = in(4);

persistent thetahat;
persistent differentiator;
persistent error_d1;
persistent integrator;
persistent xref

switch P.control_selection_flag,
    case 1, % PD control
        p_ref = pr;
        error = p_ref-p;
        if t==0
            differentiator = 0;
            error_d1 = 0;
        else
            differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator...
                + 2/(2*P.tau+P.Ts)*(error-error_d1);
        end
        error_d1 = error;
        
        T = P.kp*error + P.kd*differentiator;
        
        thetahat = zeros(P.num_adaptive_param,1);
        
    case 2, % backstepping
        p_ref = pr;
        z = p - p_ref;
        xi = -P.vc - P.k1*z;
        xi_dot = -P.k1*(v + P.vc);
        
        T = P.m/P.alpha*(P.mu1*v*abs(v)/P.m + P.mu2*v*abs(v)^3/P.m + xi_dot - z- P.k2*(v - xi));
        
        thetahat = zeros(P.num_adaptive_param,1);
        
    case 3, % feedback linearization
        p_ref = pr;
        z1 = p - p_ref;
        z2 = v + P.vc;
        
        nu = -P.k1*z1 - P.k2*z2;
        
        T = P.m/P.alpha * (P.mu1*v*abs(v)/P.m + P.mu2*v*abs(v)^3/P.m + nu);
        
        thetahat = zeros(P.num_adaptive_param,1);
        
    case 4, % sliding mode
        p_ref = pr;
        error = p - p_ref;
        if t==0
            differentiator = 0;
            error_d1 = 0;
        else
            differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator...
                + 2/(2*P.tau+P.Ts)*(error-error_d1);
        end
        error_d1 = error;
        s = differentiator + P.ke*error;
        p_x = abs(-50*v*abs(v)/0.1 - 15*v*abs(v)^3/0.1)+abs(20*(v+1));
        
        T = -(p_x + P.beta)*sat1(s/0.1);
        %             T = -(p_x + P.beta)*sign(s);
        
        thetahat = zeros(P.num_adaptive_param,1);
        
    case 5, % adaptive
        % baseline controller
        p_ref = pr;
        error = p - p_ref;
        if t==0
            integrator = 0;
            error_d1 = 0;
        else
            integrator = integrator + (P.Ts/2)*(error+error_d1);
        end
        error_d1 = error;
        x = [p; v; integrator];
        u_bl = -P.K*x;
        
        
        % build regressor vector
        Phi = [...
            u_bl;...  % baseline controller
            1;...
            p;...
            v;...
            v+P.vc;...
            -P.mu1*v*abs(v)/P.m;...
            -P.mu2*v*abs(v)^3/P.m;...
            P.alpha/P.m;...
            ];
        % initialize persistent variables
        if t<P.Ts,
            xref = zeros(3,1);
            thetahat = zeros(P.num_adaptive_param,1);
        end
        % update reference model and adaptive parameters
        N = 10;
        for i=1:N,
            xref = xref + P.Ts/N*(P.A_ref*xref + P.B_ref*pr + P.L_ref_obsv*(x-xref));
            zref = P.C_ref*xref;
            e = deadzone(x-xref, P.deadzone_limit);
            thetahat = sat( thetahat + P.Ts/N*Proj(thetahat,P.Gam*Phi*e'*P.P_ref_obsv*P.B,P), P.proj_limit);
            u_ad = -thetahat'*Phi;
        end
%         u_ad = 0;
        
        u = u_bl + u_ad;
        
        T = u;
        
end
out = [T; p_ref; thetahat];
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

%-----------------------------------------------------------------
% saturation function
function out = sat1(in)
out = in;
if     in > 1,      out = 1;
elseif in < -1,     out = -1;
end
end