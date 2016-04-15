function out=underwater_ctrl(in,P)
    pr    = in(1);
    p     = in(2);
    v     = in(3);
    t     = in(4);

    persistent thetahat;
    persistent differentiator;
    persistent error_d1;
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
            T = 0;
            p_ref = pr;
            thetahat = zeros(P.num_adaptive_param,1);

        case 4, % sliding mode
            T = 0;
            p_ref = pr;
            thetahat = zeros(P.num_adaptive_param,1);
            
        case 5, % adaptive
            T = 0;
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