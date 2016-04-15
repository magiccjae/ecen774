function out=underwater_ctrl(in,P)
    pr    = in(1);
    p     = in(2);
    v     = in(3);
    t     = in(4);

    persistent thetahat

    switch P.control_selection_flag,
        case 1, % PD control
            p_ref = pr;
            e = p_ref-p;
            thetahat = zeros(P.num_adaptive_param,1);
            T = P.kp*e;            
            
        case 2, % backstepping
            T = 0;
            p_ref = pr;
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