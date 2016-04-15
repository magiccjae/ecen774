function u=ctrl(in,P)
    yr    = in(1);
    y     = in(2);
    ydot  = in(3);
    z     = in(4);
    t     = in(5);
    
    mode = 2;  % a is known
               % a is unknown
%     persistent xi3_derivative;
%     persistent xi3_d1;
%     if t==0
%         xi3_derivative = 0;
%         xi3_d1 = 0;
%     end
   
    if mode == 1
        beta0 = 0.3;
        beta1 = 1;
        beta2 = 1;

        x1 = y-yr;
        x2 = ydot;
        x3 = z;

        x1_dot = x2;
        x2_dot = -P.a*x2*abs(x2) + x3;

        xi2 = -beta0 * sat(x1);
        xi2_dot = 0;
        xi2_ddot = 0;
        xi3 = -x1 + P.a*x2*abs(x2) + xi2_dot - beta1*sat(x2 - xi2);
        if x2 >= 0
            xi3_dot = -x1_dot + 2*P.a*x2*x2_dot + xi2_ddot - beta1*x2_dot;
        else
            xi3_dot = -x1_dot - 2*P.a*x2*x2_dot + xi2_ddot - beta1*x2_dot;
        end

        u = -x2 + xi2 + x3 + xi3_dot - beta2*sat(x3 - xi3);
        
    elseif mode==2        
        beta0 = 0.1;
        beta1 = 1;
        beta2 = 1;

        x1 = y-yr;
        x2 = ydot;
        x3 = z;

        x1_dot = x2;
        x2_dot = -P.a*x2*abs(x2) + x3;

        xi2 = -beta0 * sat(x1);
        xi2_dot = 0;
        xi2_ddot = 0;
        if x2-xi2~=0
            xi3 = -x1 + xi2_dot - beta1*sat(x2 - xi2) - 2*abs((x2-xi2)*x2)*abs(x2)/(x2-xi2);
        else
            xi3 = -x1 + xi2_dot - beta1*sat(x2 - xi2) - 2*abs((x2-xi2)*x2)*abs(x2)/(x2-xi2+1);
        end
%         error = xi3 - xi3_d1;
%         xi3_derivative = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*xi3_derivative...
%             + 2 / (2*P.tau+P.Ts) * (error);
%         xi3_d1 = xi3;
       
%         u = -x2 + xi2 + x3 + xi3_derivative - beta2*sat(x3 - xi3);        
        u = -x2 + xi2 + x3 - beta2*sat(x3 - xi3);        
        
    end
    
end

function out = sat(in)
    if     in>1,  out=1;
    elseif in<-1, out=-1;
    else          out=in;
    end
end
