function uout = magnetic_levitation_controller(uu)
    x1 = uu(1);
    x2 = uu(2);
    x3 = uu(3);
    
    mode = 2; % 1 for linear controller
              % 2 for back stepping
              % 3 for sliding mode
    if mode==1
        K = [-1.5625 -2 2];
        uout = -K * ([x1; x2; x3]-[1; 0; 1]) + 1;
    
    elseif mode==2    
        if x1 ~= -1
            f_x = 8 * x3^2 * x2 / (1 + x1)^3;
            phi_x = -x3 + 2 * x2 * x3 / (1 + x1)^2;
        else
            f_x = 8 * x3^2 * x2 / (1 + x1 + .01)^3;
            phi_x = -x3 + 2 * x2 * x3 / (1 + x1 + .01)^2;
        end
        if x1 ~= -5/3 && x1 ~= -1
            g_x = -8 * x3 / ((5 + 3 * x1) * (1 + x1));
        else
            g_x = -8 * x3 / ((5 + 3 * x1) * (1 + x1 + .01));
        end

        if x3 ~= 0 && g_x ~= 0
            gx_inv = 1 / g_x;
        else
            gx_inv = 1 / (g_x + 0.01);
        end

        k1 = 1;%.2;
        k2 = 1;%.41;
        k3 = 1;%.1;

        z1 = x1 - 1;
        z2 = x2;
        z3 = 1 - 4 * x3 ^ 2 / (1 + x1)^2;

        z1_dot = x2;
        z2_dot = 1 - 4 * x3 ^ 2 / (1 + x1)^2;
        
        xi2 = -k1 * z1;
        xi2_dot = -k1 * z1_dot;
        xi2_ddot = -k1 * z2_dot;
        xi3 = -z1 + xi2_dot - k2 * (z2 - xi2);
        xi3_dot = -z1_dot + xi2_ddot - k2 * (z2_dot - xi2_dot);

        uout = -phi_x + gx_inv * (-z2 + xi2 - f_x + xi3_dot - k3 * (z3 - xi3));
    
    elseif mode==3
        if x1 ~= -1
            f_x = 8 * x3^2 * x2 / (1 + x1)^3;
            phi_x = -x3 + 2 * x2 * x3 / (1 + x1)^2;
        else
            f_x = 8 * x3^2 * x2 / (1 + x1 + .01)^3;
            phi_x = -x3 + 2 * x2 * x3 / (1 + x1 + .01)^2;
        end
        if x1 ~= -5/3 && x1 ~= -1
            g_x = -8 * x3 / ((5 + 3 * x1) * (1 + x1));
        else
            g_x = -8 * x3 / ((5 + 3 * x1) * (1 + x1 + .01));
        end

        if x3 ~= 0 && g_x ~= 0
            gx_inv = 1 / g_x;
        else
            gx_inv = 1 / (g_x + 0.01);
        end

        beta0 = 0.5;
        beta1 = 0.4;
        beta2 = 0.1;

        z1 = x1 - 1;
        z2 = x2;
        z3 = 1 - 4 * x3 ^ 2 / (1 + x1)^2;

        z1_dot = x2;
        z2_dot = 1 - 4 * x3 ^ 2 / (1 + x1)^2;
        
        xi2 = -beta0 * sat(z1);
        xi2_dot = 0;
        xi2_ddot = 0;
        xi3 = -z1 + xi2_dot - beta1 * sat(z2 - xi2);
        xi3_dot = -z1_dot + xi2_ddot - beta1 * z2_dot;

        uout = -phi_x + gx_inv * (-z2 + xi2 - f_x + xi3_dot - beta2 * sat(z3 - xi3));
    end                
end

function out=sat(u)
    if u > 1
        out = 1;
    elseif u < -1
        out = -1;
    else
        out = u;
    end
end