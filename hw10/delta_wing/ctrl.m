function u = ctrl(in,P)
    phi_ref    = in(1);
    p_ref     = in(2);
    phi     = in(3);
    p       = in(4);
    phi_cmd   = in(5);
    t       = in(6);    

    x = [phi; p];
    x_ref = [phi_ref; p_ref];
    
    persistent k_x_hat;
    persistent k_r_hat;
    persistent theta_hat;
    if t==0
        k_x_hat = [0; 0];
        k_r_hat = 0;
        theta_hat = [0; 0; 0];
    else
        e = x - x_ref;
        k_x_hat = k_x_hat + P.Ts * (-P.gamma_x * x * e' * P.P * P.B);
        k_r_hat = k_r_hat + P.Ts * (-P.gamma_r * phi_cmd * e' * P.P * P.B);
        theta_hat = theta_hat + P.Ts * (P.gamma_theta * [abs(phi)*p abs(p)*p phi^3]' * e' * P.P * P.B);
    end
    tellme = [k_x_hat'*x k_r_hat*phi_cmd -theta_hat'*[abs(phi)*p; abs(p)*p; phi^3]]
    u = k_x_hat'*x + k_r_hat*phi_cmd - theta_hat'*[abs(phi)*p; abs(p)*p; phi^3];
end