function out = ctrl(in,P)
    e_i        = in(1);
    phi_ref    = in(2);
    p_ref     = in(3);
    phi     = in(4);
    p       = in(5);
    phi_cmd   = in(6);
    t       = in(7);    
    
    
    
    persistent theta_hat;
    if t==0
        theta_hat = [0; 0; 0];
    else
        e = [e_i; phi; p] - [e_i; phi_ref; p_ref];
        theta_hat = theta_hat + P.Ts * (P.gamma_theta * [abs(phi)*p abs(p)*p phi^3]' * e' * P.P_ref * P.B);
    end
    x = [e_i; phi; p];
    u_bl = -P.K*x;
    u_ad = -theta_hat'*[abs(phi)*p abs(p)*p phi^3]';
    u = u_bl + u_ad;
    out = [u_bl; u_ad; u];
end