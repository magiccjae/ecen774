function u = ctrl(in,P)
    q_cmd    = in(1);
    q     = in(2);
    t     = in(3);
    
    a_ref = -4;
    b_ref = 4;
    
    persistent q_ref;
    persistent k_q_hat;
    persistent k_q_cmd_hat;
    persistent theta_hat;
    if t==0
        q_ref = 0;
        k_q_hat = 0;
        k_q_cmd_hat = 0;
        theta_hat = 0;
    else
        q_ref = q_ref + P.Ts * (a_ref*q_ref + b_ref*q_cmd);
        e = q - q_ref;        
        k_q_hat = k_q_hat + P.Ts * (P.gamma_q * q * e);
        k_q_cmd_hat = k_q_cmd_hat + P.Ts * (P.gamma_q_cmd * q_cmd * e);
        theta_hat = theta_hat + P.Ts * (-P.gamma_theta * tanh(360/pi*q) * e);
    end
    
    u = k_q_hat*q + k_q_cmd_hat*q_cmd - theta_hat*tanh(360/pi*q);
end