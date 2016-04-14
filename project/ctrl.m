function u = ctrl(in,P)
    u_ref   = in(1);
    w_ref   = in(2);
    u_actual = in(3);
    w_actual = in(4);
    t        = in(5);
    
    s = [u_actual; w_actual];
    s_ref = [u_ref; w_ref];
    e = s - s_ref;
    persistent theta_hat;
    if isempty(theta_hat)
       theta_hat = 0;
    end
    
    phi = [-P.lambda*P.v_cx+u_actual*P.v_cz;...
           -P.lambda*P.v_cy+w_actual*P.v_cz];
    theta_hat = theta_hat + P.Ts * (P.gamma_theta * e' * phi);
%     u_actual
%     w_actual
    denominator = P.lambda^2 + u_actual^2 + w_actual^2;
    L_w_inv = [0 P.lambda/denominator;...
               -P.lambda/denominator 0;...
               w_actual/denominator -u_actual/denominator];
%     u = [(L_w_inv * (-P.theta * phi - P.alpha * s))' 0];     % ideal controller with known z
    u = [(L_w_inv*(-theta_hat*phi - P.alpha * s))' theta_hat];
end