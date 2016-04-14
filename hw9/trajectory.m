function out = trajectory(u, P)
    t = u(1);
    
    y_traj = [1 t t^2 t^3 0 0 0 0; 0 0 0 0 1 t t^2 t^3] * P.C;
    ydot_traj = [0 1 2*t 3*t^2 0 0 0 0; 0 0 0 0 0 1 2*t 3*t^2] * P.C;
    yddot_traj = [0 0 2 6*t 0 0 0 0; 0 0 0 0 0 0 2 6*t] * P.C;
    ydddot_traj = [0 0 0 6 0 0 0 0; 0 0 0 0 0 0 0 6] * P.C;
        
    rx = y_traj(1);
    ry = y_traj(2);
    rx_dot = ydot_traj(1);
    ry_dot = ydot_traj(2);
    rx_ddot = yddot_traj(1);
    ry_ddot = yddot_traj(2);
    rx_dddot = ydddot_traj(1);
    ry_dddot = ydddot_traj(2);
    
    psi = atan2(ry_dot,rx_dot);
    tellme = [rx_dot ry_dot psi*180/pi]
    v = sqrt(rx_dot^2 + ry_dot^2);
    r = (rx_dot*ry_ddot - ry_dot*rx_ddot)/(rx_dot^2 + ry_dot^2) + psi;
    a = (rx_dot*rx_ddot + ry_dot*ry_ddot)/v + 0.1*v;
    
    tau = P.Jz * ((rx_dot * (ry_ddot + ry_dddot) - ry_dot * (rx_ddot + rx_dddot)) / (rx_dot^2 + ry_dot^2) ...
          - (2*rx_dot*rx_ddot + 2*ry_dot*ry_ddot)*(rx_dot*ry_ddot - ry_dot*rx_ddot) / (rx_dot^2 + ry_dot^2)^2);
    F = P.m * ((rx_ddot^2 + rx_dot*rx_dddot + ry_ddot^2 + ry_dot*ry_dddot) / v ...
        - ((rx_dot*rx_ddot + ry_dot*ry_ddot)^2) / ((rx_dot^2 + ry_dot^2)^1.5) + 0.1*(-0.1*v+a));
    
    x_r = [rx, ry, psi, v, r, a];
    u_r = [tau, F];
    out = [u_r x_r];
end

