function out = compute_coefficient(T, P)

   phi_0 = [1 0 0 0 0 0 0 0; 0 0 0 0 1 0 0 0];
   phidot_0 = [0 1 0 0 0 0 0 0; 0 0 0 0 0 1 0 0];
   phi_T = [1 T T^2 T^3 0 0 0 0; 0 0 0 0 1 T T^2 T^3];
   phidot_T = [0 1 2*T 3*T^2 0 0 0 0; 0 0 0 0 0 1 2*T 3*T^2];
   
   C = [phi_0; phidot_0; phi_T; phidot_T] \ [P.rx_0; P.ry_0; P.vx_0; P.vy_0; P.rx_T; P.ry_T; P.vx_T; P.vy_T];
   out = C;
end