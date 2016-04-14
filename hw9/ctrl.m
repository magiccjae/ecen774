% Differential flatness example for ECE 774
%
% Modified: 4/15/2014 - R. Beard
%
%
function u_tilde = ctrl(uu,P)
    xr = uu(1:6);
    x  = uu(7:12);
    
    rx_ref = xr(1);
    ry_ref = xr(2);
    psi_ref = xr(3);
    v_ref = xr(4);
    r_ref = xr(5);
    a_ref = xr(6);    
    
    A = [...
         0 0 -v_ref*sin(psi_ref) cos(psi_ref) 0 0;
         0 0 v_ref*cos(psi_ref) sin(psi_ref) 0 0;
         0 0 -1 0 1 0;
         0 0 0 -0.1 0 1;
         0 0 0 0 0 0;
         0 0 0 0 0 0
         ];
     
    B = [0 0; 0 0; 0 0; 0 0; 1/P.Jz 0; 0 1/P.m];
    
    [K,S,E] = lqr(A,B,eye(6),eye(2));

    x_tilde = x - xr;
    u_tilde = -K*x_tilde;
    
%     A = [...
%         0, 0, -xr(4)*sin(xr(3)), cos(xr(3));...
%         0, 0,  xr(4)*cos(xr(3)), sin(xr(3));...
%         0, 0, 0, 0;...
%         0, 0, 0, 0;...
%         ];
%     B = [0, 0; 0, 0; 1, 0; 0, 1];
%     
%     K = place(A,B,[-P.a+j*P.a,-P.a-j*P.a,-P.b+j*P.b,-P.b-j*P.b]);
%     
%     u = -K*(x-wrap(xr,x));
end

% function out = wrap(in,x)
%     out = in;
%     while out(3)-x(3)>pi, out(3)=out(3)-2*pi; end
%     while out(3)-x(3)<-pi, out(3) = out(3)+2*pi; end
% end