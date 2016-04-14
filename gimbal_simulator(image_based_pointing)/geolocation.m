% geolocation
%
% compute location of target given position in camera
% input is 
%    uu(1:3)   - camera data (eps_x, eps_y, eps_s)
%    uu(4:15)  - MAV states
%    uu(16:17) - gimbal azimuth, elevation 
%    uu(18)    - time
%
% output is 
%    tn - estimated North postion of target
%    te - estimated East position of target
%    L  - range to target
%
% modified 05/06/2010 - Randy Beard
%
function out = geolocation(in,P)
    
    persistent g;
    
    % process inputs
    NN = 0;
    for i = 1:P.num_agents,
        g(i).eps_x     = -in(1+NN); % x-pixel
        g(i).eps_y     = -in(2+NN); % y-pixel
        g(i).eps_s     = in(3+NN); % pixel size
        NN = NN + 3;
    end
    for i = 1:P.num_agents,
        g(i).pn        = in(1+NN);
        g(i).pe        = in(2+NN);
        g(i).pd        = -in(3+NN);
        g(i).u         = in(4+NN);
        g(i).v         = in(5+NN);
    %     g(i).w         = in(6+NN);
        g(i).phi       = in(7+NN);
        g(i).theta     = in(8+NN);
        g(i).psi       = in(9+NN);
    %     g(i).p         = in(10+NN);
    %     g(i).q         = in(11+NN);
    %     g(i).r         = in(12+NN);
        NN = NN + 12;
    end
    for i = 1:P.num_agents,
        g(i).az        = in(1+NN); % gimbal azimuth angle
        g(i).el        = in(2+NN); % gimbal elevation angle
        NN = NN + 2;
    end
    t         = in(1+NN); % times
    
    if t==0,    %initialize estimate and covariance
        for i = 1:P.num_agents,
            g(i).xhat_a = [g(i).pn;g(i).pe;-g(i).pd];
            g(i).P_a = diag([100,100,100]);
        end
    end
    
    
    NN = 0;
    for i = 1:P.num_agents,
        pdot_mav = [g(i).u;g(i).v];
    %     pdot_mav = [Vg*cos(chi);Vg*sin(chi)];

        Q_a = diag([1,1,1]);
        R_a = diag([P.sigma_measurement_n,P.sigma_measurement_e,P.sigma_measurement_h]);

        N = 10;
        % prediction step
        for j=1:N,
            p_obj_hat = g(i).xhat_a(1:2);
            L_hat = g(i).xhat_a(3);
            p_mav_hat = [g(i).pn;g(i).pe];

            g(i).xhat_a = g(i).xhat_a + (P.Tout/N)*[0;0;-(p_obj_hat-p_mav_hat)'*pdot_mav/L_hat];
            A_a = [0,0,0;...
                   0,0,0;...
                   -pdot_mav'/L_hat,(p_obj_hat-p_mav_hat)'*pdot_mav/(L_hat^2)...
                   ];
            g(i).P_a = g(i).P_a+(P.Tout/N)*(A_a*g(i).P_a+g(i).P_a*A_a'+Q_a);
        end

        if ((g(i).eps_x ~= -9999) && (g(i).eps_y ~= -9999)), 
            % measurement updates
            p_obj_hat = g(i).xhat_a(1:2);
            L_hat = g(i).xhat_a(3);
            p_mav_hat = [g(i).pn;g(i).pe];

            Rot_c_to_g = [0 1 0;0,0,1;1,0,0]';
            l_cup_c = [g(i).eps_x;g(i).eps_y;P.f]/sqrt(g(i).eps_x^2+g(i).eps_y^2+P.f^2);

            ell_i = (Rot_v_to_b(g(i).phi,g(i).theta,g(i).psi)'*Rot_b_to_g(g(i).az,g(i).el)'*Rot_c_to_g*l_cup_c);
            h_a = [g(i).xhat_a(1),g(i).xhat_a(2),0]' - L_hat*ell_i;
            C_a = [[1,0;0,1;0,0],-ell_i];

            %estimate north position of target
            L_a = g(i).P_a*C_a(1,:)'*inv((R_a(1,1)+C_a(1,:)*g(i).P_a*C_a(1,:)'));
            g(i).P_a = (eye(3)-L_a*C_a(1,:))*g(i).P_a;
            g(i).xhat_a = g(i).xhat_a+L_a*(g(i).pn-h_a(1));
            %estimate east position of target
            L_a = g(i).P_a*C_a(2,:)'*inv((R_a(2,2)+C_a(2,:)*g(i).P_a*C_a(2,:)'));
            g(i).P_a = (eye(3)-L_a*C_a(2,:))*g(i).P_a;
            g(i).xhat_a = g(i).xhat_a+L_a*(g(i).pe-h_a(2));
            %estimate bearing to target
            L_a = g(i).P_a*C_a(3,:)'*inv((R_a(3,3)+C_a(3,:)*g(i).P_a*C_a(3,:)'));
            g(i).P_a = (eye(3)-L_a*C_a(3,:))*g(i).P_a;
            g(i).xhat_a = g(i).xhat_a+L_a*(g(i).pd-h_a(3));   
        end

        tn   = g(i).xhat_a(1);
        te   = g(i).xhat_a(2);
        L    = g(i).xhat_a(3);
    
        %create output
        out(NN+1:NN+3) = [tn; te; L];
        NN = NN + 3;

    end
    % end geolocation code 
    %--------------------------------------
    
end

%%%%%%%%%%%%%%%%%%%%%%%
function R = Rot_v_to_b(phi,theta,psi)
    % Rotation matrix from body coordinates to vehicle coordinates

    Rot_v_to_v1 = [...
        cos(psi), sin(psi), 0;...
        -sin(psi), cos(psi), 0;...
        0, 0, 1;...
        ];
    
    Rot_v1_to_v2 = [...
        cos(theta), 0, -sin(theta);...
        0, 1, 0;...
        sin(theta), 0, cos(theta);...
        ];
    
    Rot_v2_to_b = [...
        1, 0, 0;...
        0, cos(phi), sin(phi);...
        0, -sin(phi), cos(phi);...
        ];
    
    R = Rot_v2_to_b * Rot_v1_to_v2 * Rot_v_to_v1;

end

%%%%%%%%%%%%%%%%%%%%%%%
function R = Rot_b_to_g(az,el)
    % Rotation matrix from body coordinates to gimbal coordinates
    Rot_b_to_g1 = [...
        cos(az), sin(az), 0;...
        -sin(az), cos(az), 0;...
        0, 0, 1;...
        ];

    Rot_g1_to_g = [...
        cos(el), 0, -sin(el);...
        0, 1, 0;...
        sin(el), 0, cos(el);...
        ];

    R = Rot_g1_to_g * Rot_b_to_g1;

end
