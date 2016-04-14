% forces_moments.m
%   Computes the forces and moments acting on the airframe. 

function out = forces_moments(uu, P)    

    out = zeros(4*P.num_agents,1); % initialize size of output

    persistent fm  % fm =>s struct that will help us distinguish data for each distinct agent
                   % fm(1) => data for agent 1, etc...  

    NN = 0;
    for i = 1:P.num_agents,
    %     fm(i).pn      = uu(NN+1);
    %     fm(i).pe      = uu(NN+2);
        fm(i).pd      = uu(NN+3);
    %     fm(i).u       = uu(NN+4);
    %     fm(i).v       = uu(NN+5);
    %     fm(i).w       = uu(NN+6);
    %     fm(i).phi     = uu(NN+7);
    %     fm(i).theta   = uu(NN+8);
    %     fm(i).psi     = uu(NN+9);
    %     fm(i).p       = uu(NN+10);
    %     fm(i).q       = uu(NN+11);
    %     fm(i).r       = uu(NN+12);

        NN = NN + 12;
    end

    for i = 1:P.num_agents,
        thrust  = uu(NN+1);
        l       = uu(NN+2);
        m       = uu(NN+3);
        n       = uu(NN+4);

        fz = thrust;% + ground_effect(fm(i).pd);
        fm(i).forces_moments = [fz; l; m; n];

        NN = NN + 4;
    end

    NN = 0;
    for i = 1:P.num_agents,
        out(NN+1:NN+4) = fm(i).forces_moments;
        NN = NN + 4;
    end

end

function out = ground_effect(pd)
    z = -pd;
    
    if z > 1
        out = 0;
        return;
    end
    
    if z < 0.2
        z = 0.2;
    end
    a =  -55.3516;
    b =  181.8265;
    c = -203.9874;
    d =   85.3735;
    e =   -7.6619;
    
    avg = 0.0;
    sig = 1.19;
    noise = sig*randn + avg;
    
    out = a*z*z*z*z + b*z*z*z + c*z*z + d*z + e + noise;

end

