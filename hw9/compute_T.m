function T = compute_T(P)
    T = fmincon(@(x) x, 50, [], [], [], [], 0, inf, @(x) constrain(x,P));
end

function [c, ceq] = constrain(u, P)
   
   C = compute_coefficient(u, P);
   c = [norm([2*C(3); 2*C(7)])-P.acc_max;...
        norm([2*C(3) + 6*C(4)*u; 2*C(7) + 6*C(8)*u])-P.acc_max
        ];
   ceq = [];
end