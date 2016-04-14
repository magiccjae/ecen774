function [sys,x0,str,ts,simStateCompliance] = mav_dynamics(t,x,u,flag,P)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 12*P.num_agents;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12*P.num_agents;
sizes.NumInputs      = 4*P.num_agents;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = zeros(12*P.num_agents,1); %initialize size of x0
NN = 0;
for i = 1:P.num_agents,
    x0(NN+1:NN+12)  = [P.pn0(i); P.pe0(i); P.pd0(i); P.u0(i); P.v0(i); P.w0(i); P.phi0(i); P.theta0(i); P.psi0(i); P.p0(i); P.q0(i); P.r0(i)];
    NN = NN + 12;
end
%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%

% Double Checked March 4
function sys=mdlDerivatives(t,x,uu,P)
    
    NN = 0;
    for i = 1:P.num_agents,
        pn(i)      = x(NN+1);
        pe(i)      = x(NN+2);
        pd(i)      = x(NN+3);
        u(i)       = x(NN+4);
        v(i)       = x(NN+5);
        w(i)       = x(NN+6);
        phi(i)     = x(NN+7);
        theta(i)   = x(NN+8);
        psi(i)     = x(NN+9);
        p(i)       = x(NN+10);
        q(i)       = x(NN+11);
        r(i)       = x(NN+12);
        
        NN = NN + 12;
    end
    
    NN = 0;
    for i = 1:P.num_agents,
        F(i)       = uu(NN+1);
        T_phi(i)   = uu(NN+2);
        T_theta(i) = uu(NN+3);
        T_psi(i)   = uu(NN+4);
    end
    
    % state derivative vector
    x_dot = zeros(12*P.num_agents,1); %% initialize size of output
    NN = 0;
    for i = 1:P.num_agents,
        % calculate sines and cosines once for convenience and efficiency
        cp = cos(phi(i));
        sp = sin(phi(i));
        ct = cos(theta(i));
        st = sin(theta(i));
        tt = tan(theta(i));
        cs = cos(psi(i));
        ss = sin(psi(i));

         % translational kinematics model
        tkm = [ct*cs sp*st*cs-cp*ss cp*st*cs+sp*ss;
               ct*ss sp*st*ss+cp*cs cp*st*ss-sp*cs;
               -st   sp*ct          cp*ct];

        x_dot(NN+1:NN+3) = tkm*[u(i); v(i); w(i)];

        % translational dynamics model
        tdm1 = [r(i)*v(i)-q(i)*w(i); 
                p(i)*w(i)-r(i)*u(i); 
                q(i)*u(i)-p(i)*v(i)];

        tdm2 = [-P.gravity*st; 
                 P.gravity*ct*sp; 
                 P.gravity*ct*cp];

        x_dot(NN+4:NN+6) = tdm1 + tdm2 + (1/P.mass)*[-P.mu*u(i); -P.mu*v(i); -F(i)];

        % rotational kinematic model
        rkm = [1 sp*tt cp*tt;
               0 cp    -sp;
               0 sp/ct cp/ct];

        x_dot(NN+7:NN+9) = rkm*[p(i); q(i); r(i)];

        % rotational dynamics model
        rdm1 = [(P.Jyy-P.Jzz)/P.Jxx*q(i)*r(i);
                (P.Jzz-P.Jxx)/P.Jyy*p(i)*r(i);
                (P.Jxx-P.Jyy)/P.Jzz*p(i)*q(i)];

        rdm2 = [1/P.Jxx 0       0;
                0       1/P.Jyy 0;
                0       0       1/P.Jzz];

        x_dot(NN+10:NN+12) = rdm1 + rdm2*[T_phi(i); T_theta(i); T_psi(i)];
        
        NN = NN + 12;
    end

sys = x_dot;
% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

sys = x;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
