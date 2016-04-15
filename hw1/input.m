clear; close all; clc;

warning('off','all')

% 2.1-(1)
f = @(t,X) [-X(1)^3+X(2); X(1)-X(2)^3];
phase_portrait(f,-3,3,40);

% 2.1-(3)
f = @(t,X) [X(2); 2*X(1)+3*X(1)^2+X(1)^3-X(2)];
phase_portrait(f,-3,1,20);

% 2.1-(7)
f = @(t,X) [X(2); -(X(1)+X(2))/(X(1)+2)];
phase_portrait(f,-2,2,20);


%2.8 - u = 0
f = @(t,X) [X(2); -0.5*X(1)+1.5*X(2)];
phase_portrait(f,-2,2,20);

%2.8 - u = 0.9x1-3.2x2
f = @(t,X) [X(2); -0.5*X(1)+1.5*X(2)+0.5*(0.9*X(1)-3.2*X(2))];
phase_portrait(f,-2,2,20);

%2.8 - u = 0
sat = @(x) (x>1)*1+(x<-1)*(-1);
f = @(t,X) [X(2); -0.5*X(1)+1.5*X(2)+0.5*sat(0.9*X(1)-3.2*X(2))];
phase_portrait(f,-2,2,40);

warning('on','all')