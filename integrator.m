% tspan = [0 9]; % set time interval
% 
% q0 = [-0.1897 -0.0002 -0.0132 -0.3928 0.0002 -0.1897   -0.0002   -0.0132   -0.3928    0.0002]; 
% 
% % dstate evaluates r.h.s. of the ode
% 
% [T,Y]=ode45(@dstate,tspan,q0)
% 
% plot(T,Y(:,5))
% grid

function [t,x] = integrator(q0,opt)

tResult = [];
xResult = [];
tStep = [opt.ti'];

for index = 2:numel(tStep)
  % Integrate:
  u1 = opt.u(index - 1,1);
  u2 = opt.u(index - 1,2);
  u3 = opt.u(index - 1,3);
  u4 = opt.u(index - 1,4);
  u5 = opt.u(index - 1,5);
  u = [u1 u2 u3 u4 u5];
  af   = @(t,x) dstate(t, x, u);
  t    = tStep(index-1:index);
  [t, x] = ode45(af, t, q0);
    % Collect the results:
    tResult = cat(1, tResult, t);
    xResult = cat(1, xResult, x);
    % Final value of x is initial value for next step:
    q0 = x(end, :);
end
  
plot(t,x)
end