function [t,x] = integrator(q0,opt)
% Function used to call the dstate for the five link robot and perform the simulation using the ode45 solver
% inputs:
% q0 is a 10x1 array with the initial conditions for the 10 state variables
% opt is a struct generated after loading the min_tf.mat file
% Last modification: 30/03/2014 (by Feliphe G. Galiza)

tResult = [];
xResult = [];
tStep = [opt.ti'];

for index = 2:numel(tStep)
  % Integrate
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