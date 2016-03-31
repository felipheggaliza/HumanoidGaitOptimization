function xcm = computeCMx(x)
% computes the center of mass in the x-direction for the five link robot

mass = 50; % [kg]
m1 = 0.061 * mass; % [kg]
m2 = 0.1 * mass; % [kg]
m3 = 0.678 * mass; % [kg]
m4 = m2; % [kg]
m5 = m1; % [kg]
 
m = [m1 m2 m3 m4 m5]; % [kg]

xcm = ( m(1)*x(1) + m(2)*x(2) + m(3)*x(3) + m(4)*x(4) + m(5)*x(5) ) / sum(m); % [kg]


end

