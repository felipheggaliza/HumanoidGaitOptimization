function cm = computeCM(var)
% computes the center of mass in the var-direction for the five link robot

mass = 50; % [kg]
m1 = 0.061 * mass; % [kg]
m2 = 0.1 * mass; % [kg]
m3 = 0.678 * mass; % [kg]
m4 = m2; % [kg]
m5 = m1; % [kg]
 
m = [m1 m2 m3 m4 m5]; % [kg]

cm = ( m(1)*var(1) + m(2)*var(2) + m(3)*var(3) + m(4)*var(4) + m(5)*var(5) ) / sum(m); % [kg]


end

